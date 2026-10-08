"""
thorp_world's store: tables and objects in the map frame, updated by detections and by skills.
"""

import copy
import dataclasses

import yaml

from .entities import BELIEVED, DISTURBED, GRIPPER, MISSING, OBJECT, OBSERVED, OK, TABLE, TRAY, Entity, Pose
from .geometry import distance_to_table, normalize_table_yaw, on_table, planar_distance


class World:
    """
    What Thorp knows about the world, in the map frame. Detections update it with observations, matched against the
    entities already known so that each keeps its id; skills update it with the effects they believe they had.
    """

    def __init__(self, object_tolerance=0.05, table_tolerance=0.3, table_area_ratio=0.8, at_table_distance=0.3):
        """
        :param object_tolerance: how far from where it was a redetected object can be, to be the same one
        :param table_tolerance: how far off a known table's footprint a redetected table's center can be
        :param table_area_ratio: a known table takes an observation's pose and size only if its area is at least
          this ratio of the current one, as partial views give smaller tables
        :param at_table_distance: how far from a table's edge the robot can be, to be at it
        """
        self.object_tolerance = object_tolerance
        self.table_tolerance = table_tolerance
        self.table_area_ratio = table_area_ratio
        self.at_table_distance = at_table_distance
        self.docked_at = None
        self._entities = {}

    def get(self, entity_id):
        try:
            return copy.deepcopy(self._entities[entity_id])
        except KeyError:
            raise KeyError(f"no entity '{entity_id}'") from None

    def entities(self):
        return [copy.deepcopy(e) for e in self._entities.values()]

    def tables(self):
        return [copy.deepcopy(e) for e in self._entities.values() if e.kind == TABLE]

    def objects(self, support=None, status=None):
        return [copy.deepcopy(e) for e in self._entities.values() if e.kind == OBJECT and
                (support is None or e.support == support) and (status is None or e.status == status)]

    def observe_tables(self, observations, stamp):
        """
        Record tables seen at stamp. Each keeps the id of the known table it overlaps, or gets a new one.
        :return: the tables' ids, in the observations' order
        """
        ids = []
        for obs in observations:
            pose = Pose(obs.pose.x, obs.pose.y, obs.pose.z, normalize_table_yaw(obs.pose.yaw))
            table = self._match_table(pose, exclude=ids)
            if table is None:
                table = Entity(self._new_id(obs.type), TABLE, obs.type, tuple(obs.size), obs.color, pose)
                self._entities[table.id] = table
            elif obs.size[0] * obs.size[1] >= self.table_area_ratio * table.size[0] * table.size[1]:
                table.pose, table.size, table.color = pose, tuple(obs.size), obs.color
            table.provenance, table.stamp, table.status = OBSERVED, stamp, OK
            ids.append(table.id)
        return ids

    def observe_objects(self, surface, observations, stamp, robot_pose=None):
        """
        Record objects seen at stamp on a table, surface. Each keeps the id of the known object at about the same
        place, or gets a new one. Seen from robot_pose at that table, objects known on it, where the observed surface
        covers, that were not seen become missing; from further away, detection misses objects that are there.
        :return: the table's id, and the objects' ids in the observations' order
        """
        table_id = self.observe_tables([surface], stamp)[0]
        ids = []
        for obs in observations:
            entity = self._match_object(obs, exclude=ids)
            if entity is None:
                entity = Entity(self._new_id(obs.type), OBJECT, obs.type, tuple(obs.size))
                self._entities[entity.id] = entity
            entity.pose, entity.color = copy.copy(obs.pose), obs.color
            entity.support, entity.slot = table_id, None
            entity.provenance, entity.stamp, entity.status = OBSERVED, stamp, OK
            ids.append(entity.id)

        if robot_pose is None or self.table_at(robot_pose) != table_id:
            return table_id, ids
        # Compared with the surface as observed, not the table as known: a partial view says nothing of the rest
        seen = Entity('', TABLE, surface.type, tuple(surface.size),
                      pose=Pose(surface.pose.x, surface.pose.y, surface.pose.z, surface.pose.yaw))
        for entity in self._entities.values():
            if entity.kind == OBJECT and entity.support == table_id and entity.id not in ids and \
                    entity.status != MISSING and on_table(seen, entity.pose.x, entity.pose.y):
                entity.status = MISSING
        return table_id, ids

    def mark_picked(self, object_id, stamp):
        """The object is in the gripper."""
        self._set(object_id, stamp, pose=None, support=GRIPPER, slot=None, status=OK)

    def mark_placed(self, object_id, stamp, table_id, pose):
        """The object is on a table, at a map pose."""
        self._table(table_id)
        self._set(object_id, stamp, pose=copy.copy(pose), support=table_id, slot=None, status=OK)

    def mark_stowed(self, object_id, stamp, slot):
        """The object is on the tray, on a slot."""
        self._set(object_id, stamp, pose=None, support=TRAY, slot=slot, status=OK)

    def mark_disturbed(self, object_id, stamp):
        """A skill touched the object without the result it wanted, so it may have moved."""
        self._set(object_id, stamp, status=DISTURBED)

    def mark_lost(self, object_id, stamp):
        """
        The object left the gripper, for somewhere unknown, as when the gripper is cleared. Nothing changes for an
        object not believed in the gripper, as one a failed pickup left on its table.
        """
        if self._object(object_id).support == GRIPPER:
            self._set(object_id, stamp, pose=None, support=None, slot=None, status=MISSING)

    def dock(self, table_id):
        """The robot is parked against a table."""
        self._table(table_id)
        self.docked_at = table_id

    def undock(self):
        self.docked_at = None

    def table_at(self, robot_pose):
        """
        The table the robot is at: the one it docked at, if it's still within at_table_distance, or else the closest
        one that close, as the robot may stand at a table without docking, as in front of the bench's.
        """
        docked = self._entities.get(self.docked_at)
        if docked is not None and distance_to_table(docked, robot_pose.x, robot_pose.y) <= self.at_table_distance:
            return docked.id
        nearby = [(distance_to_table(t, robot_pose.x, robot_pose.y), t.id)
                  for t in self._entities.values() if t.kind == TABLE]
        nearby = [n for n in nearby if n[0] <= self.at_table_distance]
        return min(nearby)[1] if nearby else None

    def within_reach(self, object_id, arm_pose, reach):
        """Whether an object on a table is closer than reach to the arm's base."""
        entity = self._object(object_id)
        return entity.pose is not None and planar_distance(entity.pose, arm_pose) <= reach

    def to_dict(self):
        entities = []
        for entity in self._entities.values():
            data = dataclasses.asdict(entity)
            data['size'] = list(entity.size)
            entities.append(data)
        return {'docked_at': self.docked_at, 'entities': entities}

    def from_dict(self, data):
        self._entities = {}
        for item in data.get('entities') or []:
            item = dict(item)
            item['size'] = tuple(item['size'])
            item['pose'] = Pose(**item['pose']) if item.get('pose') else None
            entity = Entity(**item)
            self._entities[entity.id] = entity
        self.docked_at = data.get('docked_at')

    def save(self, path):
        with open(path, 'w') as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False)

    def load(self, path):
        with open(path) as f:
            self.from_dict(yaml.safe_load(f) or {})

    def _match_table(self, pose, exclude):
        best, best_distance = None, None
        for table in self._entities.values():
            if table.kind != TABLE or table.id in exclude or \
                    not on_table(table, pose.x, pose.y, margin=self.table_tolerance):
                continue
            distance = planar_distance(table.pose, pose)
            if best is None or distance < best_distance:
                best, best_distance = table, distance
        return best

    def _match_object(self, obs, exclude):
        """
        The known object closest to an observation, within object_tolerance, whatever type the observation gives:
        detection sometimes misidentifies an object, and the object keeps the type it was first seen as.
        """
        best, best_distance = None, None
        for entity in self._entities.values():
            # carried objects have no pose to compare, and aren't on the table being looked at anyway
            if entity.kind != OBJECT or entity.id in exclude or entity.pose is None:
                continue
            distance = planar_distance(entity.pose, obs.pose)
            if distance <= self.object_tolerance and (best is None or distance < best_distance):
                best, best_distance = entity, distance
        return best

    def _new_id(self, type):
        index = 1
        while f'{type} {index}' in self._entities:
            index += 1
        return f'{type} {index}'

    def _object(self, object_id):
        entity = self._entities.get(object_id)
        if entity is None or entity.kind != OBJECT:
            raise KeyError(f"no object '{object_id}'")
        return entity

    def _table(self, table_id):
        entity = self._entities.get(table_id)
        if entity is None or entity.kind != TABLE:
            raise KeyError(f"no table '{table_id}'")
        return entity

    def _set(self, object_id, stamp, **fields):
        entity = self._object(object_id)
        for name, value in fields.items():
            setattr(entity, name, value)
        entity.provenance, entity.stamp = BELIEVED, stamp
