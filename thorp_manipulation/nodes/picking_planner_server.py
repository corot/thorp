#!/usr/bin/env python

import rospy
import actionlib

import geometry_msgs.msg as geometry_msgs

from copy import deepcopy
from itertools import permutations

from thorp_toolkit.geometry import TF2, to_transform, translate_pose, transform_pose, create_2d_pose, distance_2d
from thorp_toolkit.transform import Transform
from thorp_toolkit.visualization import Visualization
from thorp_msgs.msg import ObjectToPick, PickingPlan, PickLocation, PlanPickingAction, PlanPickingResult


class PickingPlanner(actionlib.SimpleActionServer):
    """
    Group objects into picking locations and sort them to make a picking plan
    """
    def __init__(self):
        super().__init__('manipulation/plan_picking', PlanPickingAction, self.execute_cb, False)

        self.poses_viz = rospy.Publisher('manipulation/picking_poses', geometry_msgs.PoseArray, queue_size=1)
        self.start()

    def execute_cb(self, goal):
        picking_poses, closest_pose = self.surface_sides_poses(goal.robot_pose, goal.surface, goal.picking_dist)
        picking_locs = self.group_objects(goal.robot_pose, goal.objects, picking_poses, goal.planning_frame,
                                          goal.max_arm_reach,
                                          - (goal.approach_dist - goal.picking_dist),
                                          - (goal.detach_dist - goal.picking_dist))
        picking_plan = self.make_picking_plan(goal.robot_pose, picking_locs)
        result = PlanPickingResult()
        result.picking_plan.travelled_dist = self.traveled_dist(goal.robot_pose, picking_plan)
        result.picking_plan.locations = picking_plan
        self.set_succeeded(result)

    def surface_sides_poses(self, robot_pose, surface, distance):
        """
        Calculate the four locations around a rectangular surface at a given distance.
        :param robot_pose: current robot pose, expected on map frame
        :param surface: picking surface as a CollisionObject
        :param distance:
        :return: poses, closest_pose; all on map frame
        """
        length, width, _ = surface.primitives[0].dimensions
        p_x = distance + length / 2.0
        n_x = - p_x
        p_y = distance + width / 2.0
        n_y = - p_y
        surface_pose = geometry_msgs.PoseStamped(surface.header, surface.pose)
        surface_pose.header.stamp = rospy.Time()  # as we don't know how old is the observation
        surface_pose = TF2().transform_pose(surface_pose, surface_pose.header.frame_id, 'map')
        surface_tf = to_transform(surface_pose, 'surface_frame')
        TF2().publish_transform(surface_tf)
        surface_tf.transform.translation.z = 0.0
        closest_pose = None
        closest_dist = float('inf')
        from math import pi
        poses = [('p_x', create_2d_pose(p_x, 0.0, -pi, 'surface_frame')),
                 ('n_x', create_2d_pose(n_x, 0.0, 0.0, 'surface_frame')),
                 ('p_y', create_2d_pose(0.0, p_y, -pi/2.0, 'surface_frame')),
                 ('n_y', create_2d_pose(0.0, n_y, +pi/2.0, 'surface_frame'))]
        pose_array = geometry_msgs.PoseArray()  # for visualization
        sides_poses = {}
        for name, pose in poses:
            pose = transform_pose(pose, surface_tf)
            pose_array.poses.append(deepcopy(pose.pose))
            pose_array.poses[-1].position.z += 0.025  # raise over costmap to make it visible
            sides_poses[name] = pose
            dist = distance_2d(pose, robot_pose)
            if dist < closest_dist:
                closest_pose = pose
                closest_dist = dist
        pose_array.header = deepcopy(closest_pose.header)
        pose_array.poses.append(deepcopy(closest_pose.pose))
        pose_array.poses[-1].position.z += 0.05  # remark the closest pose with a double arrow
        self.poses_viz.publish(pose_array)
        return sides_poses, closest_pose

    def group_objects(self, robot_pose, objects, picking_poses, planning_frame, max_arm_reach, approach_offset, detach_offset):
        """
        Group detected objects reachable from each picking location (that is, within arm's reach).
        We first eliminate the locations without objects only reachable from there.
        If an object can be reached from two of the remaining locations, we choose
        the one that places the object closer to the robot arm.
        :param robot_pose:
        :param objects:
        :param picking_poses: list of picking locations on map frame
        :param max_arm_reach:
        :param planning_frame:
        :param approach_offset:
        :param detach_offset:
        :return: picking locations, a list of PickLocation objects
        """
        bfp_to_arm_tf = Transform.create(TF2().lookup_transform('base_footprint', planning_frame))  # base to arm tf
        map_to_fbp_tf = Transform.create(TF2().lookup_transform('map', 'base_footprint'))  # map to base
        pick_locs = []
        for name, picking_pose in picking_poses.items():
            # current distance from the robot (stored but not used by now)
            dist_from_robot = distance_2d(picking_pose, robot_pose)
            # apply base to arm tf, so we get arm pose on map reference for each location
            arm_pose_mrf = (Transform.create(picking_pose) * bfp_to_arm_tf).to_geometry_msg_pose_stamped()
            # detected objects poses are in arm reference, so their modulo is the distance to the arm
            objs = []
            for i, obj in enumerate(objects):
                # transform object pose from base to map frame, so we can compare distances to picking locations
                # we must limit max arm reach with our navigation tolerance when reaching the goal, as that will
                # be our probable picking pose, instead of the ideal one received as input
                obj_pose_mrf = (map_to_fbp_tf * Transform.create(obj.pose)).to_geometry_msg_pose_stamped()
                dist = distance_2d(obj_pose_mrf, arm_pose_mrf)  # both on map rf
                if dist <= max_arm_reach:
                    objs.append(ObjectToPick(obj.id, dist, obj_pose_mrf))
            if not objs:
                continue  # no objects reachable from here; keep going
            # sort objects by increasing distance from the arm; that should make picking easier,
            # as we won't hit closer objects when going over them (not 100% sure if this is true)
            objs = sorted(objs, key=lambda o: o.distance)

            # set also the approach and detach poses by translating by the respective offsets
            approach_pose = deepcopy(picking_pose)
            translate_pose(approach_pose, approach_offset, 'x')
            detach_pose = deepcopy(picking_pose)
            translate_pose(detach_pose, detach_offset, 'x')
            pick_locs.append(PickLocation(name, dist_from_robot, objs, arm_pose_mrf,
                                          approach_pose, picking_pose, detach_pose))
        if not pick_locs:
            rospy.loginfo("No reachable objects")
            return pick_locs

        # find the most efficient sequence of picking locations: try all permutations, sort and get the first
        possible_plans = []
        for perm in permutations(pick_locs):
            perm = list(perm)  # convert to list, so we can remove elements
            self.filter_pick_locs(perm)
            possible_plans.append(PickingPlan(self.traveled_dist(robot_pose, perm), perm))
        # sort by  1) fewer locations to visit  2) less travelled distance (in case of match)
        sorted_plans = sorted(possible_plans, key=lambda p: (len(p.locations), p.travelled_dist))
        sorted_plocs = sorted_plans[0].locations

        # remove duplicated objects from the location where they are at the longest distance to the arm
        objs_to_remove = []
        for ploc in sorted_plocs:
            dup_objs = 0
            for obj in ploc.objects:
                for other_pl in [pl for pl in sorted_plocs if pl != ploc]:
                    for other_obj in other_pl.objects:
                        if obj.name == other_obj.name:
                            if obj.distance >= other_obj.distance:
                                obj_to_rm = (ploc, obj)
                            else:
                                obj_to_rm = (other_pl, other_obj)
                            if obj_to_rm not in objs_to_remove:
                                objs_to_remove.append(obj_to_rm)
                            dup_objs += 1
        for ploc, obj in objs_to_remove:
            ploc.objects.remove(obj)  # TODO make a test for this and move the remove to within the loop (much simpler)
        # TODO: why I sort again? I may break the optimization done above!!!
        sorted_plocs = sorted(sorted_plocs, key=lambda pl: len(pl.objects))  # sort again after removing duplicates
        self.viz_pick_locs(sorted_plocs, max_arm_reach)
        return sorted_plocs

    @staticmethod
    def traveled_dist(robot_pose, pick_locs):
        """
        The total distance the robot will travel from its current location after visiting all pick locations
        """
        if not pick_locs:
            return 0.0
        total_dist = distance_2d(robot_pose, pick_locs[0].picking_pose)
        for ploc1, ploc2 in zip(pick_locs, pick_locs[1:]):
            total_dist += distance_2d(ploc1.picking_pose, ploc2.picking_pose)
        return total_dist

    @staticmethod
    def filter_pick_locs(pick_locs):
        """
        Traverse pick locations in order, discarding those whose objects are all present in other locations
        (that is, we can skip them when picking without missing any object)
        """
        pls_to_remove = []
        for ploc in pick_locs:
            objs_in_this = {o.name for o in ploc.objects}
            objs_in_others = set()
            for other_pl in [pl for pl in pick_locs if pl.name != ploc.name and pl not in pls_to_remove]:
                objs_in_others.update(o.name for o in other_pl.objects)
            if objs_in_this.issubset(objs_in_others):
                pls_to_remove.append(ploc)
        for ploc in pls_to_remove:
            pick_locs.remove(ploc)

    def viz_pick_locs(self, pick_locs, max_arm_reach):
        Visualization().clear_markers()
        for pl in pick_locs:
            color = Visualization.rand_color(0.4)
            Visualization().add_disc_marker(pl.arm_pose, [max_arm_reach * 2.0] * 2, color)

            text_pose = deepcopy(pl.arm_pose)
            text_pose.pose.position.z += 0.15
            Visualization().add_text_marker(text_pose, pl.name + ' ' + str(len(pl.objects)), 0.2, color)

            for obj in pl.objects:
                text_pose = deepcopy(obj.pose)
                text_pose.pose.position.z += 0.05
                Visualization().add_text_marker(text_pose, pl.name + ' ' + obj.name, 0.1, color)

        Visualization().publish_markers()

    def make_picking_plan(self, robot_pose, picking_locs):
        """
        Sort picking locations by visiting order, so we can iterate over the list for collecting all objects.
        :return: picking plan
        """
        picking_plan = []
        while picking_locs:
            closest_ploc = None
            closest_dist = float('inf')
            for ploc in picking_locs:
                dist = distance_2d(ploc.approach_pose, robot_pose)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_ploc = ploc
            picking_locs.remove(closest_ploc)
            picking_plan.append(closest_ploc)
            robot_pose = closest_ploc.approach_pose
            Visualization().add_text_marker(closest_ploc.approach_pose, str(len(picking_plan)) + ' ' + str(closest_dist))
            Visualization().publish_markers()
        return picking_plan


if __name__ == '__main__':
    rospy.init_node('picking_planner')
    pp = PickingPlanner()
    rospy.spin()
