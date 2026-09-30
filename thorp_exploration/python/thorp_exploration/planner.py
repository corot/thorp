"""
Exploration planning on an occupancy grid: segment the free space into rooms, order the rooms to visit, and choose
and order the poses from where a camera sees all of a room.

Grids are indexed [row, column], with row 0 at the map origin, as OccupancyGrid's data. Only free cells can be
traveled and seen through; unknown cells count as obstacles.
"""

import math

import cv2
import numpy as np
from scipy import ndimage, sparse
from scipy.sparse import csgraph

KERNEL = np.ones((3, 3), np.uint8)


class Grid:
    """ An occupancy grid's free space, with each cell's distance to the nearest obstacle """

    def __init__(self, data, resolution, origin):
        self.free = np.asarray(data) == 0
        self.resolution = resolution
        self.origin = origin  # x, y of the grid's corner, on the map frame
        # pad with obstacles, so the free cells on the grid's edge don't look far from everything
        padded = np.pad(self.free, 1).astype(np.uint8)
        self.clearance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1] * resolution

    @property
    def shape(self):
        return self.free.shape

    def to_cell(self, x, y):
        return (math.floor((y - self.origin[1]) / self.resolution), math.floor((x - self.origin[0]) / self.resolution))

    def to_point(self, row, col):
        return self.origin[0] + (col + 0.5) * self.resolution, self.origin[1] + (row + 0.5) * self.resolution


class Roadmap:
    """ 8-connected graph of the cells with enough clearance to travel through, for geodesic distances """

    def __init__(self, grid, clearance):
        self.traversable = grid.clearance >= clearance
        self.shape = grid.shape
        cells = np.flatnonzero(self.traversable)
        self.node = np.full(grid.free.size, -1, np.int64)
        self.node[cells] = np.arange(cells.size)
        index = self.node.reshape(grid.shape)
        rows, cols = grid.shape
        sources, targets, weights = [], [], []
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
            source = index[:rows - dr, max(0, -dc):cols - max(0, dc)]
            target = index[dr:, max(0, dc):cols - max(0, -dc)]
            valid = (source >= 0) & (target >= 0)
            sources.append(source[valid])
            targets.append(target[valid])
            weights.append(np.full(valid.sum(), grid.resolution * math.hypot(dr, dc)))
        self.graph = sparse.coo_matrix((np.concatenate(weights), (np.concatenate(sources), np.concatenate(targets))),
                                       shape=(cells.size, cells.size)).tocsr()
        # the nearest traversable cell to each cell, for points too close to obstacles
        _, nearest = ndimage.distance_transform_edt(~self.traversable, return_indices=True)
        self.nearest = np.ravel_multi_index(tuple(nearest), grid.shape)

    def node_at(self, row, col):
        return self.node[self.nearest[row, col]]

    def distances(self, nodes):
        """ Distances from each of the given nodes to every node; infinite for those not connected """
        return csgraph.dijkstra(self.graph, directed=False, indices=nodes)

    def cells(self, nodes):
        """ Row and column of each node """
        return np.unravel_index(np.flatnonzero(self.traversable)[nodes], self.shape)


def segment_rooms(grid, area_min, area_max, step=0.05):
    """
    Label the free space into rooms (0 for no room), returning the labels and the rooms count. The free space is
    eroded until regions split off below area_max, each seeding a room, and the seeds are grown back over it. So
    area_max bounds a room's eroded area, smaller than the room's. Rooms smaller than area_min merge into a neighbor.
    """
    cell_area = grid.resolution ** 2
    seeds = np.zeros(grid.shape, np.uint16)
    count = 0
    for threshold in np.arange(step, grid.clearance.max(), step):
        n, regions = cv2.connectedComponents((grid.clearance > threshold).astype(np.uint8), connectivity=8)
        sizes = np.bincount(regions.ravel(), minlength=n)
        seeded = set(np.unique(regions[seeds > 0]))
        for region in range(1, n):
            if region not in seeded and sizes[region] * cell_area <= area_max:
                count += 1
                seeds[regions == region] = count

    labels = seeds
    while True:
        grown = cv2.dilate(labels, KERNEL)
        new = (labels == 0) & grid.free & (grown > 0)
        if not new.any():
            break
        labels[new] = grown[new]

    areas = np.bincount(labels.ravel(), minlength=count + 1) * cell_area
    for room in np.argsort(areas):
        if room == 0 or areas[room] == 0 or areas[room] >= area_min:
            continue
        mask = labels == room
        border = cv2.dilate(mask.astype(np.uint8), KERNEL).astype(bool) & ~mask
        neighbors = labels[border]
        neighbors = neighbors[neighbors > 0]
        target = np.bincount(neighbors).argmax() if neighbors.size else 0
        labels[mask] = target
        areas[target] += areas[room]
        areas[room] = 0

    ids = np.unique(labels[labels > 0])
    renumber = np.zeros(count + 1, np.int32)
    renumber[ids] = np.arange(1, ids.size + 1)
    return renumber[labels], ids.size


def room_center(grid, mask):
    """ The room's cell farthest from obstacles """
    return np.unravel_index(np.argmax(np.where(mask, grid.clearance, -1.0)), grid.shape)


def order_tour(distances):
    """
    Order to visit all the points from the first one traveling the least, given their distances matrix: nearest
    neighbor, improved with 2-opt. Returns the indices of the other points, in order.
    """
    n = len(distances)
    tour = [0]
    left = set(range(1, n))
    while left:
        closest = min(left, key=lambda j: distances[tour[-1]][j])
        tour.append(closest)
        left.remove(closest)
    improved = True
    while improved:
        improved = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                # reverse the tour between i and j; the path is open, so the last point has no successor
                gain = distances[tour[i - 1]][tour[i]] - distances[tour[i - 1]][tour[j]]
                if j + 1 < n:
                    gain += distances[tour[j]][tour[j + 1]] - distances[tour[i]][tour[j + 1]]
                if gain > 1e-9:
                    tour[i:j + 1] = tour[i:j + 1][::-1]
                    improved = True
    return tour[1:]


class Camera:
    """ The camera's field of view, a convex polygon on the robot frame, rasterized for a grid """

    def __init__(self, polygon, resolution, headings):
        polygon = np.asarray(polygon, float)
        if np.cross(polygon[1] - polygon[0], polygon[2] - polygon[1]) < 0:
            polygon = polygon[::-1]  # make it counterclockwise, for the inside test
        self.radius = int(math.ceil(np.linalg.norm(polygon, axis=1).max() / resolution))
        self.headings = np.arange(headings) * 2.0 * math.pi / headings

        # rays in all directions, sampled every half cell, as cell offsets
        max_range = self.radius * resolution
        rays = int(math.ceil(2.0 * math.pi * max_range / (0.5 * resolution)))
        angles = np.arange(rays)[:, None] * 2.0 * math.pi / rays
        ranges = (np.arange(int(math.ceil(max_range / (0.5 * resolution)))) + 1)[None, :] * 0.5 * resolution
        self.ray_rows = np.rint(ranges * np.sin(angles) / resolution).astype(int)
        self.ray_cols = np.rint(ranges * np.cos(angles) / resolution).astype(int)

        # the cells within the field of view for each heading, on a square around the robot
        offsets = np.arange(-self.radius, self.radius + 1) * resolution
        dx, dy = np.meshgrid(offsets, offsets)
        self.views = []
        for heading in self.headings:
            x = dx * math.cos(heading) + dy * math.sin(heading)
            y = -dx * math.sin(heading) + dy * math.cos(heading)
            inside = np.ones(dx.shape, bool)
            for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
                inside &= (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) >= 0
            self.views.append(inside)

    def visible(self, grid, row, col):
        """ The cells on sight from a cell, on the square around it; sight stops at the first cell not free """
        rows, cols = row + self.ray_rows, col + self.ray_cols
        inside = (rows >= 0) & (rows < grid.shape[0]) & (cols >= 0) & (cols < grid.shape[1])
        free = np.zeros(rows.shape, bool)
        free[inside] = grid.free[rows[inside], cols[inside]]
        on_sight = np.logical_and.accumulate(free, axis=1)
        square = np.zeros(self.views[0].shape, bool)
        square[self.ray_rows[on_sight] + self.radius, self.ray_cols[on_sight] + self.radius] = True
        return square


def plan_viewpoints(grid, camera, targets, candidates, spacing, coverage, min_gain):
    """
    Choose poses from where the camera sees the targets cells, greedily: each time, the pose seeing the most targets
    not seen yet, until seeing the coverage fraction of those seen from any candidate, or seeing less than min_gain
    new cells. Candidate poses are the cells farthest from obstacles on each spacing square of the candidates mask,
    with each of the camera's headings. Returns (row, column, heading) tuples.
    """
    step = max(1, int(round(spacing / grid.resolution)))
    cells = np.flatnonzero(candidates)
    if cells.size == 0:
        return []
    rows, cols = np.unravel_index(cells, grid.shape)
    squares = (rows // step) * (grid.shape[1] // step + 1) + cols // step
    order = np.lexsort((-grid.clearance[rows, cols], squares))
    _, first = np.unique(squares[order], return_index=True)
    positions = [(rows[i], cols[i]) for i in order[first]]

    target_index = np.full(grid.free.size, -1, np.int64)
    target_cells = np.flatnonzero(targets)
    target_index[target_cells] = np.arange(target_cells.size)
    r = camera.radius
    poses, row_cells = [], []
    for row, col in positions:
        visible = camera.visible(grid, row, col)
        for heading, view in zip(camera.headings, camera.views):
            seen_rows, seen_cols = np.nonzero(visible & view)
            seen = target_index[np.ravel_multi_index((seen_rows + row - r, seen_cols + col - r), grid.shape)]
            poses.append((row, col, heading))
            row_cells.append(seen[seen >= 0])
    sizes = [len(c) for c in row_cells]
    views = sparse.csr_matrix((np.ones(sum(sizes), np.float32), np.concatenate(row_cells),
                               np.concatenate(([0], np.cumsum(sizes)))), shape=(len(poses), target_cells.size))

    unseen = (np.asarray(views.sum(axis=0)).ravel() > 0).astype(np.float32)
    goal = (1.0 - coverage) * unseen.sum()
    chosen = []
    while unseen.sum() > goal:
        gains = views @ unseen
        best = int(np.argmax(gains))
        if gains[best] < min_gain:
            break
        chosen.append(poses[best])
        unseen[views.indices[views.indptr[best]:views.indptr[best + 1]]] = 0.0
    return chosen


class Planner:
    """
    Exploration planning on a map: segment it into rooms, and plan their sequence and the viewpoints to see each.
    Points are (x, y) on the map frame; errors raise ValueError, with the reason.
    """

    def __init__(self, grid, camera, clearance, viewpoint_clearance, room_area_min, room_area_max, viewpoint_spacing,
                 coverage, min_view_area):
        self.grid = grid
        self.camera = camera
        self.roadmap = Roadmap(grid, clearance)
        self.viewpoint_clearance = max(viewpoint_clearance, clearance)  # viewpoints must be on the roadmap
        self.room_area_min = room_area_min
        self.room_area_max = room_area_max
        self.viewpoint_spacing = viewpoint_spacing
        self.coverage = coverage
        self.min_view_cells = min_view_area / grid.resolution ** 2
        self.labels = None
        self.rooms = {}

    def segment_rooms(self):
        """ Segment the map into rooms; returns their ids, areas and centers """
        self.labels, count = segment_rooms(self.grid, self.room_area_min, self.room_area_max)
        areas = np.bincount(self.labels.ravel(), minlength=count + 1) * self.grid.resolution ** 2
        self.rooms = {room: (areas[room], self.grid.to_point(*room_center(self.grid, self.labels == room)))
                      for room in range(1, count + 1)}
        return [(room, area, center) for room, (area, center) in self.rooms.items()]

    def plan_room_sequence(self, start, rooms):
        """ Order the rooms to visit them all from start, traveling the least; leaves out those not reachable """
        for room in rooms:
            self.check_room(room)
        nodes = [self.start_node(start)] + [self.roadmap.node_at(*self.grid.to_cell(*self.rooms[r][1])) for r in rooms]
        distances = self.roadmap.distances(nodes)[:, nodes]
        reachable = [0] + [i for i in range(1, len(nodes)) if np.isfinite(distances[0, i])]
        order = order_tour(distances[np.ix_(reachable, reachable)])
        return [rooms[reachable[i] - 1] for i in order]

    def plan_room_exploration(self, start, room):
        """
        Poses (x, y, yaw) from where the camera sees all of the room, or of the whole map for room 0, reachable from
        start and ordered to travel the least
        """
        start_node = self.start_node(start)
        if room:
            self.check_room(room)
            area = self.labels == room
        else:
            area = self.grid.free
        reachable = np.zeros(self.grid.free.size, bool)
        reachable[np.flatnonzero(self.roadmap.traversable)[np.isfinite(self.roadmap.distances(start_node))]] = True
        candidates = reachable.reshape(self.grid.shape) & (self.grid.clearance >= self.viewpoint_clearance) & area
        viewpoints = plan_viewpoints(self.grid, self.camera, area & self.grid.free, candidates, self.viewpoint_spacing,
                                     self.coverage, self.min_view_cells)
        nodes = [start_node] + [self.roadmap.node[row * self.grid.shape[1] + col] for row, col, _ in viewpoints]
        order = order_tour(self.roadmap.distances(nodes)[:, nodes])
        return [self.grid.to_point(*viewpoints[i - 1][:2]) + (viewpoints[i - 1][2],) for i in order]

    def start_node(self, start):
        row, col = self.grid.to_cell(*start)
        if not (0 <= row < self.grid.shape[0] and 0 <= col < self.grid.shape[1]):
            raise ValueError('start point (%.2f, %.2f) is out of the map' % start)
        node = self.roadmap.node_at(row, col)
        if node < 0:
            raise ValueError('no free space to travel on the map')
        return node

    def check_room(self, room):
        if self.labels is None:
            raise ValueError('rooms not segmented yet')
        if room not in self.rooms:
            raise ValueError('unknown room %d; rooms go from 1 to %d' % (room, len(self.rooms)))
