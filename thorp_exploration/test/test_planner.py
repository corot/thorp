import math
import os

import cv2
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from thorp_exploration.planner import Camera, Grid, Planner, order_tour, segment_rooms

XTION = [(0.2, 0.2), (0.2, -0.2), (1.2, -0.6), (1.2, 0.6)]


def load_map(name):
    """ A map from thorp_navigation, as map_server loads it """
    maps = os.path.join(get_package_share_directory('thorp_navigation'), 'maps')
    with open(os.path.join(maps, name + '.yaml')) as f:
        info = yaml.safe_load(f)
    image = cv2.imread(os.path.join(maps, info['image']), cv2.IMREAD_GRAYSCALE)
    occupancy = (255 - image.astype(float)) / 255.0
    data = np.full(image.shape, -1, np.int8)
    data[occupancy > info['occupied_thresh']] = 100
    data[occupancy < info['free_thresh']] = 0
    return Grid(np.flipud(data), info['resolution'], info['origin'][:2])


def make_planner(grid):
    return Planner(grid, Camera(XTION, grid.resolution, 8), clearance=0.25, viewpoint_clearance=0.4,
                   room_area_min=2.0, room_area_max=20.0, viewpoint_spacing=0.3, coverage=0.95, min_view_area=0.25)


@pytest.fixture(scope='module')
def simple_rooms():
    planner = make_planner(load_map('simple_rooms'))
    planner.segment_rooms()
    return planner


def room_at(planner, x, y):
    return planner.labels[planner.grid.to_cell(x, y)]


def test_segment_simple_rooms(simple_rooms):
    areas = sorted(area for area, _ in simple_rooms.rooms.values())
    # six rooms of about 28 square meters, and the corridor joining them
    assert len(areas) == 7
    assert all(26.0 < area < 30.0 for area in areas[:6])
    assert areas[6] > 30.0


def test_segment_fun_house():
    grid = load_map('fun_house')
    labels, count = segment_rooms(grid, 2.0, 20.0)
    assert 4 <= count <= 7
    # every cell the robot can travel through is in some room
    assert np.all(labels[grid.clearance > 0.25] > 0)


def test_room_sequence_visits_all_rooms(simple_rooms):
    start = (1.5, 1.5)
    rooms = list(simple_rooms.rooms)
    sequence = simple_rooms.plan_room_sequence(start, rooms)
    assert sorted(sequence) == sorted(rooms)
    assert sequence[0] == room_at(simple_rooms, *start)


def test_room_exploration_sees_the_room(simple_rooms):
    start = (1.5, 1.5)
    room = room_at(simple_rooms, *start)
    poses = simple_rooms.plan_room_exploration(start, room)
    assert poses
    grid, camera = simple_rooms.grid, simple_rooms.camera
    seen = np.zeros(grid.shape, bool)
    for x, y, yaw in poses:
        row, col = grid.to_cell(x, y)
        assert simple_rooms.labels[row, col] == room
        assert grid.clearance[row, col] >= 0.4
        heading = int(round(yaw / (2.0 * math.pi / len(camera.headings)))) % len(camera.headings)
        rows, cols = np.nonzero(camera.visible(grid, row, col) & camera.views[heading])
        seen[rows + row - camera.radius, cols + col - camera.radius] = True
    room_cells = simple_rooms.labels == room
    assert (seen & room_cells).sum() >= 0.9 * room_cells.sum()


def test_whole_map_exploration():
    planner = make_planner(load_map('fun_house'))
    poses = planner.plan_room_exploration((8.5, 6.0), 0)
    assert len(poses) > 10
    assert all(planner.grid.clearance[planner.grid.to_cell(x, y)] >= 0.4 for x, y, _ in poses)


def test_errors(simple_rooms):
    with pytest.raises(ValueError, match='not segmented'):
        make_planner(simple_rooms.grid).plan_room_exploration((1.5, 1.5), 1)
    with pytest.raises(ValueError, match='unknown room'):
        simple_rooms.plan_room_exploration((1.5, 1.5), 99)
    with pytest.raises(ValueError, match='out of the map'):
        simple_rooms.plan_room_sequence((-5.0, 1.5), [1])


def test_order_tour():
    points = np.array([(0, 0), (3, 0), (1, 0), (4, 0), (2, 0)], float)
    distances = np.linalg.norm(points[:, None] - points[None, :], axis=2)
    assert order_tour(distances) == [2, 4, 1, 3]
    # nearest neighbor goes first to (-2, -3), and 2-opt finds the shortest tour, through (-4, -3) first
    points = np.array([(-2, -1), (1, 0), (-2, -3), (2, 2), (-4, -3)], float)
    distances = np.linalg.norm(points[:, None] - points[None, :], axis=2)
    assert order_tour(distances) == [4, 2, 1, 3]
