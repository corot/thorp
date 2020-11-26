DEFAULT_PLANNER = 'GlobalPlanner'
DEFAULT_CONTROLLER = 'TEBPlanner'
FOLLOW_CONTROLLER = 'PoseFollower'
GET_PATH_RECOVERY = ['clear_global_cm']
EXE_PATH_RECOVERY = ['clear_local_cm', 'escape_distance', 'out_to_free_space']
MOVE_BASE_RECOVERY = ['clear_both_cms', 'escape_distance', 'out_to_free_space']
TIGHT_DIST_TOLERANCE = 0.035
TIGHT_ANGLE_TOLERANCE = 0.05  # ~3 deg
LOOSE_DIST_TOLERANCE = 0.25
LOOSE_ANGLE_TOLERANCE = 0.5  # ~30 deg
INF_ANGLE_TOLERANCE = float('inf')
APPROACH_DIST_TO_TABLE = 0.25  # close enough to properly detect tabletop objects but not for picking
CLEAR_TABLE_WAY_TIMEOUT = 15.0  # seconds; restore costmap after this time
PICKING_DIST_TO_TABLE = 0.12  # ideal picking distance to the table, as close as possible without risk
PLACING_HEIGHT_ON_TABLE = 0.005  # slightly above the table to avoid crashing the object against it
PLACING_HEIGHT_ON_TRAY = 0.03  # well above the tray to avoid colliding with (or ejecting) previously placed objects
MAX_ARM_REACH = 0.3
TRAY_SIDE_X = 0.15
TRAY_SIDE_Y = 0.15
TRAY_DEPTH = 0.02
TRAY_SLOT = 0.0375  # better a divisor of both tray sides
