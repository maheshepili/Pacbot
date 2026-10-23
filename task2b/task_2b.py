"""PB Task 2B solver.

Pipeline: read the official maze -> observe the ghost before the clock starts and learn its loop ->
choose the pellet order with the fastest predicted finish (ghost waits included) -> drive the plan
with gyro heading, odometry and wall landmarks -> recover and re-plan if something goes wrong.

Actions in a plan:
  ("run", n, dir, end_cell)  straight piece of n cells
  ("turn", dir)              stop-and-pivot on the cell centre
  ("arc", dir)               rolling quarter-circle corner (can be chained back to back)
  ("exit", metres)           straight through the exit gap

Arc safety layers: planner checks the (stretched) arc path against the walls, an entry gate downgrades
a risky arc to a pivot, inner/outer wall guards steer inside the arc, and a watchdog recovers from a
bad exit.

Run:  ./task_2b_launch      (terminal 1)
      python3 task_2b.py    (terminal 2)
"""

import bisect
import heapq
import json
import math
from functools import lru_cache

import paho.mqtt.client as mqtt

import maze_2b
from maze_2b import Maze

MQTT_HOST = "localhost"
MQTT_PORT = 1883

TOPIC_SENSORS = "pacbot/sensors"
TOPIC_PELLETS = "pacbot/pellets"
TOPIC_GHOST = "pacbot/ghost"
TOPIC_RESULT = "pacbot/result"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

# ----------------------------------------------------------------------------
# Geometry / robot constants
# ----------------------------------------------------------------------------
N = 15
PITCH = 0.22
CELL = PITCH
WALL_HALF = 0.005
STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))      # headings E, N, W, S as (drow, dcol)

WHEEL_RADIUS = 0.017
HALF_TRACK = 0.039
WHEEL_R = WHEEL_RADIUS
TRACK = 2.0 * HALF_TRACK
WHEEL_LIMIT = 30.0
WHEEL_MAX = WHEEL_LIMIT
TICKS_PER_REV = 360
TICK_RAD = 2.0 * math.pi / TICKS_PER_REV
M_PER_TICK = WHEEL_R * TICK_RAD
AXLE_BACK = 0.033                               # chassis centre is this far ahead of the axle

SENSOR_MAX = 1.0

# Pivot end limits; the lane controller removes the small residual in the next run.
TURN_TOL = 0.045
TURN_RATE_TOL = 0.12
TURN_SETTLE_TIME = 0.0

WALL_THICKNESS = 2 * WALL_HALF
INNER = PITCH - WALL_THICKNESS
SIDE_LATERAL = 0.005
SIDE_AHEAD = 0.036 + AXLE_BACK
FRONT_AHEAD = AXLE_BACK - 0.005
SIDE_CENTERED = INNER / 2 - SIDE_LATERAL        # side-sensor reading when centred in a corridor
FRONT_AT_CENTER = INNER / 2 - FRONT_AHEAD       # front reading with the axle on the cell centre
_WALL_END = CELL / 2 + WALL_THICKNESS / 2
GAP_TO_CENTER = CELL - _WALL_END + SIDE_AHEAD
WALL_TO_CENTER = _WALL_END + SIDE_AHEAD
SIDE_WALL = SIDE_CENTERED + 0.27 * CELL         # side reading below this = wall present
FRONT_TRUST = FRONT_AT_CENTER + 1.00 * CELL

EXIT_DISTANCE = 0.30
FWD_SPEED = 26.0            # rad/s; check "run:" log lines for the real m/s
APPROACH_MIN = 5.0
BRAKE_ACCEL = 3.0
EXIT_SPEED = 26.0
K_HEADING = 6.0
K_CENTER = 4.0
MAX_HEADING_OFFSET = math.radians(12)
MAX_OMEGA = 3.0
CORRECTION_MAX_YAW_ERR = math.radians(8)
TURN_ALPHA = 70.0
TURN_MIN = 2.5
TURN_MAX = 22.0
STALL_TIMEOUT = 45.0
SPEED_SLEW = 90.0
EVENT_MATCH = 0.105
FRONT_RANGE = 0.60
FRONT_GATE = 0.12
FRONT_MAX_YAW_ERR = math.radians(3)
FRONT_STOP = 0.045

SHORT_RUN_DIST = 0.15        # a run with less than this left before a pivot is capped to ARC_SPEED

# ---- rolling corners: quarter circle about the inner wall post of the junction cell ----------
USE_ARCS = True
ARC_R = 0.11                 # nominal radius (half a cell)
ARC_HALF = 0.11              # arc starts this far before the junction cell centre
ARC_SPEED = 17.0             # wheel rad/s through the arc
ARC_V = ARC_SPEED * WHEEL_R
K_ARC = 10.0                 # heading feedback gain inside the arc
ARC_STRETCH = 1.15           # path radius = ARC_R * this.  Real arcs track tighter than commanded (inner-wall hits at
                             # 0.040 m exit readings); >= 1.18 makes the planner reject half of all corners
ARC_EXTRA_DIST = 0.05        # arc may run this far past the nominal length waiting for the yaw
ARC_MARGIN = 0.020           # extra wall clearance demanded by the planner's arc check; 0.030 rejects 85% of corners
ARC_T = 0.20                 # planner cost of a rolling corner (s)
ARC_TIME = 0.78              # timeline model: seconds to roll through an arc
ARC_LEAD_T = 0.0             # the yaw does not coast, so no early stop of the rotation
ARC_DECEL = 1.0
ARC_LAG_DIST = 0.04          # start slowing this much earlier to cover speed-command lag
# Wall guards inside arcs: below the threshold the rotation is eased (inner) or increased (outer).
ARC_INNER_MIN = 0.085
ARC_INNER_GAIN = 40.0
ARC_INNER_PHASE = 1.0        # guards are active over this fraction of the arc length
ARC_OUTER_MIN = 0.075
ARC_OUTER_GAIN = 30.0
# Entry gate: shortly before an arc, heading and lateral offset are checked; a bad state turns the arc
# into a stop-and-pivot.
ARC_GATE_DIST = 0.15
ARC_ENTRY_YAW = math.radians(12)
ARC_ENTRY_LAT = 0.04
ARC_ENTRY_MIX = 1.6          # also unsafe when |lat|/ARC_ENTRY_LAT + |yaw|/ARC_ENTRY_YAW exceeds this
# Late check in the last 4 cm before the arc: only EXTREME errors downgrade the arc.  The lane controller
# deliberately swings the heading by 4-9 deg to re-centre, so a strict every-frame test turned 2/3 of all
# arcs into pivots (about 1 s lost each).
ARC_LATE_YAW = math.radians(14)
ARC_LATE_LAT = 0.045
ARC_ONE_WALL = 0.14          # a single side reading below this is trusted as a lateral reference
AFTER_ARC_OFFSET = math.radians(9)   # max lane-centring heading offset right after an arc
AFTER_ARC_DIST = 0.15
BAD_ARCS_PRESET = set()      # {(junction cell, heading in, heading out)} never planned as arcs
ARC_BACK_TO_BACK = True      # allow arcs with no straight piece between them (off after 2nd recovery)

# ---- recovery (watchdog) -----------------------------------------------------------------
RECOVER_YAW = math.radians(25)      # heading error inside a run that means "something hit"
RECOVER_ARC_YAW = math.radians(20)  # heading error at arc exit that triggers recovery
RECOVER_FRONT = 0.055               # front ToF this small while pushing = wedged
RECOVER_STUCK_T = 0.7
RECOVER_BACK_DIST = 0.06
RECOVER_BACK_SPEED = 6.0
RECOVER_MAX = 6
RECOVER_GRACE = 1.0                 # watchdog stays quiet this long after a recovery
RECOVER_TRUST_INTENDED = math.radians(70)
RELOC_TOL = 0.12                    # front reading must match the map's wall distance to re-localise
ARC_MAX_T = 2.4
ARC_FIX_TOL = 0.10                  # an arc that ran out of distance finishes the turn in place down to this
ARC_FIX_T = 1.5
ARC_WHEEL_CAP = 26.0                # outer wheel cap at higher arc speed

# ----------------------------------------------------------------------------
# Planner cost model (seconds) and ghost parameters
# ----------------------------------------------------------------------------
CELL_T = 0.73
TURN90_T = 1.50
TURN180_T = 2.35
PIVOT180_AFTER_ARC_T = 3.0   # a 180 deg pivot one cell after an arc starts off-centre and clips dead-end walls
PIVOT180_APPROACH = 9.0      # wheel rad/s over the last 0.25 m of a run that ends in a 180 deg pivot
UNSAFE_T = 60.0      # pivot that the sweep check says may clip a wall
ZONE_T = 3.0         # extra cost per cell inside the ghost's territory
ZONE_PATH_T = 12.0   # extra cost per cell on the ghost's own patrol path

GHOST_SAFE = 0.20    # centre-to-centre distance to keep from the ghost (touching is ~0.12)
DANGER_R = 0.30      # closer than this to the patrol path = ghost territory
WAIT_BACK = 0.12     # wait this far before the territory edge
MERGE_GAP = 0.5      # territory stretches closer than this are crossed in one go
GATE_LOOKAHEAD = 2.5
GATE_EVERY = 5       # frames between entry-window checks
V_RUN = 0.36         # m/s while moving in the timeline model (logs: 0.41 on straights, 0.24 in arcs / short runs)
RUN_DWELL = 0.20
RUN_TAIL = 0.25
TURN_T = 1.1
TURN180_TIME = 2.1
# Entry-window envelope: (turn time scale, speed scale, late start) for the slow and fast timelines and
# the distance to keep.  Every position between them must stay clear of the ghost.  Looser levels are
# used only after a long fruitless wait.
ENVELOPES = (
    ((1.5, 0.85, 0.5), (0.5, 1.15, 0.0), GHOST_SAFE),
    ((1.2, 0.92, 0.2), (0.7, 1.08, 0.0), GHOST_SAFE),
    ((1.0, 1.00, 0.0), (1.0, 1.00, 0.0), GHOST_SAFE - 0.03),
)
GATE_ESCALATE = (3.0, 7.0)   # seconds of waiting after which envelope level 1 / 2 is used (was 45 s)
GATE_COMMIT_DIST = 0.5       # once the window is open this close to the territory the crossing is committed
GATE_BRAKE_ACCEL = 0.6       # m/s^2: gentle slow-down towards the wait point
GATE_CREEP = 4.0             # wheel rad/s minimum while closing in on the wait point
GHOST_HORIZON = 1.5  # reactive fallback only
GHOST_GO_SPEED = 0.30
NO_GHOST_WAIT = 3.0
OBS_SOFT = 40.0      # accept a shorter confirmation of the loop after this many seconds
OBS_HARD = 70.0      # stop observing, use the reactive fallback

# ----------------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------------


def _mqtt_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def drive_vw(wheel_speed, omega):
    """Forward wheel speed (rad/s) + body yaw rate (rad/s) -> (left, right)."""
    diff = omega * TRACK / 2.0 / WHEEL_R
    return (clamp(wheel_speed - diff, -WHEEL_MAX, WHEEL_MAX),
            clamp(wheel_speed + diff, -WHEEL_MAX, WHEEL_MAX))


def clean(v):
    if v is None or math.isnan(v) or v < 0.0:
        return SENSOR_MAX
    return min(SENSOR_MAX, v)


def inside(row, col):
    return 0 <= row < N and 0 <= col < N


def cell_centre(cell):
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH


# ----------------------------------------------------------------------------
# Maze -> wall table
# ----------------------------------------------------------------------------
WALLS = {}
START_CELL = (14, 0)
EXIT_CELL = (7, 14)
EXIT_HEADING = 0


def load_walls(maze):
    """WALLS[row, col, d] is True when side d of the cell is blocked."""
    global START_CELL, EXIT_CELL
    WALLS.clear()
    for r in range(N):
        for c in range(N):
            for d in range(4):
                nr, nc = r + STEP[d][0], c + STEP[d][1]
                if not inside(nr, nc):
                    WALLS[r, c, d] = True
                    continue
                try:
                    WALLS[r, c, d] = not maze.can_move((r, c), d)
                except Exception:
                    WALLS[r, c, d] = True
    for (r, c, d) in list(WALLS):            # a wall seen from either side is a wall from both
        nr, nc = r + STEP[d][0], c + STEP[d][1]
        if inside(nr, nc) and (WALLS[r, c, d] or WALLS[nr, nc, (d + 2) % 4]):
            WALLS[r, c, d] = True
            WALLS[nr, nc, (d + 2) % 4] = True
    START_CELL = (maze.spawn_row, 0)
    EXIT_CELL = (maze.exit_row, N - 1)
    WALLS[START_CELL[0], 0, 2] = False       # gap on the west border
    WALLS[EXIT_CELL[0], N - 1, 0] = False    # gap on the east border
    _wall_boxes.cache_clear()
    _turn_sweep_clear.cache_clear()
    _arc_clear.cache_clear()


def ascii_maze():
    lines = []
    for r in range(N):
        top = "+"
        mid = ""
        for c in range(N):
            top += ("---" if WALLS[r, c, 1] else "   ") + "+"
            mid += ("|" if WALLS[r, c, 2] else " ") + "   "
        mid += "|" if WALLS[r, N - 1, 0] else " "
        lines += [top, mid]
    lines.append("+" + "".join(("---" if WALLS[N - 1, c, 3] else "   ") + "+" for c in range(N)))
    return "\n".join(lines)


# ----------------------------------------------------------------------------
# Pivot / arc safety (chassis swept against the walls)
# ----------------------------------------------------------------------------
@lru_cache(maxsize=None)
def _wall_boxes(cell):
    row, col = cell
    x, y = cell_centre(cell)
    half = PITCH / 2.0
    boxes = []
    if WALLS[row, col, 0]:
        boxes.append((x + half - WALL_HALF, y - half, x + half + WALL_HALF, y + half))
    if WALLS[row, col, 2]:
        boxes.append((x - half - WALL_HALF, y - half, x - half + WALL_HALF, y + half))
    if WALLS[row, col, 1]:
        boxes.append((x - half, y + half - WALL_HALF, x + half, y + half + WALL_HALF))
    if WALLS[row, col, 3]:
        boxes.append((x - half, y - half - WALL_HALF, x + half, y - half + WALL_HALF))
    return tuple(boxes)


def _box_overlap_with_chassis(cx, cy, angle, wall, margin):
    """Separating-axis test between the rotated chassis rectangle and a wall box."""
    x0, y0, x1, y1 = wall
    hx, hy = 0.046 + margin, 0.043 + margin
    ca, sa = math.cos(angle), math.sin(angle)
    robot = [(cx + sx * hx * ca - sy * hy * sa, cy + sx * hx * sa + sy * hy * ca)
             for sx, sy in ((-1, -1), (-1, 1), (1, -1), (1, 1))]
    obstacle = ((x0, y0), (x0, y1), (x1, y0), (x1, y1))
    for ux, uy in ((1.0, 0.0), (0.0, 1.0), (ca, sa), (-sa, ca)):
        rproj = [x * ux + y * uy for x, y in robot]
        wproj = [x * ux + y * uy for x, y in obstacle]
        if max(rproj) <= min(wproj) or max(wproj) <= min(rproj):
            return False
    return True


def _nearby_walls(cell):
    row, col = cell
    walls = set()
    for r in range(max(0, row - 1), min(N, row + 2)):
        for c in range(max(0, col - 1), min(N, col + 2)):
            walls.update(_wall_boxes((r, c)))
    return walls


@lru_cache(maxsize=None)
def _turn_sweep_clear(cell, heading, target_heading):
    cell = tuple(cell)
    start_angle = heading * math.pi / 2.0
    delta = wrap((target_heading - heading) * math.pi / 2.0)
    axle_x, axle_y = cell_centre(cell)
    walls = _nearby_walls(cell)
    samples = max(1, math.ceil(abs(delta) / (math.pi / 36.0)))
    for i in range(samples + 1):
        angle = start_angle + delta * i / samples
        chassis_x = axle_x + AXLE_BACK * math.cos(angle)
        chassis_y = axle_y + AXLE_BACK * math.sin(angle)
        if any(_box_overlap_with_chassis(chassis_x, chassis_y, angle, w, 0.008) for w in walls):
            return False
    return True


@lru_cache(maxsize=None)
def _arc_clear(cell, h, h2):
    """Does a quarter-circle arc (entry edge -> exit edge of `cell`) clear every wall?"""
    cell = tuple(cell)
    sign = 1 if (h2 - h) % 4 == 1 else -1
    cx, cy = cell_centre(cell)
    din = (math.cos(h * math.pi / 2), math.sin(h * math.pi / 2))
    left = (-din[1], din[0])
    entry = (cx - ARC_R * din[0], cy - ARC_R * din[1])
    rs = ARC_R * ARC_STRETCH                       # the real path is wider than the nominal quarter circle
    ctr = (entry[0] + sign * rs * left[0], entry[1] + sign * rs * left[1])
    v0 = (entry[0] - ctr[0], entry[1] - ctr[1])
    walls = _nearby_walls(cell)
    for i in range(25):                            # dense sampling: no clipping between samples
        phi = sign * (math.pi / 2) * i / 24.0
        ca, sa = math.cos(phi), math.sin(phi)
        px = ctr[0] + v0[0] * ca - v0[1] * sa
        py = ctr[1] + v0[0] * sa + v0[1] * ca
        ang = h * math.pi / 2 + phi
        chx, chy = px + AXLE_BACK * math.cos(ang), py + AXLE_BACK * math.sin(ang)
        if any(_box_overlap_with_chassis(chx, chy, ang, w, ARC_MARGIN) for w in walls):
            return False
    return True


def warm_turn_cache():
    for r in range(N):
        for c in range(N):
            for h in range(4):
                for h2 in range(4):
                    if h != h2:
                        _turn_sweep_clear((r, c), h, h2)
                        if (h2 - h) % 4 in (1, 3):
                            _arc_clear((r, c), h, h2)


# ----------------------------------------------------------------------------
# Planner: start -> all pellets (best order) -> exit, minimising estimated time
# ----------------------------------------------------------------------------
def _turn_cost(h0, h1):
    k = (h1 - h0) % 4
    if k == 0:
        return 0.0
    return TURN90_T if k in (1, 3) else TURN180_T


def plan_route(start, heading, pellets, zone=None, order=None, arcs=None, chain=None, bad_arcs=None):
    """Returns (actions, path_cells, est_seconds, pellet_order).  zone: {cell: extra seconds}.

    Search state = (row, col, heading, pellet mask, a) with a:
      0 at rest, 1 arrived rolling (pivot or arc allowed), 2 just left an arc (must go straight),
      3 one cell after an arc, chaining off (pivot allowed, arc not),
      4 one cell after an arc, chaining on (pivot or arc allowed).  3 and 4 charge extra for a 180 deg pivot.
    chain: allow an arc right after the previous arc's cell (None -> ARC_BACK_TO_BACK).
    """
    arcs = USE_ARCS if arcs is None else arcs
    chain = ARC_BACK_TO_BACK if chain is None else chain
    banned = bad_arcs or ()
    zone = zone or {}
    start = tuple(start)
    pellets = [tuple(p) for p in (order if order is not None else pellets) if tuple(p) != start]
    bit = {p: 1 << i for i, p in enumerate(pellets)}
    full = (1 << len(pellets)) - 1
    no_arc = set(pellets) | {EXIT_CELL, start}      # pivot on these cells: collect pellets / aim at the exit

    s0 = (start[0], start[1], heading, 0, 0)
    best = {s0: 0.0}
    parent = {}
    pq = [(0.0, s0)]
    goal, goal_cost = None, float("inf")

    def relax(state, cost, prev):
        if cost < best.get(state, float("inf")):
            best[state] = cost
            parent[state] = prev
            heapq.heappush(pq, (cost, state))

    while pq:
        cost, st = heapq.heappop(pq)
        if cost > best.get(st, float("inf")):
            continue
        if cost >= goal_cost:
            break
        r, c, h, m, a = st
        if (r, c) == EXIT_CELL and m == full and a != 2:
            total = cost + _turn_cost(h, EXIT_HEADING)
            if h != EXIT_HEADING and not _turn_sweep_clear((r, c), h, EXIT_HEADING):
                total += UNSAFE_T
            if total < goal_cost:
                goal, goal_cost = st, total
        if a != 2:
            for h2 in range(4):
                if h2 == h:
                    continue
                k = _turn_cost(h, h2)
                if not _turn_sweep_clear((r, c), h, h2):
                    k += UNSAFE_T
                if a in (3, 4) and (h2 - h) % 4 == 2:
                    k += PIVOT180_AFTER_ARC_T
                relax((r, c, h2, m, 0), cost + k, st)
                if (arcs and a in (1, 4) and (h2 - h) % 4 in (1, 3) and (r, c) not in no_arc
                        and ((r, c), h, h2) not in banned and _arc_clear((r, c), h, h2)):
                    relax((r, c, h2, m, 2), cost + ARC_T, st)
        if not WALLS[r, c, h]:
            nr, nc = r + STEP[h][0], c + STEP[h][1]
            if inside(nr, nc):
                k = CELL_T + zone.get((nr, nc), 0.0)
                b = bit.get((nr, nc), 0)
                if order is not None and b and m != b - 1:
                    b = 0                          # forced order: only count the next pellet in line
                na = (4 if chain else 3) if a == 2 else 1
                relax((nr, nc, h, m | b, na), cost + k, st)

    if goal is None:
        raise ValueError(f"no route from {start} via {pellets}")

    states = [goal]
    while states[-1] != s0:
        states.append(parent[states[-1]])
    states.reverse()

    actions = []
    path_cells = [start]
    order_out = []
    for prev, st in zip(states, states[1:]):
        if st[2] != prev[2]:
            if st[4] == 2:
                actions.append(("arc", st[2]))
            elif actions and actions[-1][0] == "turn":
                actions[-1] = ("turn", st[2])
            else:
                actions.append(("turn", st[2]))
        else:
            d, nxt = st[2], (st[0], st[1])
            path_cells.append(nxt)
            if nxt in bit and nxt not in order_out:
                order_out.append(nxt)
            if actions and actions[-1][0] == "run" and actions[-1][2] == d:
                actions[-1] = ("run", actions[-1][1] + 1, d, nxt)
            else:
                actions.append(("run", 1, d, nxt))
    if goal[2] != EXIT_HEADING:
        actions.append(("turn", EXIT_HEADING))
    actions.append(("exit", EXIT_DISTANCE))
    return actions, path_cells, goal_cost, order_out


# ----------------------------------------------------------------------------
# Controller
# ----------------------------------------------------------------------------
ALIGN = "ALIGN"
RUN = "RUN"
TURN = "TURN"
SETTLE = "SETTLE"
EXITING = "EXIT"
DONE = "DONE"
ARC = "ARC"
RECOVER = "RECOVER"


class Controller:
    def __init__(self, maze):
        self.maze = maze
        load_walls(maze)
        print("maze as the planner sees it (compare with the drawing):")
        print(ascii_maze(), flush=True)
        print("precomputing pivot clearances ...", flush=True)
        warm_turn_cache()

        self.pellets = []
        self.have_pellets = False
        self.elapsed = 0.0
        self.ghost = None
        self.prev_ticks = None

        # plan / execution state
        self.actions = None
        self.action_index = 0
        self.path_cells = [START_CELL]
        self.moves_done = 0
        self.replans = 0
        self.done = False
        self.failed = False
        self.theta = 0.0                 # compass heading from the gyro, 0 = east
        self.command_heading = 0
        self.mode = None
        self.cell = START_CELL
        self.rem = 0.0
        self.s = 0.0                     # odometry progress along the current run
        self.s_end = 0.0
        self.pending_s = 0.0
        self.run_dir = 0
        self.landmarks = []
        self.front_wall_s = None
        self.cmd_speed = 0.0
        self.prev_left_wall = None
        self.prev_right_wall = None
        self.turn_planned = 0.0
        self.turn_settle = 0.0
        self.action_time = 0.0
        self.hold_time = 0.0
        self.next_progress_log = 2.0
        self.prog = 0.0                  # progress along the whole route, metres
        self.frame = 0
        self.was_holding = False

        # arcs
        self.arc_s = 0.0
        self.arc_sign = 1                # +1 left turn, -1 right turn
        self.arc_theta0 = 0.0
        self.arc_th_start = 0.0
        self.arc_delta = 0.0
        self.arc_t = 0.0
        self.arc_n = 0
        self.arc_info = {}
        self.arc_inner_min = 9.0
        self.v_meas = 0.0
        self.arc_checked = False         # entry gate already evaluated for the arc after this run
        self.after_arc = False           # the current run started right after an arc

        # recovery
        self.arcs_ok = True              # False from the 4th recovery on: pivots only
        self.chain_ok = True             # False from the 2nd recovery on: no back-to-back arcs
        self.bad_arcs = set(BAD_ARCS_PRESET)
        self.rec_n = 0
        self.rec_k = 0
        self.rec_back = 0.0
        self.rec_t = 0.0
        self.stuck_t = 0.0
        self.rec_grace = 0.0

        # ghost
        self.obs_start = None
        self.gseq = []
        self.gtraj_t = []
        self.gtraj_xy = []
        self.g_live = None
        self.g_hist = []
        self.g_T = None
        self.g_tref = 0.0
        self.g_off = 0.0
        self.gm_t = []
        self.gm_xy = []
        self.zone = {}
        self.corridor = frozenset()
        self.cp = []
        self.danger = []
        self.gate_ok = True
        self.gate_iv = None
        self.gate_frame = -100
        self.gate_wait_t = 0.0
        self.gate_latched = False
        self.wait_dist = None

    # ------------------------------------------------------------------ io
    def log(self, text):
        print(f"[{self.elapsed:7.2f}s] {text}", flush=True)

    def on_pellets(self, msg):
        self.pellets = [tuple(c) for c in msg.get("pellets", [])]
        self.have_pellets = True
        print(f"pellets remaining: {self.pellets}", flush=True)

    # --------------------------------------------------------------- ghost
    def _observe_ghost(self):
        g = self.ghost
        if not g:
            return
        try:
            x, y = float(g["x"]), float(g["y"])
            cell = (int(g["row"]), int(g["col"]))
        except (KeyError, TypeError, ValueError):
            return
        self.g_live = (x, y)
        self.g_hist.append((x, y))
        if len(self.g_hist) > 10:
            self.g_hist.pop(0)
        if self.obs_start is None:
            self.obs_start = self.elapsed
        if self.g_T is None and self.actions is None:
            self.gtraj_t.append(self.elapsed)
            self.gtraj_xy.append((x, y))
            if not self.gseq or self.gseq[-1][1] != cell:
                self.gseq.append((self.elapsed, cell))

    def _ghost_dist(self):
        """Distance (m) from the robot's believed position to the live ghost; -1 if unknown.
        Logged on arc / run / recovery lines to tell wall hits from ghost hits."""
        if self.g_live is None or self.actions is None:
            return -1.0
        px, py = self._pos_at(self.prog)
        return math.hypot(self.g_live[0] - px, self.g_live[1] - py)

    def _try_build_model(self, obs):
        """Find the shortest repeating cell sequence of the ghost; period and trajectory come from it."""
        seq = [c for _, c in self.gseq[1:]]       # [0] is wherever it was when observation began
        tms = [t for t, _ in self.gseq[1:]]
        n = len(seq)
        k_found = None
        for k in range(2, n - 5):                 # whole loop seen + 6 repeats confirmed
            if all(seq[i] == seq[i + k] for i in range(n - k)):
                k_found = k
                break
        if k_found is None and obs > OBS_SOFT:
            for k in range(2, n - 3):
                if all(seq[i] == seq[i + k] for i in range(n - k)):
                    k_found = k
                    break
        if k_found is None:
            return False
        period = tms[k_found] - tms[0]
        if period < 1.0:
            return False
        self.g_T, self.g_tref = period, tms[0]
        for t, xy in zip(self.gtraj_t, self.gtraj_xy):
            if self.g_tref <= t < self.g_tref + period:
                self.gm_t.append(t - self.g_tref)
                self.gm_xy.append(xy)
        self.corridor = frozenset(seq[:k_found])
        self.cp = self.gm_xy[::20] or list(self.gm_xy)
        self.zone = {}
        for r in range(N):
            for c in range(N):
                cx, cy = cell_centre((r, c))
                if min(math.hypot(cx - x, cy - y) for x, y in self.cp) < DANGER_R:
                    self.zone[r, c] = ZONE_T + (ZONE_PATH_T if (r, c) in self.corridor else 0.0)
        self.log(f"ghost patrol learned: period {period:.1f}s, path cells {sorted(self.corridor)}, "
                 f"{len(self.zone)} cells count as its territory")
        self.gtraj_t, self.gtraj_xy = [], []
        return True

    def _model_ready(self):
        if self.g_T is not None:
            return True
        if self.ghost is None or self.obs_start is None:
            return self.elapsed > NO_GHOST_WAIT
        obs = self.elapsed - self.obs_start
        if self._try_build_model(obs):
            return True
        if obs > OBS_HARD:
            self.log("could not learn the ghost loop; using reactive fallback only")
            return True
        return False

    def _model_xy(self, t):
        phase = (t - self.g_tref) % self.g_T
        i = bisect.bisect_left(self.gm_t, phase)
        if i >= len(self.gm_t):
            i = len(self.gm_t) - 1
        elif i > 0 and abs(self.gm_t[i - 1] - phase) < abs(self.gm_t[i] - phase):
            i -= 1
        return self.gm_xy[i]

    def _resync(self):
        """Shift the model clock so the predicted ghost position matches the live one."""
        if self.g_T is None or self.g_live is None:
            return
        px, py = self._model_xy(self.elapsed + self.g_off)
        lx, ly = self.g_live
        if math.hypot(px - lx, py - ly) < 0.04:
            return
        best = None
        for k in range(-40, 41):
            dlt = k * 0.02
            x, y = self._model_xy(self.elapsed + self.g_off + dlt)
            sc = math.hypot(x - lx, y - ly) + 0.01 * abs(dlt)
            if best is None or sc < best[0]:
                best = (sc, dlt)
        self.g_off += best[1]

    def _ghost_at(self, tau):
        """Predicted ghost position tau seconds from now."""
        if self.g_live is None:
            return None
        if self.g_T is not None:
            return self._model_xy(self.elapsed + self.g_off + tau)
        gx, gy = self.g_live
        if len(self.g_hist) >= 2:
            span = (len(self.g_hist) - 1) * 0.01
            vx = (self.g_hist[-1][0] - self.g_hist[0][0]) / span
            vy = (self.g_hist[-1][1] - self.g_hist[0][1]) / span
        else:
            vx = vy = 0.0
        tc = min(tau, 0.8)
        return gx + vx * tc, gy + vy * tc

    def _pos_at(self, p):
        """World (x, y) of route progress p (metres along the cell centres of the plan)."""
        cells = self.path_cells
        n = len(cells) - 1
        c0 = cell_centre(cells[0])
        if p <= 0.0 or n == 0:
            return c0[0] + p, c0[1]
        if p >= n * CELL:
            cx, cy = cell_centre(cells[-1])
            return cx + (p - n * CELL), cy
        i = int(p // CELL)
        f = (p - i * CELL) / CELL
        a, b = cell_centre(cells[i]), cell_centre(cells[i + 1])
        return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f

    def _update_prog(self):
        if self.mode in (ALIGN, RUN):
            self.prog = self.moves_done * CELL + self.s
        elif self.mode == EXITING:
            self.prog = (len(self.path_cells) - 1) * CELL + (EXIT_DISTANCE - self.rem)
        elif self.mode == ARC:
            self.prog = (self.moves_done * CELL - ARC_HALF
                         + 2 * ARC_HALF * min(1.0, self.arc_s / (ARC_R * math.pi / 2)))
        else:
            self.prog = self.moves_done * CELL

    def _reactive_hold(self):
        """Fallback when the patrol could not be learned: look GHOST_HORIZON ahead."""
        if self.g_live is None or self.actions is None or self.mode == DONE:
            return False
        px, py = self._pos_at(self.prog)
        if math.hypot(self.g_live[0] - px, self.g_live[1] - py) > 1.3:
            return False
        dmin_go = dmin_stay = 9.0
        for k in range(int(GHOST_HORIZON / 0.1) + 1):
            tau = k * 0.1
            g = self._ghost_at(tau)
            ax, ay = self._pos_at(self.prog + GHOST_GO_SPEED * tau)
            dmin_go = min(dmin_go, math.hypot(g[0] - ax, g[1] - ay))
            dmin_stay = min(dmin_stay, math.hypot(g[0] - px, g[1] - py))
        if dmin_go >= GHOST_SAFE:
            return False
        return dmin_stay >= GHOST_SAFE or dmin_stay > dmin_go

    def _compute_danger(self, quiet=False):
        """Stretches of the route (progress in m) that run through the ghost's territory."""
        self.danger = []
        self.gate_ok = True
        self.gate_iv = None
        self.gate_frame = -100
        if not self.cp:
            return
        p_max = (len(self.path_cells) - 1) * CELL + EXIT_DISTANCE
        ivs, start, p = [], None, -0.25
        while p <= p_max:
            x, y = self._pos_at(p)
            near = min(math.hypot(x - cx, y - cy) for cx, cy in self.cp) < DANGER_R
            if near and start is None:
                start = p
            elif not near and start is not None:
                ivs.append([start, p])
                start = None
            p += 0.02
        if start is not None:
            ivs.append([start, p_max])
        merged = []
        for a, b in ivs:
            if merged and a - merged[-1][1] < MERGE_GAP:
                merged[-1][1] = b
            else:
                merged.append([a, b])
        self.danger = [(a, b) for a, b in merged]
        if not quiet:
            self.log("ghost territory on the route (progress m): "
                     + (", ".join(f"{a:.2f}-{b:.2f}" for a, b in self.danger) or "none"))

    def _segments(self, tscale, vscale):
        """The whole plan as (p0, p1, seconds, kind) pieces; progress is measured from the start cell."""
        segs, cur, h = [], 0.0, 0
        A = self.actions
        for i, a in enumerate(A):
            prev = A[i - 1][0] if i > 0 else None
            nxt = A[i + 1][0] if i + 1 < len(A) else None
            if a[0] == "turn":
                segs.append((cur, cur, (TURN_T if (a[1] - h) % 4 in (1, 3) else TURN180_TIME) * tscale, "pivot"))
                h = a[1]
            elif a[0] == "arc":
                segs.append((cur - ARC_HALF, cur + ARC_HALF, ARC_TIME * tscale / vscale, "arc"))
                h = a[1]
            elif a[0] == "run":
                d = a[1] * CELL
                p0 = cur + (ARC_HALF if prev == "arc" else 0.0)
                p1 = cur + d - (ARC_HALF if nxt == "arc" else 0.0)
                if prev in (None, "turn"):
                    segs.append((p0, p0, RUN_DWELL * tscale, "dwell"))
                segs.append((p0, p1, (p1 - p0) / (V_RUN * vscale) + (RUN_TAIL * tscale if nxt == "turn" else 0.0), "move"))
                cur += d
                h = a[2]
            else:
                segs.append((cur, cur + a[1], a[1] / (V_RUN * vscale), "move"))
        return segs

    def _timeline_from(self, start_prog, p_stop, tscale, vscale, moving=False,
                       skip_pivot_at_start=False, lead=0.0):
        """(time, progress) keyframes of the plan for a robot at start_prog."""
        ts, ps = [0.0], [start_prog]
        t = lead
        if lead:
            ts.append(t)
            ps.append(start_prog)
        p = start_prog
        fresh = True
        for p0, p1, dt, kind in self._segments(tscale, vscale):
            if p > p_stop:
                break
            if kind in ("pivot", "dwell"):
                if p0 < start_prog - 1e-6:
                    continue
                if kind == "pivot" and skip_pivot_at_start and abs(p0 - start_prog) < 1e-6:
                    continue
                if p < p0 - 1e-9:
                    t += (p0 - p) / (V_RUN * vscale)
                    p = p0
                    ts.append(t)
                    ps.append(p)
                t += dt
                ts.append(t)
                ps.append(p)
                fresh = False
                continue
            if p1 < start_prog - 1e-9:
                continue
            if p < p0 - 1e-9:                      # a gap before this piece: drive it
                t += (p0 - p) / (V_RUN * vscale)
                p = p0
                ts.append(t)
                ps.append(p)
            a0 = max(p0, p)
            if a0 >= p1 - 1e-9:
                continue
            frac = (p1 - a0) / (p1 - p0) if p1 > p0 else 1.0
            if fresh and not moving and a0 > p0 + 1e-9 and kind == "move":
                t += RUN_DWELL * tscale            # standing start in the middle of a run
                ts.append(t)
                ps.append(p)
            t += dt * frac
            p = p1
            ts.append(t)
            ps.append(p)
            fresh = False
        return ts, ps

    def _timeline(self, p_stop, tscale=1.0, vscale=1.0):
        """Predicted (time, progress) keyframes for the rest of the plan from the robot's real state."""
        if self.mode in (TURN, SETTLE):
            lead = max(0.3, TURN_T * tscale - self.action_time)
            return self._timeline_from(self.prog, p_stop, tscale, vscale,
                                       skip_pivot_at_start=True, lead=lead)
        if self.mode in (ALIGN, RUN, ARC, EXITING):
            return self._timeline_from(self.prog, p_stop, tscale, vscale, moving=self.cmd_speed > 1.0)
        return [0.0], [self.prog]

    @staticmethod
    def _interp(ts, ps, x):
        if x <= ts[0]:
            return ps[0]
        if x >= ts[-1]:
            return ps[-1]
        i = bisect.bisect_right(ts, x)
        f = (x - ts[i - 1]) / (ts[i] - ts[i - 1])
        return ps[i - 1] + (ps[i] - ps[i - 1]) * f

    def _env_safe(self, tl_slow, tl_fast, d_slow, d_fast, ghost_fn, safe):
        """Every position the robot could be at, at every moment, stays `safe` away from the ghost."""
        ts_s, ps_s = tl_slow
        ts_f, ps_f = tl_fast
        total = min(d_slow + ts_s[-1], 45.0)
        tau = 0.0
        while tau <= total:
            a = self._interp(ts_s, ps_s, max(0.0, tau - d_slow))
            b = self._interp(ts_f, ps_f, max(0.0, tau - d_fast))
            lo, hi = min(a, b) - 0.05, max(a, b) + 0.05
            g = ghost_fn(tau)
            if g is None:
                return False
            p = lo
            while True:
                x, y = self._pos_at(p)
                if math.hypot(g[0] - x, g[1] - y) < safe:
                    return False
                if p >= hi:
                    break
                p = min(hi, p + 0.05)
            tau += 0.1
        return True

    def _entry_safe(self, p_out):
        """If we carry on from now, do we stay clear of the ghost until we are out the far side?"""
        level = sum(self.gate_wait_t >= t for t in GATE_ESCALATE)
        slow, fast, safe = ENVELOPES[level]
        tl_s = self._timeline(p_out + 0.30, slow[0], slow[1])
        tl_f = self._timeline(p_out + 0.30, fast[0], fast[1])
        return self._env_safe(tl_s, tl_f, slow[2], fast[2], self._ghost_at, safe)

    def _plan_timeline(self, start_prog, p_stop, tscale, vscale):
        """Timeline of the plan for a robot standing still at start_prog."""
        return self._timeline_from(start_prog, p_stop, tscale, vscale, moving=False)

    @staticmethod
    def _time_at_prog(ts, ps, p):
        """First time at which the timeline has reached progress p."""
        if p <= ps[0]:
            return 0.0
        for i in range(1, len(ps)):
            if ps[i] >= p - 1e-9:
                if ps[i] - ps[i - 1] < 1e-9:
                    return ts[i - 1]
                return ts[i - 1] + (ts[i] - ts[i - 1]) * (p - ps[i - 1]) / (ps[i] - ps[i - 1])
        return ts[-1]

    def _predict_finish(self):
        """Seconds from now until the robot leaves the maze with the current plan, including ghost
        waits; None if some ghost stretch never has a safe window."""
        end_prog = (len(self.path_cells) - 1) * CELL + EXIT_DISTANCE
        slow, fast, safe = ENVELOPES[0]
        T, p0 = self.elapsed, 0.0
        for p_in, p_out in self.danger:
            if p_out <= p0:
                continue
            pw = max(p_in - WAIT_BACK, p0)
            ts, ps = self._plan_timeline(p0, end_prog + 1.0, 1.0, 1.0)
            T_arrive = T + self._time_at_prog(ts, ps, pw)
            tl_s = self._plan_timeline(pw, p_out + 0.30, slow[0], slow[1])
            tl_f = self._plan_timeline(pw, p_out + 0.30, fast[0], fast[1])
            T_enter, found = T_arrive, False
            while T_enter < T_arrive + self.g_T + 0.5:
                if self._env_safe(tl_s, tl_f, slow[2], fast[2],
                                  lambda tau, te=T_enter: self._model_xy(self.g_off + te + tau), safe):
                    found = True
                    break
                T_enter += 0.2
            if not found:
                return None
            tn, pn = self._plan_timeline(pw, p_out + 0.30, 1.0, 1.0)
            T = T_enter + self._time_at_prog(tn, pn, p_out + 0.05)
            p0 = p_out + 0.05
        ts, ps = self._plan_timeline(p0, end_prog + 1.0, 1.0, 1.0)
        return T + ts[-1] - self.elapsed

    def _infeasible_intervals(self):
        """Territory stretches of the current plan for which NO entry window exists at all."""
        bad = []
        if not self.danger or self.g_T is None:
            return bad
        slow, fast, safe = ENVELOPES[0]
        for p_in, p_out in self.danger:
            pw = p_in - WAIT_BACK
            tl_s = self._plan_timeline(pw, p_out + 0.30, slow[0], slow[1])
            tl_f = self._plan_timeline(pw, p_out + 0.30, fast[0], fast[1])
            ok, t0 = False, 0.0
            while t0 < self.g_T + 0.5 and not ok:
                ok = self._env_safe(tl_s, tl_f, slow[2], fast[2],
                                    lambda tau, t0=t0: self._model_xy(self.g_tref + t0 + tau), safe)
                t0 += 0.25
            if not ok:
                bad.append((p_in, p_out))
        return bad

    def _ghost_hold(self):
        """True -> stand still this frame (also sets self.wait_dist while approaching a wait point)."""
        self.wait_dist = None
        if self.actions is None or self.mode in (DONE, RECOVER):
            return False
        if self.g_T is None:
            return self._reactive_hold()
        iv = None
        for a, b in self.danger:
            if b > self.prog:
                iv = (a, b)
                break
        if iv is None:
            return False
        p_in, p_out = iv
        if self.prog > p_in + 0.05:
            return False                       # already inside: committed, keep going
        if p_in - self.prog > GATE_LOOKAHEAD:
            return False
        if iv != self.gate_iv:
            self.gate_iv = iv
            self.gate_frame = -100
            self.gate_wait_t = 0.0
            self.gate_latched = False
        if self.gate_latched:
            return False                       # window was open close to the territory: do not stop again
        if self.frame - self.gate_frame >= GATE_EVERY:
            self.gate_ok = self._entry_safe(p_out)
            self.gate_frame = self.frame
            if self.gate_ok and p_in - self.prog < GATE_COMMIT_DIST:
                self.gate_latched = True
        if self.gate_ok:
            return False
        self.gate_wait_t += 0.01
        self.wait_dist = (p_in - WAIT_BACK) - self.prog
        return self.wait_dist <= 0.02

    # ------------------------------------------------------------ planning
    def _start_plan(self, front):
        marks, wall_s = self._run_geometry(START_CELL, 0, 1)
        s0 = -(0.15 + AXLE_BACK)                       # nominal spawn: x = -0.15
        if wall_s is not None and front < 0.99:
            est = wall_s + FRONT_AT_CENTER - front      # axle position from the front ToF
            if -0.45 < est < 0.02:
                s0 = est
        self.log(f"start calibration: axle is {-s0:.3f} m before cell {START_CELL} centre")
        try:
            acts, cells, cost, order = self._choose_plan()
        except (ValueError, KeyError) as exc:
            return self._stop_with_error(f"route planning failed: {exc}") is None
        self.actions, self.path_cells = acts, cells
        self.action_index = 0
        self.moves_done = 0
        self.cell = START_CELL
        self.command_heading = 0
        self.theta = 0.0
        self.s = s0
        self.pending_s = s0
        self._compute_danger()
        self.log(f"pellet order {order}, est. {cost:.1f}s")
        self.log(f"plan ({len(acts)} actions): {acts}")
        if acts[0][0] == "run":
            self._begin_action()
        else:
            self.mode = ALIGN
            self.action_time = 0.0
        return True

    def _choose_plan(self):
        """Try every pellet order (with a few ghost-avoidance strengths), predict the real finish
        time of each including ghost waits, and keep the fastest."""
        import itertools
        cands, seen = [], set()
        orders = list(itertools.permutations(self.pellets))
        for zm in (0.0, 1.0, 3.0):
            zone = {c: v * zm for c, v in self.zone.items()}
            for order in orders:
                try:
                    acts, cells, cost, ordr = plan_route(START_CELL, 0, self.pellets, zone,
                                                         order=order, bad_arcs=self.bad_arcs)
                except ValueError:
                    continue
                key = tuple(acts)
                if key in seen:
                    continue
                seen.add(key)
                self.actions, self.path_cells = acts, cells
                self._compute_danger(quiet=True)
                fin = self._predict_finish()
                if fin is not None:
                    cands.append((fin, cost, acts, cells, ordr))
        if cands:
            cands.sort(key=lambda c: c[0])
            self.log("route candidates (predicted finish s, order): "
                     + "; ".join(f"{c[0]:.1f} {c[4]}" for c in cands[:4]))
            fin, cost, acts, cells, ordr = cands[0]
            return acts, cells, fin, ordr
        # No candidate has a safe window: penalise the bad stretches and re-plan.
        penal = {}
        for attempt in range(6):
            zone = dict(self.zone)
            for c, v in penal.items():
                zone[c] = zone.get(c, 0.0) + v
            acts, cells, cost, ordr = plan_route(START_CELL, 0, self.pellets, zone,
                                                 bad_arcs=self.bad_arcs)
            self.actions, self.path_cells = acts, cells
            self._compute_danger()
            bad = self._infeasible_intervals()
            if not bad:
                break
            self.log(f"route has a ghost stretch with no safe window {bad}; re-planning")
            for a, b in bad:
                for i, c in enumerate(cells):
                    if a - 0.15 <= i * CELL <= b + 0.15:
                        penal[c] = penal.get(c, 0.0) + 40.0
        return acts, cells, cost, ordr

    def _replan(self, cell=None, heading=None, count=True):
        """Re-plan from `cell`/`heading` (default: where the controller believes it is)."""
        if count:
            self.replans += 1
        cell = self.cell if cell is None else cell
        heading = self.command_heading if heading is None else heading
        try:
            acts, cells, cost, order = plan_route(cell, heading, self.pellets, self.zone,
                                                  arcs=None if self.arcs_ok else False,
                                                  chain=None if self.chain_ok else False,
                                                  bad_arcs=self.bad_arcs)
        except (ValueError, KeyError) as exc:
            self._stop_with_error(f"replan failed: {exc}")
            return False
        self.log(f"pellets still on board {self.pellets}: replanning from {cell} heading "
                 f"{heading} -> {order}, est. {cost:.1f}s")
        self.actions, self.path_cells = acts, cells
        self.action_index = 0
        self.moves_done = 0
        self.pending_s = 0.0
        self._compute_danger()
        return True

    def _stop_with_error(self, message):
        self.failed = True
        self.done = True
        self.mode = DONE
        self.log(f"ERROR: {message}; stopping")
        return 0.0, 0.0

    # ------------------------------------------------------------ recovery
    def _estimate_cell(self):
        """Best guess of the cell the robot is in right now."""
        if self.mode == RUN:
            n = self.actions[self.action_index][1]
            j = int(clamp((self.s + CELL / 2) // CELL, 0, n))
            dr, dc = STEP[self.run_dir]
            cell = (self.cell[0] + j * dr, self.cell[1] + j * dc)
            if inside(*cell):
                return cell
        return self.cell          # in an arc, self.cell is the junction cell

    def _recovery_heading(self):
        """Nearest compass heading, preferring the intended one when we are not too far off it."""
        nearest = int(round(self.theta / (math.pi / 2))) % 4
        intended = None
        if self.mode == RUN:
            intended = self.run_dir
        elif self.mode == ARC:
            intended = self.actions[self.action_index][1]
        if intended is not None and abs(wrap(intended * math.pi / 2 - self.theta)) < RECOVER_TRUST_INTENDED:
            return intended
        return nearest

    def _watchdog(self, dt, front):
        """Return a reason string when the run/arc has clearly gone wrong, else None."""
        if self.mode == RUN:
            err = abs(wrap(self.run_dir * math.pi / 2 - self.theta))
            if err > RECOVER_YAW:
                return f"heading error {math.degrees(err):.0f} deg in run"
        elif self.mode == ARC and self.arc_t > ARC_MAX_T:
            return f"arc took {self.arc_t:.2f}s"
        # Odometry keeps moving while the wheels spin against a wall, so speed is not part of the test.
        pushing = self.cmd_speed > 3.0 and front < RECOVER_FRONT
        self.stuck_t = self.stuck_t + dt if pushing else 0.0
        if self.stuck_t > RECOVER_STUCK_T:
            return "stuck against a wall"
        return None

    def _start_recovery(self, why):
        self.rec_n += 1
        if self.rec_n > RECOVER_MAX:
            return self._stop_with_error("too many recoveries")
        if self.mode == ARC:                       # never plan that particular arc again
            self.bad_arcs.add((tuple(self.cell), self.command_heading, self.actions[self.action_index][1]))
        self.cell = self._estimate_cell()          # must happen before the mode changes
        self.rec_k = self._recovery_heading()
        if self.rec_n >= 2:
            self.chain_ok = False
        if self.rec_n >= 4:
            self.arcs_ok = False
        self.rec_back = 0.0
        self.rec_t = 0.0
        self.stuck_t = 0.0
        self.cmd_speed = 0.0
        gd = self._ghost_dist()
        self.mode = RECOVER
        self.log(f"RECOVERY #{self.rec_n}: {why}; assuming cell {self.cell}, "
                 f"yaw {math.degrees(self.theta):.1f} deg -> heading {self.rec_k} "
                 f"(arcs {'on' if self.arcs_ok else 'off'}, chaining {'on' if self.chain_ok else 'off'}, "
                 f"{len(self.bad_arcs)} arc(s) banned) g={gd:.2f}m")
        return 0.0, 0.0

    def _relocalize(self, k, front):
        """Robot is squared up on compass heading k.  A front reading under 0.20 m means it is in the
        last cell before the wall ahead.  Test the estimated cell and its two neighbours along the
        heading (the estimate was observed to lag by exactly one cell) and keep the one whose
        predicted front distance matches best; reject the match if even the best is off."""
        if front >= 0.20:
            return self.cell
        best = None
        for shift in (0, 1, -1):
            if shift == 1 and WALLS[self.cell[0], self.cell[1], k]:
                continue                           # wall between: that cell is unreachable
            if shift == -1 and WALLS[self.cell[0], self.cell[1], (k + 2) % 4]:
                continue
            cand = (self.cell[0] + shift * STEP[k][0], self.cell[1] + shift * STEP[k][1])
            if not inside(*cand):
                continue
            r, c = cand
            steps = 0
            while not WALLS[r, c, k] and steps < 4:
                nr, nc = r + STEP[k][0], c + STEP[k][1]
                if not inside(nr, nc):
                    break
                r, c, steps = nr, nc, steps + 1
            err = abs(front - (steps * CELL + FRONT_AT_CENTER))
            if best is None or err < best[0]:
                best = (err, cand)
        if best and best[0] < RELOC_TOL:
            return best[1]
        self.log(f"not re-localising: best match off by {best[0] if best else -1:.3f} m "
                 f"(unmapped obstacle or bad estimate)")
        return self.cell

    def _recover_step(self, ds, front):
        """Back off the wall, square up on the compass heading, re-localise on the wall ahead, re-plan."""
        self.rec_t += 0.01
        self.rec_back += -ds
        if front < 0.10 and self.rec_back < RECOVER_BACK_DIST and self.rec_t < 0.8:
            return drive_vw(-RECOVER_BACK_SPEED, 0.0)
        k = self.rec_k
        err = wrap(k * math.pi / 2 - self.theta)
        if abs(err) > 0.06 and self.rec_t < 2.5:   # pivot onto the exact compass heading first
            w = clamp(math.sqrt(2.0 * TURN_ALPHA * abs(err)) * TRACK / 2.0 / WHEEL_R, TURN_MIN, TURN_MAX)
            return (-w, w) if err > 0 else (w, -w)
        cell = self._relocalize(k, front)
        if cell != self.cell:
            self.log(f"re-localised on the wall ahead: {self.cell} -> {cell}")
            self.cell = cell
        self.command_heading = k
        if not self._replan(cell=self.cell, heading=k, count=False):
            return 0.0, 0.0
        if abs(err) > 0.06 and self.actions[0][0] != "turn":   # could not square up in time
            self.actions.insert(0, ("turn", k))
        self.rec_grace = RECOVER_GRACE
        self._begin_action()
        return 0.0, 0.0

    # ----------------------------------------------------- action sequencing
    def _begin_action(self):
        action = self.actions[self.action_index]
        kind = action[0]
        self.action_time = 0.0
        self.next_progress_log = 2.0
        self.prev_left_wall = None
        self.prev_right_wall = None
        self.stuck_t = 0.0
        if kind == "turn":
            self.mode = TURN
            self.turn_settle = 0.0
            self.turn_planned = wrap((action[1] - self.command_heading) * math.pi / 2)
        elif kind == "run":
            self.mode = RUN
            self.run_dir = action[2]
            self.s = self.pending_s
            self.pending_s = 0.0
            self.s_end = action[1] * CELL
            self.arc_checked = False
            self.landmarks, self.front_wall_s = self._run_geometry(self.cell, action[2], action[1])
            if self._next_kind() == "arc":
                self.s_end -= ARC_HALF             # the arc starts half a cell before the junction centre
        elif kind == "arc":
            self.mode = ARC
            self.arc_s = 0.0
            self.arc_sign = 1 if (action[1] - self.command_heading) % 4 == 1 else -1
            self.arc_theta0 = self.command_heading * math.pi / 2
            self.arc_t = 0.0
            self.arc_n += 1
            self.arc_info = {}
            self.arc_inner_min = 9.0
        elif kind == "exit":
            self.mode = EXITING
            self.rem = action[1]
        self.log(f"action {self.action_index + 1}/{len(self.actions)}: {action}")

    def _advance(self):
        completed = self.actions[self.action_index]
        if completed[0] == "turn":
            self.command_heading = completed[1]
            self.after_arc = False
        elif completed[0] == "arc":
            self.command_heading = completed[1]
            self.pending_s = ARC_HALF              # we leave the arc half a cell past the junction centre
            self.after_arc = True
        elif completed[0] == "run":
            self.command_heading = completed[2]
            self.moves_done += completed[1]
            self.after_arc = False
        self.action_index += 1
        if self.action_index >= len(self.actions):
            self.mode = DONE
            self.done = True
            self.log("route complete; awaiting simulator result")
            return
        nxt = self.actions[self.action_index]
        if nxt[0] == "exit" and self.pellets and self.replans < 2:
            if not self._replan():                 # a pellet was missed: go back for it before leaving
                return
        self._begin_action()

    def _next_kind(self):
        nxt = self.action_index + 1
        return self.actions[nxt][0] if nxt < len(self.actions) else None

    # --------------------------------------------------------------- driving
    def _lane_command(self, route_dir, side_l, side_r, speed, max_off=MAX_HEADING_OFFSET):
        """Hold the compass heading and stay centred between the side walls (one wall is enough)."""
        err_h = wrap(route_dir * math.pi / 2 - self.theta)
        left_wall = side_l < SIDE_WALL
        right_wall = side_r < SIDE_WALL
        if left_wall and right_wall:
            err_c = (side_l - side_r) / 2.0
        elif left_wall:
            err_c = side_l - SIDE_CENTERED
        elif right_wall:
            err_c = SIDE_CENTERED - side_r
        else:
            err_c = 0.0
        offset = clamp(K_CENTER * err_c, -max_off, max_off)
        omega = clamp(K_HEADING * wrap(err_h + offset), -MAX_OMEGA, MAX_OMEGA)
        return drive_vw(speed, omega)

    def _arc_gate(self, err_h, side_l, side_r):
        """Full check once, ARC_GATE_DIST before the arc (also right after a chained arc), then only an
        extreme-error check in the last 4 cm.  Turns the coming arc into a stop-and-pivot when heading /
        lateral offset are too poor to roll through the corner, or when a chained arc has no side wall
        to correct against."""
        full = not self.arc_checked
        self.arc_checked = True
        wl, wr = side_l < ARC_ONE_WALL, side_r < ARC_ONE_WALL
        if side_l < SIDE_WALL and side_r < SIDE_WALL:
            lat = (side_l - side_r) / 2.0          # > 0: closer to the right wall
        elif wl:
            lat = side_l - SIDE_CENTERED
        elif wr:
            lat = SIDE_CENTERED - side_r
        else:
            lat = 0.0
        yaw = abs(err_h)
        blind = side_l >= SIDE_WALL and side_r >= SIDE_WALL
        chained = self.after_arc and self.actions[self.action_index][1] == 1
        if full:
            bad = (abs(lat) > ARC_ENTRY_LAT or yaw > ARC_ENTRY_YAW
                   or abs(lat) / ARC_ENTRY_LAT + yaw / ARC_ENTRY_YAW > ARC_ENTRY_MIX
                   or (blind and chained and yaw > math.radians(4)))
        else:
            bad = self.s_end - self.s < 0.04 and (yaw > ARC_LATE_YAW or abs(lat) > ARC_LATE_LAT)
        if not bad:
            return False
        nxt = self.actions[self.action_index + 1]
        self.actions[self.action_index + 1] = ("turn", nxt[1])
        self.s_end += ARC_HALF                      # the run now ends on the junction centre
        self.log(f"arc entry unsafe (yaw {math.degrees(err_h):+.1f} deg, lat {lat:+.3f} m"
                 f"{', blind chained arc' if blind and chained else ''}) -> pivot")
        return True

    @staticmethod
    def _run_geometry(cell, d, n):
        """Wall landmarks (side-sensor edges) along a run, and the distance to the wall ahead."""
        dr, dc = STEP[d]
        marks = []
        for name, side_dir in (("L", (d + 1) % 4), ("R", (d + 3) % 4)):
            for j in range(1, n + 1):
                a = (cell[0] + (j - 1) * dr, cell[1] + (j - 1) * dc)
                b = (cell[0] + j * dr, cell[1] + j * dc)
                if not (inside(*a) and inside(*b)):
                    continue
                wa = WALLS[a[0], a[1], side_dir]
                wb = WALLS[b[0], b[1], side_dir]
                if wa != wb:
                    marks.append((name, wb, j * CELL - (WALL_TO_CENTER if wb else GAP_TO_CENTER)))
        steps, r, c = 0, cell[0], cell[1]
        wall_s = None
        while True:
            if WALLS[r, c, d]:
                wall_s = steps * CELL
                break
            nr, nc = r + dr, c + dc
            if not inside(nr, nc):
                break
            r, c, steps = nr, nc, steps + 1
        return marks, wall_s

    def _ramp(self, target, dt):
        if target > self.cmd_speed:
            self.cmd_speed = min(target, self.cmd_speed + SPEED_SLEW * dt)
        else:
            self.cmd_speed = target
        return self.cmd_speed

    @staticmethod
    def _gate_speed(distance):
        v = math.sqrt(2.0 * GATE_BRAKE_ACCEL * max(0.0, distance)) / WHEEL_R
        return clamp(v, GATE_CREEP, FWD_SPEED)

    @staticmethod
    def _braking_speed(distance):
        v = math.sqrt(2.0 * BRAKE_ACCEL * max(0.0, distance)) / WHEEL_R
        return clamp(v, APPROACH_MIN, FWD_SPEED)

    def _arc_step(self, dt, ds, side_l, side_r, turn_rate, front):
        """Roll through a corner at constant speed with yaw rate = v / R, tracking a heading profile
        that runs from the real entry heading to the exact target heading."""
        L = ARC_R * ARC_STRETCH * math.pi / 2.0      # length of the (stretched) heading profile
        self.arc_s += ds
        self.arc_t += dt
        inner = side_l if self.arc_sign > 0 else side_r      # left sensor for a left turn, else right
        outer = side_r if self.arc_sign > 0 else side_l
        self.arc_inner_min = min(self.arc_inner_min, inner)
        if not self.arc_info:
            self.arc_info = {"v": self.v_meas, "sides": (side_l, side_r),
                             "yaw_err": math.degrees(wrap(self.arc_theta0 - self.theta))}
            self.arc_th_start = self.theta
            self.arc_delta = wrap(self.arc_theta0 + self.arc_sign * math.pi / 2.0 - self.theta)
        if self.arc_t > 8.0:
            return self._stop_with_error("arc timeout")
        rel = self.arc_sign * wrap(self.theta - self.arc_theta0)       # heading turned so far, 0..pi/2
        predicted = rel + max(0.0, self.arc_sign * turn_rate) * ARC_LEAD_T
        tgt_h = self.arc_theta0 + self.arc_sign * math.pi / 2.0
        if (self.arc_s >= L + ARC_EXTRA_DIST and predicted < math.pi / 2 - 0.02
                and abs(wrap(tgt_h - self.theta)) > ARC_FIX_TOL and self.arc_t < ARC_FIX_T
                and front >= RECOVER_FRONT):
            # Ran out of distance short of the target: stop driving forward and finish the rotation in
            # place instead of a recovery + re-plan.
            err = wrap(tgt_h - self.theta)
            w = clamp(math.sqrt(2.0 * TURN_ALPHA * abs(err)) * TRACK / 2.0 / WHEEL_R, TURN_MIN, TURN_MAX)
            self.cmd_speed = 0.0
            return (-w, w) if err > 0 else (w, -w)
        if predicted >= math.pi / 2 - 0.02 or self.arc_s >= L + ARC_EXTRA_DIST:
            tgt = self.arc_theta0 + self.arc_sign * math.pi / 2
            info = self.arc_info
            exit_err = abs(wrap(tgt - self.theta))
            self.log(f"arc #{self.arc_n}: entry v={info['v']:.2f} m/s sides=({info['sides'][0]:.3f},"
                     f"{info['sides'][1]:.3f}) yaw_err={info['yaw_err']:+.1f} deg | exit "
                     f"yaw_err={math.degrees(wrap(tgt - self.theta)):+.1f} deg sides=({side_l:.3f},"
                     f"{side_r:.3f}) front={front:.3f} v={self.v_meas:.2f} t={self.arc_t:.2f}s "
                     f"inner_min={self.arc_inner_min:.3f} g={self._ghost_dist():.2f}m")
            if exit_err > RECOVER_ARC_YAW or front < RECOVER_FRONT:
                return self._start_recovery(
                    f"bad arc exit, yaw_err {math.degrees(exit_err):.0f} deg, front {front:.3f}")
            self._advance()                        # the next action is the run that continues
            if self.mode == RUN:
                return self._lane_command(self.run_dir, side_l, side_r, self._ramp(FWD_SPEED, dt),
                                          AFTER_ARC_OFFSET)
            return 0.0, 0.0
        self.cmd_speed = ARC_SPEED
        th_ref = self.arc_th_start + self.arc_delta * min(1.0, self.arc_s / L)
        omega = self.arc_delta / L * ARC_V + K_ARC * wrap(th_ref - self.theta)
        if self.arc_s < ARC_INNER_PHASE * L:
            if inner < ARC_INNER_MIN:
                omega -= self.arc_sign * ARC_INNER_GAIN * (ARC_INNER_MIN - inner)   # rotate less: drift outward
            elif outer < ARC_OUTER_MIN:
                omega += self.arc_sign * ARC_OUTER_GAIN * (ARC_OUTER_MIN - outer)  # rotate more: drift inward
        omega = clamp(omega, -4.0, 4.0)
        diff = abs(omega) * TRACK / 2.0 / WHEEL_R
        speed = min(ARC_SPEED, max(3.0, ARC_WHEEL_CAP - diff))     # rotation has priority over speed
        self.cmd_speed = speed
        return drive_vw(speed, omega)

    # ------------------------------------------------------------- main step
    def decide(self, s):
        """One sensor frame in, one wheel command out -> (left, right) rad/s."""
        try:
            dt = float(s["dt"])
            ticks = (int(s["ticks_l"]), int(s["ticks_r"]))
            turn_rate = float(s["gyro"][2])
            side_l = clean(float(s["fl"]))
            side_r = clean(float(s["fr"]))
            sl_c = clean(float(s["sl"]))
            sr_c = clean(float(s["sr"]))
        except (KeyError, TypeError, ValueError, IndexError):
            return 0.0, 0.0
        if dt <= 0:
            return 0.0, 0.0
        if math.isnan(turn_rate):
            turn_rate = 0.0
        front = min(sl_c, sr_c)
        self.elapsed += dt
        self.frame += 1

        if self.prev_ticks is None:
            self.prev_ticks = ticks
        ds = ((ticks[0] - self.prev_ticks[0]) + (ticks[1] - self.prev_ticks[1])) * M_PER_TICK / 2.0
        self.prev_ticks = ticks
        self.v_meas += 0.3 * (ds / dt - self.v_meas)      # filtered forward speed, m/s

        self._observe_ghost()
        if self.frame % 25 == 0:
            self._resync()

        if self.done or not self.have_pellets:
            return 0.0, 0.0

        if self.actions is None:
            self.theta = 0.0                       # standing still facing east: zero the gyro drift
            if not self._model_ready():
                return 0.0, 0.0
            if not self._start_plan(front):
                return 0.0, 0.0
            return 0.0, 0.0

        self.theta = wrap(self.theta + turn_rate * dt)
        return self._drive(dt, ds, turn_rate, side_l, side_r, sl_c, sr_c, front)

    def _drive(self, dt, ds, turn_rate, side_l, side_r, sl_c, sr_c, front):
        self._update_prog()
        hold = self._ghost_hold()
        if hold:
            self.hold_time += dt
            if not self.was_holding:
                self.log(f"holding for ghost at prog {self.prog:.2f} m")
        else:
            self.action_time += dt
            if self.was_holding:
                self.log("ghost clear, moving on")
        self.was_holding = hold

        if self.mode == DONE:
            return 0.0, 0.0

        # watchdog: notice a hit / wedge and recover instead of driving on blind
        if self.rec_grace > 0.0:
            self.rec_grace -= dt
        if self.mode in (RUN, ARC) and self.rec_grace <= 0.0:
            why = self._watchdog(dt, front)
            if why:
                return self._start_recovery(why)

        if self.mode == RECOVER:
            return self._recover_step(ds, front)

        # ALIGN: bring the axle onto the centre of the start cell
        if self.mode == ALIGN:
            self.s += ds
            rem = -self.s
            if rem <= 0.0:
                self.s = 0.0
                self.pending_s = 0.0
                self.cmd_speed = 0.0
                self._begin_action()
                return 0.0, 0.0
            if hold:
                self.cmd_speed = 0.0
                return self._lane_command(0, side_l, side_r, 0.0)
            return self._lane_command(0, side_l, side_r,
                                      self._ramp(self._braking_speed(rem), dt))

        if self.mode == ARC:
            return self._arc_step(dt, ds, side_l, side_r, turn_rate, front)

        if self.mode in (TURN, SETTLE):
            self.cmd_speed = 0.0
            if hold:
                return 0.0, 0.0
            target = self.actions[self.action_index][1] * math.pi / 2
            err_h = wrap(target - self.theta)
            if self.mode == TURN:
                if abs(self.turn_planned) > 3.0:   # 180 deg: keep turning the planned way round
                    if self.turn_planned > 0 and err_h < -0.5:
                        err_h += 2.0 * math.pi
                    elif self.turn_planned < 0 and err_h > 0.5:
                        err_h -= 2.0 * math.pi
                if self.action_time > 20.0:
                    return self._stop_with_error(f"turn timeout, heading error {err_h:.3f} rad")
                if abs(err_h) < TURN_TOL:
                    self.mode = SETTLE
                    self.turn_settle = 0.0
                    return 0.0, 0.0
                yaw_cmd = math.sqrt(2.0 * TURN_ALPHA * abs(err_h))
                w = clamp(yaw_cmd * TRACK / 2.0 / WHEEL_R, TURN_MIN, TURN_MAX)
                w = w if err_h > 0 else -w
                return -w, w
            # SETTLE
            if abs(err_h) > 3.0 * TURN_TOL:
                self.mode = TURN
                return 0.0, 0.0
            if abs(err_h) < TURN_TOL and abs(turn_rate) <= TURN_RATE_TOL:
                self.turn_settle += dt
            else:
                self.turn_settle = 0.0
            if self.turn_settle >= TURN_SETTLE_TIME and abs(err_h) < TURN_TOL and abs(turn_rate) <= TURN_RATE_TOL:
                self.log(f"pivot {math.degrees(self.turn_planned):+.0f} deg took {self.action_time:.2f}s, "
                         f"residual {math.degrees(err_h):+.1f} deg")
                self._advance()
                return 0.0, 0.0
            omega = clamp(K_HEADING * err_h, -MAX_OMEGA, MAX_OMEGA)
            return drive_vw(0.0, omega)

        # EXIT: straight through the gap
        if self.mode == EXITING:
            self.rem -= ds
            if self.rem <= 0.0:
                self._advance()
                return 0.0, 0.0
            if self.action_time > STALL_TIMEOUT:
                return self._stop_with_error("exit timeout")
            if hold:
                self.cmd_speed = 0.0
                return self._lane_command(EXIT_HEADING, side_l, side_r, 0.0)
            return self._lane_command(EXIT_HEADING, side_l, side_r, self._ramp(EXIT_SPEED, dt))

        # RUN
        action = self.actions[self.action_index]
        route_dir = self.run_dir
        err_h = wrap(route_dir * math.pi / 2 - self.theta)
        aligned = abs(err_h) < CORRECTION_MAX_YAW_ERR
        self.s += ds

        # Odometry correction 1: side-wall edges seen by the side sensors match map landmarks.
        left_wall = side_l < SIDE_WALL
        right_wall = side_r < SIDE_WALL
        if aligned:
            for side, prev, now in (("L", self.prev_left_wall, left_wall),
                                    ("R", self.prev_right_wall, right_wall)):
                if prev is None or prev == now:
                    continue
                expected = [m[2] for m in self.landmarks if m[0] == side and m[1] == now]
                if expected:
                    nearest = min(expected, key=lambda e: abs(e - self.s))
                    if abs(nearest - self.s) < EVENT_MATCH:
                        self.s = nearest
        self.prev_left_wall = left_wall
        self.prev_right_wall = right_wall

        # Odometry correction 2: the front wall distance, when square to the wall.
        if (self.front_wall_s is not None and abs(err_h) < FRONT_MAX_YAW_ERR
                and front < FRONT_RANGE and abs(sl_c - sr_c) < 0.02):
            s_est = self.front_wall_s + FRONT_AT_CENTER - front
            if abs(s_est - self.s) < FRONT_GATE:
                self.s = s_est

        if self._next_kind() == "arc" and self.s_end - self.s < ARC_GATE_DIST:
            self._arc_gate(err_h, side_l, side_r)  # also fires for a chained arc, where the run is 0 m long

        rem = self.s_end - self.s
        if rem <= 0.0:
            self.cell = action[3]
            self._advance()
            if self.mode == ARC:
                return self._arc_step(dt, 0.0, side_l, side_r, turn_rate, front)
            if self.mode == EXITING:
                return self._lane_command(EXIT_HEADING, side_l, side_r,
                                          self._ramp(EXIT_SPEED, dt))
            return 0.0, 0.0
        if self.action_time > STALL_TIMEOUT:
            return self._stop_with_error(f"run timeout, {rem:.3f} m left")

        if self.action_time >= self.next_progress_log:
            self.log(f"run: s={self.s:.3f}/{self.s_end:.3f} side=({side_l:.3f},{side_r:.3f}) "
                     f"front={front:.3f} yaw={math.degrees(self.theta):.1f} v={self.v_meas:.2f} "
                     f"tgt={math.degrees(route_dir * math.pi / 2):.0f} g={self._ghost_dist():.2f}m")
            self.next_progress_log += 2.0

        if hold or front < FRONT_STOP:
            self.cmd_speed = 0.0
            return self._lane_command(route_dir, side_l, side_r, 0.0)

        speed = FWD_SPEED
        next_kind = self._next_kind()
        if next_kind == "turn":
            speed = self._braking_speed(rem)
            if rem < SHORT_RUN_DIST:               # short run into a pivot: do not overshoot the centre
                speed = min(speed, ARC_SPEED)
            turn_to = self.actions[self.action_index + 1][1]
            if (turn_to - route_dir) % 4 == 2 and rem < 0.25:   # 180 deg pivot: only ~1.5 cm of clearance
                speed = min(speed, PIVOT180_APPROACH)
        elif next_kind == "arc":                   # arrive at the arc start doing exactly ARC_SPEED
            v = math.sqrt(ARC_V ** 2 + 2.0 * ARC_DECEL * max(0.0, rem - ARC_LAG_DIST)) / WHEEL_R
            speed = min(speed, clamp(v, ARC_SPEED, FWD_SPEED))
        if front < FRONT_TRUST and aligned:
            speed = min(speed, self._braking_speed(front - FRONT_AT_CENTER))
            if next_kind == "arc":
                speed = max(speed, ARC_SPEED)
        if self.wait_dist is not None:
            speed = min(speed, self._gate_speed(self.wait_dist))
        max_off = (AFTER_ARC_OFFSET if (self.after_arc and self.s < ARC_HALF + AFTER_ARC_DIST)
                   else MAX_HEADING_OFFSET)
        return self._lane_command(route_dir, side_l, side_r, self._ramp(speed, dt), max_off)


# ----------------------------------------------------------------------------
def main():
    maze = Maze.official()
    print(f"official maze {maze_2b.MAZE_ID}: {maze.grid}x{maze.grid}, "
          f"spawn row {maze.spawn_row}, exit row {maze.exit_row}")

    controller = Controller(maze)
    client = _mqtt_client()

    def on_message(cli, userdata, msg):
        payload = json.loads(msg.payload.decode())
        if msg.topic == TOPIC_PELLETS:
            controller.on_pellets(payload)
            return
        if msg.topic == TOPIC_GHOST:
            controller.ghost = payload
            return
        if msg.topic == TOPIC_RESULT:
            print(f"run over: solved={payload['solved']}, collisions={payload['collisions']}")
            controller.done = True
            cli.publish(TOPIC_WHEEL_VEL, json.dumps({"left": 0.0, "right": 0.0}))
            return

        left, right = controller.decide(payload)
        left = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(left)))
        right = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(right)))
        cli.publish(TOPIC_WHEEL_VEL, json.dumps({"left": left, "right": right}))

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_PELLETS)
    client.subscribe(TOPIC_GHOST)
    client.subscribe(TOPIC_RESULT)
    client.subscribe(TOPIC_SENSORS)
    print("connected; waiting for the simulator")
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
