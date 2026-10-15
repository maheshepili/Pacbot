"""Plan and drive PacBot through the pellet and out of the maze."""
import json
import math

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_POSES = "pacbot/poses"
TOPIC_PELLETS = "pacbot/pellets"
TOPIC_RESULT = "pacbot/result"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

N = 15
PITCH = 0.22
WALL_HALF = 0.005

EXIT = (N - 1, N - 1, 3)

STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))

LAYOUT = (
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
    "|                           |       |               |       |",
    "+---+---+   +---+   +   +   +   +   +---+   +---+   +---+   +",
    "|           |           |   |   |       |       |           |",
    "+   +   +   +   +---+---+---+   +---+   +   +   +---+---+   +",
    "|   |           |               |   |   |   |   |   |       |",
    "+   +---+---+---+   +---+   +---+   +   +---+   +   +   +---+",
    "|   |               |               |       |   |           |",
    "+   +   +---+---+---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |           |       |   |       |       |       |",
    "+   +---+   +   +   +---+---+   +   +   +---+---+   +---+   +",
    "|           |   |       |       |   |       |       |   |   |",
    "+   +   +   +   +---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |       |   |   |   |       |       |       |   |",
    "+   +   +   +---+---+   +   +   +   +   +   +---+   +   +   +",
    "|   |   |           |       |       |           |   |       |",
    "+   +   +---+---+   +   +---+---+   +---+---+   +   +---+---+",
    "|   |   |           |           |       |       |           |",
    "+   +   +   +---+---+---+---+   +   +   +   +---+---+---+   +",
    "|   |   |               |       |   |   |           |       |",
    "+   +---+---+   +---+   +   +---+   +   +---+   +   +   +---+",
    "|   |       |   |           |           |       |   |   |   |",
    "+   +   +   +   +   +---+   +---+---+   +   +---+   +   +   +",
    "|   |   |           |   |               |   |               |",
    "+   +   +---+---+---+   +---+---+---+   +   +---+   +---+   +",
    "|   |       |                   |       |       |       |   |",
    "+   +   +   +   +   +   +---+   +   +---+---+   +---+   +   +",
    "|   |   |       |   |           |   |       |   |       |   |",
    "+   +   +---+---+   +   +   +   +   +---+   +   +---+---+   +",
    "|               |           |               |               |",
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ E +",
)

def _wall_table():
    """{(row, col, d): True if side d of cell (row, col) is walled}."""
    walls = {}
    for row in range(N):
        for col in range(N):
            r, c = 2 * row + 1, 4 * col
            walls[row, col, 0] = LAYOUT[r][c + 4] == "|"
            walls[row, col, 1] = LAYOUT[r - 1][c + 1:c + 4] == "---"
            walls[row, col, 2] = LAYOUT[r][c] == "|"
            walls[row, col, 3] = LAYOUT[r + 1][c + 1:c + 4] == "---"
    return walls

WALLS = _wall_table()

def cell_centre(cell):
    """World (x, y) of the centre of cell (row, col)."""
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH

def inside(row, col):
    return 0 <= row < N and 0 <= col < N

def neighbours(cell):
    """Open neighbouring cells of `cell`, as (direction, cell) pairs."""
    row, col = cell
    out = []
    for d in range(4):
        n = (row + STEP[d][0], col + STEP[d][1])
        if not WALLS[row, col, d] and inside(*n):
            out.append((d, n))
    return out

WHEEL_R = 0.017
TRACK = 0.078
TICKS_PER_REV = 360
M_PER_TICK = 2 * math.pi * WHEEL_R / TICKS_PER_REV
WHEEL_MAX = 30.0
AXLE_BACK = 0.033
NO_HIT = 2.0
TURN_TOL = 0.025
TURN_RATE_TOL = 0.04
TURN_SETTLE_TIME = 0.06

SENSORS = (
    ("sl", (-0.005, 0.034), 0.0),
    ("sr", (-0.005, -0.034), 0.0),
    ("fl", (0.036, 0.005), math.pi / 2),
    ("fr", (0.036, -0.005), -math.pi / 2),
)

WALL_THICKNESS = 2 * WALL_HALF
CELL = PITCH
INNER = PITCH - WALL_THICKNESS
SIDE_LATERAL = 0.005
SIDE_AHEAD = 0.036 + AXLE_BACK
FRONT_AHEAD = AXLE_BACK - 0.005

SIDE_CENTERED = INNER / 2 - SIDE_LATERAL
FRONT_AT_CENTER = INNER / 2 - FRONT_AHEAD
_WALL_END = CELL / 2 + WALL_THICKNESS / 2
GAP_TO_CENTER = CELL - _WALL_END + SIDE_AHEAD
WALL_TO_CENTER = _WALL_END + SIDE_AHEAD
SIDE_WALL = SIDE_CENTERED + 0.27 * CELL
FRONT_TRUST = FRONT_AT_CENTER + 1.00 * CELL
GATE = 0.36 * CELL
SENSOR_MAX = 1.0

EXIT_DISTANCE = 0.30

FWD_SPEED = 20.0
APPROACH_MIN = 5.0
BRAKE_ACCEL = 2.0
EXIT_SPEED = 14.0
K_HEADING = 6.0
K_CENTER = 4.0
MAX_HEADING_OFFSET = math.radians(12)
MAX_OMEGA = 3.0
CORRECTION_MAX_YAW_ERR = math.radians(8)
TURN_ALPHA = 40.0
TURN_MIN = 2.0
TURN_MAX = 20.0
STALL_TIMEOUT = 45.0
SPEED_SLEW = 50.0
EVENT_MATCH = 0.105
FRONT_RANGE = 0.60
FRONT_GATE = 0.12
FRONT_MAX_YAW_ERR = math.radians(3)
FRONT_STOP = 0.045

def wrap(a):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wheels(v, omega):
    """Convert forward speed and turn rate to left and right wheel speeds."""
    left = (v - omega * TRACK / 2) / WHEEL_R
    right = (v + omega * TRACK / 2) / WHEEL_R
    return clamp(left, -WHEEL_MAX, WHEEL_MAX), clamp(right, -WHEEL_MAX, WHEEL_MAX)

def drive_vw(wheel_speed, omega):
    """Convert a wheel speed and turn rate to left and right wheel speeds."""
    diff = omega * TRACK / 2.0 / WHEEL_R
    return (clamp(wheel_speed - diff, -WHEEL_MAX, WHEEL_MAX),
            clamp(wheel_speed + diff, -WHEEL_MAX, WHEEL_MAX))

def clean(v):
    """ToF reading -> usable distance; 'nothing in range' becomes SENSOR_MAX."""
    if v is None or math.isnan(v) or v < 0.0:
        return SENSOR_MAX
    return min(SENSOR_MAX, v)

import heapq
from functools import lru_cache

@lru_cache(maxsize=None)
def _wall_boxes(cell):
    """Axis-aligned wall boxes bordering one cell, in world coordinates."""
    row, col = cell
    x, y = cell_centre(cell)
    half = PITCH / 2.0
    boxes = []
    if WALLS[row, col, 0]:
        boxes.append((x + half - WALL_HALF, y - half,
                      x + half + WALL_HALF, y + half))
    if WALLS[row, col, 2]:
        boxes.append((x - half - WALL_HALF, y - half,
                      x - half + WALL_HALF, y + half))
    if WALLS[row, col, 1]:
        boxes.append((x - half, y + half - WALL_HALF,
                      x + half, y + half + WALL_HALF))
    if WALLS[row, col, 3]:
        boxes.append((x - half, y - half - WALL_HALF,
                      x + half, y - half + WALL_HALF))
    return tuple(boxes)

def _box_overlap_with_chassis(cx, cy, angle, wall, margin):
    """SAT collision check with a small clearance margin around the chassis."""
    x0, y0, x1, y1 = wall
    hx, hy = 0.046 + margin, 0.043 + margin
    ca, sa = math.cos(angle), math.sin(angle)
    robot = [(cx + sx * hx * ca - sy * hy * sa,
              cy + sx * hx * sa + sy * hy * ca)
             for sx, sy in ((-1, -1), (-1, 1), (1, -1), (1, 1))]
    obstacle = ((x0, y0), (x0, y1), (x1, y0), (x1, y1))
    for ux, uy in ((1.0, 0.0), (0.0, 1.0), (ca, sa), (-sa, ca)):
        rproj = [x * ux + y * uy for x, y in robot]
        wproj = [x * ux + y * uy for x, y in obstacle]
        if max(rproj) <= min(wproj) or max(wproj) <= min(rproj):
            return False
    return True

@lru_cache(maxsize=None)
def _turn_sweep_clear(cell, heading, target_heading):
    """Whether a pivot with the AXLE on the cell centre clears the walls."""
    cell = tuple(cell)
    start_angle = heading * math.pi / 2.0
    delta = wrap((target_heading - heading) * math.pi / 2.0)
    axle_x, axle_y = cell_centre(cell)
    row, col = cell
    walls = set()
    for r in range(max(0, row - 1), min(N, row + 2)):
        for c in range(max(0, col - 1), min(N, col + 2)):
            walls.update(_wall_boxes((r, c)))
    samples = max(1, math.ceil(abs(delta) / (math.pi / 36.0)))
    for i in range(samples + 1):
        angle = start_angle + delta * i / samples
        chassis_x = axle_x + AXLE_BACK * math.cos(angle)
        chassis_y = axle_y + AXLE_BACK * math.sin(angle)
        if any(_box_overlap_with_chassis(chassis_x, chassis_y, angle, wall, 0.008)
               for wall in walls):
            return False
    return True

def _shortest_path(start, heading, goal, goal_heading=None):
    """Prefer fewer unsafe pivots, then fewer pivots, then shorter routes."""
    if not inside(*start) or not inside(*goal) or heading not in range(4):
        raise ValueError(f"invalid route request: {start=} {heading=} {goal=}")
    initial = (start, heading)
    best = {initial: (0, 0, 0)}
    parent = {}
    queue = [(0, 0, 0, start, heading)]
    finish = None
    finish_score = None
    while queue:
        risk, turns, steps, cell, direction = heapq.heappop(queue)
        state = (cell, direction)
        score = (risk, turns, steps)
        if score != best.get(state):
            continue
        if finish_score is not None and score >= finish_score:
            break
        if cell == goal:
            terminal_turn = goal_heading is not None and direction != goal_heading
            terminal_risk = int(terminal_turn and
                                not _turn_sweep_clear(cell, direction, goal_heading))
            candidate_score = (risk + terminal_risk,
                               turns + int(terminal_turn), steps)
            if finish_score is None or candidate_score < finish_score:
                finish, finish_score = state, candidate_score
            if terminal_risk == 0:
                continue
        for move_dir, nxt in neighbours(cell):
            if WALLS[cell[0], cell[1], move_dir] or not inside(*nxt):
                continue
            is_turn = move_dir != direction
            unsafe_turn = int(is_turn and
                              not _turn_sweep_clear(cell, direction, move_dir))
            next_score = (risk + unsafe_turn, turns + int(is_turn), steps + 1)
            next_state = (nxt, move_dir)
            if next_score < best.get(next_state, (10**9, 10**9, 10**9)):
                best[next_state] = next_score
                parent[next_state] = state
                heapq.heappush(queue, (*next_score, nxt, move_dir))
    if finish is None:
        raise ValueError(f"no route from {start} to {goal}")
    states = [finish]
    while states[-1] != initial:
        states.append(parent[states[-1]])
    states.reverse()
    final_heading = goal_heading if goal_heading is not None else states[-1][1]
    return [(states[i + 1][1], states[i + 1][0]) for i in range(len(states) - 1)], final_heading

def _actions_for_legs(start, heading, pellet):
    """Return direction runs for start->pellet->exit, ending south at exit."""
    exit_cell = (N - 1, N - 1)
    first, h = _shortest_path(start, heading, pellet)
    second, h = _shortest_path(pellet, h, exit_cell, goal_heading=3)
    edges = first + second
    actions = []
    current_heading = heading
    for d, next_cell in edges:
        if d != current_heading:
            actions.append(("turn", d))
            current_heading = d
        if actions and actions[-1][0] == "run":
            actions[-1] = ("run", actions[-1][1] + 1, d, next_cell)
        else:
            actions.append(("run", 1, d, next_cell))

    if WALLS[exit_cell[0], exit_cell[1], EXIT[2]]:
        raise ValueError("designated south exit is blocked in WALLS")
    if current_heading != EXIT[2]:
        actions.append(("turn", EXIT[2]))
    actions.append(("exit", EXIT_DISTANCE))
    return actions

def plan_route(start, heading, pellet):
    """Plan a wall-validated route through the pellet and out the south exit."""
    start, pellet = tuple(start), tuple(pellet)
    if not inside(*start) or not inside(*pellet):
        raise ValueError(f"spawn or pellet outside maze: {start}, {pellet}")
    return _actions_for_legs(start, int(heading), pellet)

ALIGN = "ALIGN"
RUN = "RUN"
TURN = "TURN"
SETTLE = "SETTLE"
EXITING = "EXIT"
DONE = "DONE"

class Solver:
    """One sensor frame in, wheel speeds out."""

    def __init__(self):
        self.t = 0.0
        self.spawn = None
        self.pellet = None
        self.actions = None
        self.action_index = 0
        self.prev_ticks = None
        self.done = False
        self.failed = False
        self.theta = None
        self.command_heading = None
        self.mode = None
        self.rem = 0.0
        self.cell = None
        self.s = 0.0
        self.s_end = 0.0
        self.run_dir = 0
        self.landmarks = []
        self.front_wall_s = None
        self.cmd_speed = 0.0
        self.prev_left_wall = None
        self.prev_right_wall = None
        self.turn_planned = 0.0
        self.turn_settle = 0.0
        self.action_time = 0.0
        self.next_progress_log = 1.0

    def log(self, text):
        print(f"[{self.t:7.2f}s] {text}", flush=True)

    def on_poses(self, msg):
        bot, d = msg.get("bot"), msg.get("dir")
        if self.actions is None and isinstance(bot, list) and len(bot) == 2 and d in range(4):
            cell = tuple(int(v) for v in bot)
            if inside(*cell):
                self.spawn = (int(d), cell)
                self.theta = int(d) * math.pi / 2
                self.command_heading = int(d)
                self.cell = cell
                self.log(f"spawn {cell} facing {d}")
            else:
                self.log(f"invalid spawn message: {msg}")

    def on_pellets(self, msg):
        pellet = msg.get("pellet")
        if self.actions is None and isinstance(pellet, list) and len(pellet) == 2:
            cell = tuple(int(v) for v in pellet)
            if inside(*cell):
                self.pellet = cell
                self.log(f"pellet {cell}")
            else:
                self.log(f"invalid pellet message: {msg}")

    def _stop_with_error(self, message):
        self.failed = True
        self.done = True
        self.mode = DONE
        self.log(f"ERROR: {message}; stopping")
        return 0.0, 0.0

    def _begin_action(self):
        action = self.actions[self.action_index]
        kind = action[0]
        self.action_time = 0.0
        self.next_progress_log = 1.0
        self.prev_left_wall = None
        self.prev_right_wall = None
        if kind == "turn":
            self.mode = TURN
            self.turn_settle = 0.0

            self.turn_planned = wrap((action[1] - self.command_heading) * math.pi / 2)
        elif kind == "run":
            self.mode = RUN
            self.run_dir = action[2]
            self.s = 0.0
            self.s_end = action[1] * CELL
            self.landmarks, self.front_wall_s = self._run_geometry(self.cell, action[2], action[1])
        elif kind == "exit":
            self.mode = EXITING
            self.rem = action[1]
        self.log(f"starting action {self.action_index + 1}/{len(self.actions)}: {action}")

    def _advance(self):
        completed = self.actions[self.action_index]
        self.log(f"completed action {completed}")
        if completed[0] == "turn":
            self.command_heading = completed[1]
        self.action_index += 1
        if self.action_index >= len(self.actions):
            self.mode = DONE
            self.done = True
            self.log("route complete; awaiting simulator result")
        else:
            self._begin_action()

    def _next_is_turn(self):
        nxt = self.action_index + 1
        return nxt < len(self.actions) and self.actions[nxt][0] == "turn"

    def _lane_command(self, route_dir, side_l, side_r, speed):
        """Hold the compass heading and centre between the walls (Task 1B)."""
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
        offset = clamp(K_CENTER * err_c, -MAX_HEADING_OFFSET, MAX_HEADING_OFFSET)
        omega = clamp(K_HEADING * wrap(err_h + offset), -MAX_OMEGA, MAX_OMEGA)
        return drive_vw(speed, omega)

    @staticmethod
    def _run_geometry(cell, d, n):
        """What the MAP says the sensors should see on a straight run of n cells."""
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
        """Ramp speed commands to reduce wheel slip."""
        if target > self.cmd_speed:
            self.cmd_speed = min(target, self.cmd_speed + SPEED_SLEW * dt)
        else:
            self.cmd_speed = target
        return self.cmd_speed

    @staticmethod
    def _braking_speed(distance):
        """Wheel speed from which we can still stop within `distance`."""
        v = math.sqrt(2.0 * BRAKE_ACCEL * max(0.0, distance)) / WHEEL_R
        return clamp(v, APPROACH_MIN, FWD_SPEED)

    def step(self, s):
        try:
            dt = float(s["dt"])
            ticks = (int(s["ticks_l"]), int(s["ticks_r"]))
            gyro = s["gyro"]
            if dt <= 0 or len(gyro) < 3:
                raise ValueError("invalid dt or gyro vector")
            turn_rate = float(gyro[2])
            side_l = clean(float(s["fl"]))
            side_r = clean(float(s["fr"]))
            sl_c = clean(float(s["sl"]))
            sr_c = clean(float(s["sr"]))
            front = min(sl_c, sr_c)
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            self.log(f"invalid sensor frame: {exc}")
            return self._stop_with_error("invalid sensor frame")
        if math.isnan(turn_rate):
            turn_rate = 0.0
        self.t += dt
        if self.prev_ticks is None:
            self.prev_ticks = ticks
        d_left = (ticks[0] - self.prev_ticks[0]) * M_PER_TICK
        d_right = (ticks[1] - self.prev_ticks[1]) * M_PER_TICK
        self.prev_ticks = ticks
        ds = (d_left + d_right) / 2.0
        if self.theta is not None:
            self.theta = wrap(self.theta + turn_rate * dt)

        if self.done or self.spawn is None or self.pellet is None:
            return 0.0, 0.0
        if self.actions is None:
            try:
                self.actions = plan_route(self.spawn[1], self.spawn[0], self.pellet)
                self.log(f"validated plan ({len(self.actions)} actions): {self.actions}")
            except (ValueError, IndexError) as exc:
                return self._stop_with_error(f"route planning failed: {exc}")

            # Align the axle with the cell centre before pivoting.
            self.mode = ALIGN
            self.rem = AXLE_BACK
            self.action_time = 0.0
        self.action_time += dt

        if self.mode == DONE:
            return 0.0, 0.0
        if self.mode in (ALIGN, TURN, SETTLE):
            self.cmd_speed = 0.0

        if self.mode == ALIGN:
            err_h = wrap(self.command_heading * math.pi / 2 - self.theta)
            omega = clamp(K_HEADING * err_h, -MAX_OMEGA, MAX_OMEGA)
            self.rem -= ds
            if self.rem <= 0.0:
                self._begin_action()
            return drive_vw(APPROACH_MIN, omega)

        if self.mode == TURN:
            target = self.actions[self.action_index][1] * math.pi / 2
            err_h = wrap(target - self.theta)

            # Preserve the planned direction during a 180-degree turn.
            if abs(self.turn_planned) > 3.0:
                if self.turn_planned > 0 and err_h < -0.5:
                    err_h += 2.0 * math.pi
                elif self.turn_planned < 0 and err_h > 0.5:
                    err_h -= 2.0 * math.pi
            if self.action_time >= self.next_progress_log:
                self.log(f"turn progress: heading error {err_h:+.3f} rad")
                self.next_progress_log += 1.0
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

        if self.mode == SETTLE:
            target = self.actions[self.action_index][1] * math.pi / 2
            err_h = wrap(target - self.theta)
            if abs(err_h) > 3.0 * TURN_TOL:
                self.mode = TURN
                return 0.0, 0.0
            if abs(err_h) < TURN_TOL and abs(turn_rate) <= TURN_RATE_TOL:
                self.turn_settle += dt
            else:
                self.turn_settle = 0.0
            if self.turn_settle >= TURN_SETTLE_TIME:
                self._advance()
                return 0.0, 0.0
            omega = clamp(K_HEADING * err_h, -MAX_OMEGA, MAX_OMEGA)
            return drive_vw(0.0, omega)

        if self.mode == EXITING:
            self.rem -= ds
            if self.rem <= 0.0:
                self._advance()
                return 0.0, 0.0
            if self.action_time > STALL_TIMEOUT:
                return self._stop_with_error("exit timeout")
            return self._lane_command(EXIT[2], side_l, side_r, self._ramp(EXIT_SPEED, dt))

        action = self.actions[self.action_index]
        route_dir = self.run_dir
        err_h = wrap(route_dir * math.pi / 2 - self.theta)
        aligned = abs(err_h) < CORRECTION_MAX_YAW_ERR

        self.s += ds

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

        if (self.front_wall_s is not None and abs(err_h) < FRONT_MAX_YAW_ERR
                and front < FRONT_RANGE and abs(sl_c - sr_c) < 0.02):
            s_est = self.front_wall_s + FRONT_AT_CENTER - front
            if abs(s_est - self.s) < FRONT_GATE:
                self.s = s_est

        rem = self.s_end - self.s
        if rem <= 0.0:
            self.cell = action[3]
            self._advance()
            if self.mode == EXITING:
                return self._lane_command(EXIT[2], side_l, side_r, self._ramp(EXIT_SPEED, dt))
            return 0.0, 0.0

        if self.action_time > STALL_TIMEOUT:
            return self._stop_with_error(f"run timeout, {rem:.3f} m left")
        if self.action_time >= self.next_progress_log:
            self.log(f"run: s={self.s:.3f}/{self.s_end:.3f} "
                     f"side=({side_l:.3f},{side_r:.3f}) front={front:.3f} "
                     f"yaw={math.degrees(self.theta):.1f} "
                     f"tgt={math.degrees(route_dir * math.pi / 2):.0f}")
            self.next_progress_log += 1.0

        if front < FRONT_STOP:
            self.cmd_speed = 0.0
            return self._lane_command(route_dir, side_l, side_r, 0.0)

        speed = FWD_SPEED
        if self._next_is_turn():
            speed = self._braking_speed(rem)

        if front < FRONT_TRUST and aligned:
            speed = min(speed, self._braking_speed(front - FRONT_AT_CENTER))
        return self._lane_command(route_dir, side_l, side_r, self._ramp(speed, dt))

def _client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()

def main():
    solver = Solver()
    client = _client()

    def send(cli, left, right):

        cli.publish(TOPIC_WHEEL_VEL, json.dumps({"left": float(left), "right": float(right)}))

    def on_message(cli, _userdata, msg):
        if msg.topic == TOPIC_SENSORS:
            send(cli, *solver.step(json.loads(msg.payload)))
        elif msg.topic == TOPIC_POSES:
            if msg.payload:
                solver.on_poses(json.loads(msg.payload))
        elif msg.topic == TOPIC_PELLETS:
            if msg.payload:
                solver.on_pellets(json.loads(msg.payload))
        elif msg.topic == TOPIC_RESULT:
            solver.log(f"result: {msg.payload.decode()}")
            solver.done = True
            send(cli, 0.0, 0.0)
            cli.disconnect()

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    for topic in (TOPIC_POSES, TOPIC_PELLETS, TOPIC_RESULT, TOPIC_SENSORS):
        client.subscribe(topic)
    print("connected, waiting for the simulator", flush=True)
    client.loop_forever()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
