import json
import math
import paho.mqtt.client as mqtt


MQTT_HOST = "localhost"
MQTT_PORT = 1883

TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"


WHEEL_RADIUS = 0.017
TRACK_WIDTH = 0.078
AXLE_BEHIND_CENTER = 0.033
SIDE_SENSOR_LATERAL = 0.005
SIDE_SENSOR_AHEAD = 0.069
FRONT_SENSOR_AHEAD = 0.028


WALL_THICKNESS = 0.010
SPAWN_OUTSIDE = 0.040
DEFAULT_INNER_WIDTH = 0.210

CALIB_TIME = 0.12
CALIB_SPEED = 6.0
CALIB_MIN_SAMPLES = 20
CALIB_MAX_SIDE = 0.25


SENSOR_MAX = 1.0
NO_HIT_VALUE = 0.300
NO_HIT_TOL = 0.0005
ESCAPE_RANGE = 3.0
ESCAPE_CELLS = 0.4
ESCAPE_AT_DECISION = 0.03

MAZE_COLS = None

FOLLOW_HAND = "LEFT"


FWD_SPEED = 26.0
APPROACH_MIN = 5.0
BRAKE_ACCEL = 2.0
MAX_WHEEL = 30.0

K_HEADING = 6.0
K_CENTER = 4.0
MAX_HEADING_OFFSET = math.radians(12)
MAX_OMEGA = 3.0
CORRECTION_MAX_YAW_ERR = math.radians(8)

TURN_ALPHA = 40.0
TURN_MIN = 2.0
TURN_MAX = 20.0
TURN_TOL = math.radians(1.5)
TURN_SETTLE = 0.06

GYRO_SIGN = 1.0


CALIBRATE = "CALIBRATE"
FORWARD = "FORWARD"
TURN = "TURN"
SETTLE = "SETTLE"
DONE = "DONE"


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Geometry:

    def __init__(self, inner_width):
        half = inner_width / 2.0

        self.inner = inner_width
        self.cell = inner_width + WALL_THICKNESS

        self.side_centered = half - SIDE_SENSOR_LATERAL
        self.front_at_center = half - FRONT_SENSOR_AHEAD

        wall_end = self.cell / 2.0 + WALL_THICKNESS / 2.0
        self.gap_to_center = self.cell - wall_end + SIDE_SENSOR_AHEAD
        self.wall_to_center = wall_end + SIDE_SENSOR_AHEAD

        self.side_wall = self.side_centered + 0.27 * self.cell
        self.side_open = self.side_centered + 0.36 * self.cell
        self.front_blocked = self.front_at_center + 0.40 * self.cell
        self.front_trust = self.front_at_center + 1.00 * self.cell

        self.gate = 0.36 * self.cell

        self.start_to_first_center = (
            SPAWN_OUTSIDE + self.cell / 2.0 + AXLE_BEHIND_CENTER
        )

    def describe(self):
        return (
            f"cell={self.cell:.3f} inner={self.inner:.3f} "
            f"side_centered={self.side_centered:.3f} "
            f"front_at_center={self.front_at_center:.3f}"
        )


class PacBotController:

    def __init__(self):
        self.state = CALIBRATE
        self.count = 0
        self.time = 0.0

        self.geo = Geometry(DEFAULT_INNER_WIDTH)
        self.calib_sum = 0.0
        self.calib_n = 0
        self.calib_dist = 0.0

        self.yaw = 0.0
        self.heading = 0.0

        self.rem = 0.0
        self.cells = 0
        self.grid_x = -1

        self.prev_left_wall = None
        self.prev_right_wall = None

        self.escape = 0.0
        self.settle = 0.0

        self.last_l = 0.0
        self.last_r = 0.0

    @staticmethod
    def no_hit(v):
        return math.isnan(v) or v < 0.0 or abs(v - NO_HIT_VALUE) < NO_HIT_TOL

    @classmethod
    def clean(cls, v):
        if cls.no_hit(v):
            return SENSOR_MAX
        return min(SENSOR_MAX, v)

    @classmethod
    def escaped(cls, v):
        return cls.no_hit(v) or v > ESCAPE_RANGE

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def drive(self, client, left, right):
        left = self.clamp(left, -MAX_WHEEL, MAX_WHEEL)
        right = self.clamp(right, -MAX_WHEEL, MAX_WHEEL)
        self.last_l, self.last_r = left, right
        client.publish(
            TOPIC_WHEEL_VEL,
            json.dumps({"left": float(left), "right": float(right)})
        )

    def drive_vw(self, client, wheel_speed, omega):
        diff = omega * TRACK_WIDTH / 2.0 / WHEEL_RADIUS
        self.drive(client, wheel_speed - diff, wheel_speed + diff)

    def finish_calibration(self):
        if self.calib_n >= CALIB_MIN_SAMPLES:
            inner = self.calib_sum / self.calib_n + 2.0 * SIDE_SENSOR_LATERAL
        else:
            inner = DEFAULT_INNER_WIDTH
            print("[CALIB] not enough wall readings, using default width")

        self.geo = Geometry(inner)
        self.rem = self.geo.start_to_first_center - self.calib_dist
        self.state = FORWARD
        print(f"[CALIB] {self.geo.describe()} samples={self.calib_n}")

    def start_turn(self, delta, label):
        self.heading = wrap(self.heading + delta)
        self.state = TURN
        print(f"[CELL {self.cells}] {label}")

    def stop_done(self, client, reason):
        self.state = DONE
        self.drive(client, 0.0, 0.0)
        print(f"[EXIT] {reason} at t={self.time:.2f}s after {self.cells} cells -> stopped")

    def decide(self, client, side_l, side_r, front):
        g = self.geo
        self.cells += 1

        quadrant = int(round(self.heading / (math.pi / 2))) % 4
        self.grid_x += {0: 1, 2: -1}.get(quadrant, 0)

        if self.escape >= ESCAPE_AT_DECISION:
            self.stop_done(client, "Open space on both sides")
            return

        if MAZE_COLS is not None and self.grid_x >= MAZE_COLS:
            self.stop_done(client, "Passed last column")
            return

        left_open = side_l > g.side_open
        right_open = side_r > g.side_open
        front_open = front > g.front_blocked

        if FOLLOW_HAND == "LEFT":
            order = [(left_open, math.pi / 2, "LEFT"),
                     (front_open, 0.0, "STRAIGHT"),
                     (right_open, -math.pi / 2, "RIGHT")]
        else:
            order = [(right_open, -math.pi / 2, "RIGHT"),
                     (front_open, 0.0, "STRAIGHT"),
                     (left_open, math.pi / 2, "LEFT")]

        for is_open, delta, label in order:
            if is_open:
                if delta == 0.0:
                    self.rem += g.cell
                    return
                self.start_turn(delta, label)
                return

        self.start_turn(math.pi, "DEAD END -> 180")

    def process_sensor_payload(self, client, msg):
        try:
            d = json.loads(msg.payload.decode())
            raw_l = float(d["fl"])
            raw_r = float(d["fr"])
            front_l = self.clean(float(d["sl"]))
            front_r = self.clean(float(d["sr"]))
            yaw_rate = float(d["gyro"][2])
            dt = float(d["dt"])
        except (KeyError, TypeError, ValueError, IndexError, json.JSONDecodeError):
            return

        if math.isnan(yaw_rate):
            yaw_rate = 0.0
        if not (0.0 < dt <= 0.05):
            dt = 0.002

        self.count += 1
        self.time += dt
        self.yaw = wrap(self.yaw + GYRO_SIGN * yaw_rate * dt)

        side_l = self.clean(raw_l)
        side_r = self.clean(raw_r)
        front = min(front_l, front_r)
        err_h = wrap(self.heading - self.yaw)
        g = self.geo

        if self.state == DONE:
            self.drive(client, 0.0, 0.0)
            return

        if self.state == CALIBRATE:
            if side_l < CALIB_MAX_SIDE and side_r < CALIB_MAX_SIDE:
                self.calib_sum += side_l + side_r
                self.calib_n += 1
            self.calib_dist += (self.last_l + self.last_r) / 2.0 * WHEEL_RADIUS * dt
            omega = self.clamp(K_HEADING * err_h, -MAX_OMEGA, MAX_OMEGA)
            self.drive_vw(client, CALIB_SPEED, omega)
            if self.time >= CALIB_TIME:
                self.finish_calibration()
            return

        if self.state == TURN:
            if abs(err_h) < TURN_TOL:
                self.state = SETTLE
                self.settle = TURN_SETTLE
                self.drive(client, 0.0, 0.0)
                return
            yaw_rate_cmd = math.sqrt(2.0 * TURN_ALPHA * abs(err_h))
            w = yaw_rate_cmd * TRACK_WIDTH / 2.0 / WHEEL_RADIUS
            w = self.clamp(w, TURN_MIN, TURN_MAX)
            w = w if err_h > 0 else -w
            self.drive(client, -w, w)
            return

        if self.state == SETTLE:
            self.settle -= dt
            omega = self.clamp(K_HEADING * err_h, -MAX_OMEGA, MAX_OMEGA)
            self.drive_vw(client, 0.0, omega)
            if self.settle <= 0.0:
                self.state = FORWARD
                self.rem = g.cell
                self.prev_left_wall = None
                self.prev_right_wall = None
                self.escape = 0.0
            return

        v = (self.last_l + self.last_r) / 2.0 * WHEEL_RADIUS
        step = v * dt
        self.rem -= step

        if self.escaped(raw_l) and self.escaped(raw_r):
            self.escape += step
        else:
            self.escape = max(0.0, self.escape - step)

        if self.escape >= ESCAPE_CELLS * g.cell:
            self.stop_done(client, "Open space on both sides")
            return

        left_wall = side_l < g.side_wall
        right_wall = side_r < g.side_wall
        aligned = abs(err_h) < CORRECTION_MAX_YAW_ERR

        for prev, now in ((self.prev_left_wall, left_wall),
                          (self.prev_right_wall, right_wall)):
            if prev is None or prev == now or not aligned:
                continue
            target = g.gap_to_center if not now else g.wall_to_center
            if abs(target - self.rem) < g.gate:
                self.rem = target

        self.prev_left_wall = left_wall
        self.prev_right_wall = right_wall

        if front < g.front_trust and aligned:
            est = front - g.front_at_center
            if abs(est - self.rem) < g.gate:
                self.rem = est

        if self.rem <= 0.0:
            self.decide(client, side_l, side_r, front)
            if self.state in (TURN, DONE):
                if self.state == TURN:
                    self.drive(client, 0.0, 0.0)
                return

        if left_wall and right_wall:
            err_c = (side_l - side_r) / 2.0
        elif left_wall:
            err_c = side_l - g.side_centered
        elif right_wall:
            err_c = g.side_centered - side_r
        else:
            err_c = 0.0

        offset = self.clamp(K_CENTER * err_c, -MAX_HEADING_OFFSET, MAX_HEADING_OFFSET)
        omega = self.clamp(K_HEADING * wrap(err_h + offset), -MAX_OMEGA, MAX_OMEGA)

        speed = FWD_SPEED
        turn_coming = front < g.front_at_center + g.cell or (
            FOLLOW_HAND == "LEFT" and not left_wall
        ) or (FOLLOW_HAND == "RIGHT" and not right_wall)
        if turn_coming:
            v_brake = math.sqrt(2.0 * BRAKE_ACCEL * max(0.0, self.rem)) / WHEEL_RADIUS
            speed = self.clamp(v_brake, APPROACH_MIN, FWD_SPEED)

        self.drive_vw(client, speed, omega)

        if self.count % 500 == 0:
            print(
                f"[MOVE] t={self.time:.1f} cell={self.cells} rem={self.rem:.3f} "
                f"side=({side_l:.3f},{side_r:.3f}) front={front:.3f} "
                f"yaw={math.degrees(self.yaw):.1f} tgt={math.degrees(self.heading):.0f}"
            )


def main():
    bot = PacBotController()

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        client = mqtt.Client()

    def on_connect(cl, userdata, *args):
        cl.subscribe(TOPIC_SENSORS)
        print("[MQTT] connected")

    client.on_connect = on_connect
    client.on_message = lambda cl, user, msg: bot.process_sensor_payload(cl, msg)
    client.connect(MQTT_HOST, MQTT_PORT)

    print("Task 1B Controller Started.")
    print(f"Strategy: {FOLLOW_HAND}-hand wall follower, auto-measured grid, gyro heading hold")

    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[PACBOT] Process Terminated.")