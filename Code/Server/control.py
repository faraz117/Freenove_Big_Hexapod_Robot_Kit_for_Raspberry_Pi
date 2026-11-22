# -*- coding: utf-8 -*-
import io
import time
import math
import copy
import threading
import numpy as np
from PIL import Image
from gpiozero import OutputDevice

from pid import Incremental_PID
from command import COMMAND as cmd
from imu import IMU
from servo import Servo, NullServo, SupervisedServo

class Control:
    def __init__(self, ultrasonic=None, led=None, buzzer=None, camera=None):
        self.hardware_ok = True
        self.hardware_error = None
        try:
            self.imu = IMU()
            self.servo = SupervisedServo(Servo(), self._handle_servo_failure)
        except Exception as e:
            # Most likely the PCA9685/IMU boards aren't responding on I2C (e.g. the
            # battery powering them is disconnected - the Pi's own supply doesn't reach
            # that board). Fall back to a no-op servo so everything else (GUI, camera,
            # sensing) keeps working; reinitialize_hardware() retries this later.
            print("Control: hardware init failed, I2C not responsive:", e)
            self.imu = None
            self.servo = NullServo()
            self.hardware_ok = False
            self.hardware_error = str(e)
        self.movement_flag = 0x01
        self.relaxation_flag = False
        self.pid_controller = Incremental_PID(0.500, 0.00, 0.0025)
        self.servo_power_disable = OutputDevice(4)
        self.servo_power_disable.off()
        self.status_flag = 0x00
        self.timeout = 0
        self.body_height = -25
        self.body_points = [[137.1, 189.4, self.body_height], [225, 0, self.body_height], [137.1, -189.4, self.body_height], 
                           [-137.1, -189.4, self.body_height], [-225, 0, self.body_height], [-137.1, 189.4, self.body_height]]
        self.calibration_leg_positions = self.read_from_txt('point')
        self.leg_positions = [[140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0]]
        self.calibration_angles = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.current_angles = [[90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0]]
        self.command_queue = ['', '', '', '', '', '']
        self.calibrate()
        self.set_leg_angles()
        self.condition_thread = threading.Thread(target=self.condition_monitor)
        self.Thread_conditiona = threading.Condition()
        # Autonomous obstacle-avoidance patrol, active until a client connects
        self.ultrasonic = ultrasonic
        self.led = led
        self.buzzer = buzzer
        self.camera = camera
        self.camera_baseline = {}  # (pan, tilt) -> slow-moving "normal" brightness at that angle
        self.last_camera_frame = None  # used to detect a frozen feed (identical bytes repeatedly)
        self.last_camera_frame_change_time = time.time()
        self.autonomous_mode = True
        self.paused = False  # GUI pause/resume button - holds position without stopping sensing
        self.obstacle_distance = 30  # cm; anything closer counts as an obstacle
        self.last_distance = None
        self.last_point_obstacle = False  # did the most recently sampled grid point trip either sensor
        self.consecutive_evasions = 0  # rounds stuck evading in a row, without a clean forward step between
        self.distance_map = {}  # (pan, tilt) -> most recent distance, refreshed by the continuous sweep
        self.detection_grid = {}  # (pan, tilt) -> '0'/'U'/'C'/'B', refreshed by the continuous sweep
        self.last_obstacle = None  # {'source', 'pan', 'tilt', 'distance'} describing what last tripped a threshold
        self.sweeping = False
        self.head_pan_channel = 0  # horizontal pan servo (confirmed on hardware)
        self.head_tilt_channel = 1  # vertical tilt servo (confirmed on hardware)
        self.pan_points = [30, 90, 150]
        self.tilt_points = [70, 90, 130]  # narrower throw than pan - less travel per step at the same settle_time
        for pan_angle in self.pan_points:
            for tilt_angle in self.tilt_points:
                self.detection_grid[(pan_angle, tilt_angle)] = '0'
        self.patrol_thread = threading.Thread(target=self.autonomous_patrol)
        self.sensor_thread = threading.Thread(target=self.watch_forward)

    def reinitialize_hardware(self):
        # Retries bringing up the IMU/servo boards after an earlier I2C failure (e.g. once
        # the battery powering them is reconnected). Safe to call repeatedly - a no-op if
        # already working, and leaves the NullServo stub in place if it still fails.
        if self.hardware_ok:
            return True
        try:
            imu = IMU()
            servo = Servo()
        except Exception as e:
            print("Control: I2C recheck failed, still not responsive:", e)
            self.hardware_error = str(e)
            return False
        self.imu = imu
        self.servo = SupervisedServo(servo, self._handle_servo_failure)
        self.hardware_ok = True
        self.hardware_error = None
        self.calibrate()
        self.set_leg_angles()
        print("Control: I2C recovered, hardware reinitialized")
        return True

    def _handle_servo_failure(self, error):
        # Called by SupervisedServo when a real servo call raises mid-operation (e.g. the
        # battery sags/disconnects while already running) - demote to the no-op stub instead
        # of letting the exception kill whichever thread was calling it.
        if self.hardware_ok:
            print("Control: hardware I/O error during operation, I2C no longer responsive:", error)
        self.hardware_ok = False
        self.hardware_error = str(error)
        self.servo = NullServo()

    def read_from_txt(self, filename):
        with open(filename + ".txt", "r") as file:
            lines = file.readlines()
            data = [list(map(int, line.strip().split("\t"))) for line in lines]
        return data

    def save_to_txt(self, data, filename):
        with open(filename + '.txt', 'w') as file:
            for row in data:
                file.write('\t'.join(map(str, row)) + '\n')

    def coordinate_to_angle(self, x, y, z, l1=33, l2=90, l3=110):
        a = math.pi / 2 - math.atan2(z, y)
        x_3 = 0
        x_4 = l1 * math.sin(a)
        x_5 = l1 * math.cos(a)
        l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + (x - x_3) ** 2)
        w = self.restrict_value((x - x_3) / l23, -1, 1)
        v = self.restrict_value((l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23), -1, 1)
        u = self.restrict_value((l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2), -1, 1)
        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        c = math.pi - math.acos(round(u, 2))
        return round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c))

    def angle_to_coordinate(self, a, b, c, l1=33, l2=90, l3=110):
        a = math.pi / 180 * a
        b = math.pi / 180 * b
        c = math.pi / 180 * c
        x = round(l3 * math.sin(b + c) + l2 * math.sin(b))
        y = round(l3 * math.sin(a) * math.cos(b + c) + l2 * math.sin(a) * math.cos(b) + l1 * math.sin(a))
        z = round(l3 * math.cos(a) * math.cos(b + c) + l2 * math.cos(a) * math.cos(b) + l1 * math.cos(a))
        return x, y, z

    def calibrate(self):
        self.leg_positions = [[140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0]]
        for i in range(6):
            self.calibration_angles[i][0], self.calibration_angles[i][1], self.calibration_angles[i][2] = self.coordinate_to_angle(
                -self.calibration_leg_positions[i][2], self.calibration_leg_positions[i][0], self.calibration_leg_positions[i][1])
        for i in range(6):
            self.current_angles[i][0], self.current_angles[i][1], self.current_angles[i][2] = self.coordinate_to_angle(
                -self.leg_positions[i][2], self.leg_positions[i][0], self.leg_positions[i][1])
        for i in range(6):
            self.calibration_angles[i][0] = self.calibration_angles[i][0] - self.current_angles[i][0]
            self.calibration_angles[i][1] = self.calibration_angles[i][1] - self.current_angles[i][1]
            self.calibration_angles[i][2] = self.calibration_angles[i][2] - self.current_angles[i][2]

    def set_leg_angles(self):
        if self.check_point_validity():
            for i in range(6):
                self.current_angles[i][0], self.current_angles[i][1], self.current_angles[i][2] = self.coordinate_to_angle(
                    -self.leg_positions[i][2], self.leg_positions[i][0], self.leg_positions[i][1])
            for i in range(3):
                self.current_angles[i][0] = self.restrict_value(self.current_angles[i][0] + self.calibration_angles[i][0], 0, 180)
                self.current_angles[i][1] = self.restrict_value(90 - (self.current_angles[i][1] + self.calibration_angles[i][1]), 0, 180)
                self.current_angles[i][2] = self.restrict_value(self.current_angles[i][2] + self.calibration_angles[i][2], 0, 180)
                self.current_angles[i + 3][0] = self.restrict_value(self.current_angles[i + 3][0] + self.calibration_angles[i + 3][0], 0, 180)
                self.current_angles[i + 3][1] = self.restrict_value(90 + self.current_angles[i + 3][1] + self.calibration_angles[i + 3][1], 0, 180)
                self.current_angles[i + 3][2] = self.restrict_value(180 - (self.current_angles[i + 3][2] + self.calibration_angles[i + 3][2]), 0, 180)
            # Leg 1
            self.servo.set_servo_angle(15, self.current_angles[0][0])
            self.servo.set_servo_angle(14, self.current_angles[0][1])
            self.servo.set_servo_angle(13, self.current_angles[0][2])
            # Leg 2
            self.servo.set_servo_angle(12, self.current_angles[1][0])
            self.servo.set_servo_angle(11, self.current_angles[1][1])
            self.servo.set_servo_angle(10, self.current_angles[1][2])
            # Leg 3
            self.servo.set_servo_angle(9, self.current_angles[2][0])
            self.servo.set_servo_angle(8, self.current_angles[2][1])
            self.servo.set_servo_angle(31, self.current_angles[2][2])
            # Leg 6
            self.servo.set_servo_angle(16, self.current_angles[5][0])
            self.servo.set_servo_angle(17, self.current_angles[5][1])
            self.servo.set_servo_angle(18, self.current_angles[5][2])
            # Leg 5
            self.servo.set_servo_angle(19, self.current_angles[4][0])
            self.servo.set_servo_angle(20, self.current_angles[4][1])
            self.servo.set_servo_angle(21, self.current_angles[4][2])
            # Leg 4
            self.servo.set_servo_angle(22, self.current_angles[3][0])
            self.servo.set_servo_angle(23, self.current_angles[3][1])
            self.servo.set_servo_angle(27, self.current_angles[3][2])
        else:
            print("This coordinate point is out of the active range")

    def check_point_validity(self):
        is_valid = True
        leg_lengths = [0] * 6
        for i in range(6):
            leg_lengths[i] = math.sqrt(self.leg_positions[i][0] ** 2 + self.leg_positions[i][1] ** 2 + self.leg_positions[i][2] ** 2)
        for length in leg_lengths:
            if length > 248 or length < 90:
                is_valid = False
        return is_valid

    def condition_monitor(self):
        while True:
            if (time.time() - self.timeout) > 10 and self.timeout != 0 and self.command_queue[0] == '':
                self.timeout = time.time()
                self.relax(True)
                self.status_flag = 0x00
            if cmd.CMD_POSITION in self.command_queue and len(self.command_queue) == 4:
                if self.status_flag != 0x01:
                    self.relax(False)
                x = self.restrict_value(int(self.command_queue[1]), -40, 40)
                y = self.restrict_value(int(self.command_queue[2]), -40, 40)
                z = self.restrict_value(int(self.command_queue[3]), -20, 20)
                self.move_position(x, y, z)
                self.status_flag = 0x01
                self.command_queue = ['', '', '', '', '', '']
            elif cmd.CMD_ATTITUDE in self.command_queue and len(self.command_queue) == 4:
                if self.status_flag != 0x02:
                    self.relax(False)
                roll = self.restrict_value(int(self.command_queue[1]), -15, 15)
                pitch = self.restrict_value(int(self.command_queue[2]), -15, 15)
                yaw = self.restrict_value(int(self.command_queue[3]), -15, 15)
                points = self.calculate_posture_balance(roll, pitch, yaw)
                self.transform_coordinates(points)
                self.set_leg_angles()
                self.status_flag = 0x02
                self.command_queue = ['', '', '', '', '', '']
            elif cmd.CMD_MOVE in self.command_queue and len(self.command_queue) == 6:
                if self.command_queue[2] == "0" and self.command_queue[3] == "0":
                    self.run_gait(self.command_queue)
                    self.command_queue = ['', '', '', '', '', '']
                else:
                    if self.status_flag != 0x03:
                        self.relax(False)
                    self.run_gait(self.command_queue)
                    self.status_flag = 0x03
            elif cmd.CMD_BALANCE in self.command_queue and len(self.command_queue) == 2:
                if self.command_queue[1] == "1":
                    self.command_queue = ['', '', '', '', '', '']
                    if self.status_flag != 0x04:
                        self.relax(False)
                    self.status_flag = 0x04
                    self.imu6050()
            elif cmd.CMD_CALIBRATION in self.command_queue:
                self.timeout = 0
                self.calibrate()
                self.set_leg_angles()
                if len(self.command_queue) >= 2:
                    if self.command_queue[1] == "one":
                        self.calibration_leg_positions[0][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[0][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[0][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "two":
                        self.calibration_leg_positions[1][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[1][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[1][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "three":
                        self.calibration_leg_positions[2][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[2][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[2][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "four":
                        self.calibration_leg_positions[3][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[3][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[3][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "five":
                        self.calibration_leg_positions[4][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[4][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[4][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "six":
                        self.calibration_leg_positions[5][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[5][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[5][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "save":
                        self.save_to_txt(self.calibration_leg_positions, 'point')
                self.command_queue = ['', '', '', '', '', '']

    def set_led_color(self, r, g, b):
        try:
            self.led.led_index(0x7f, r, g, b)
        except Exception as e:
            print("autonomous_patrol: LED update failed:", e)

    def set_buzzer(self, state):
        try:
            self.buzzer.set_state(state)
        except Exception as e:
            print("autonomous_patrol: Buzzer update failed:", e)

    def record_point(self, pan_angle, tilt_angle, distance):
        # Shared by watch_forward and scan_surroundings: combines both sensors' readings for
        # one grid point into a single status ('0'/'U'/'C'/'B') and records it.
        ultrasonic_hit = distance is not None and distance < self.obstacle_distance
        camera_hit = self.check_camera_obstacle(pan_angle, tilt_angle)
        if ultrasonic_hit and camera_hit:
            status, source = 'B', 'both'
        elif camera_hit:
            status, source = 'C', 'camera'
        elif ultrasonic_hit:
            status, source = 'U', 'ultrasonic'
        else:
            status, source = '0', None
        self.detection_grid[(pan_angle, tilt_angle)] = status
        if source is not None:
            self.last_obstacle = {'source': source, 'pan': pan_angle, 'tilt': tilt_angle, 'distance': distance}
        return status

    def watch_forward(self, interval=0.15):
        # Holds the head centered and watches straight ahead while walking, instead of
        # continuously sweeping - cheaper on the servos, and since the head never moves there's
        # no movement-induced stale echo to wait out, so this can poll much faster than the
        # sweep could. Only scan_surroundings() (triggered once something gets close) looks
        # around to find a way out; this just watches the one point directly ahead.
        center_pan, center_tilt = 90, 90
        self.servo.set_servo_angle(self.head_pan_channel, center_pan)
        self.servo.set_servo_angle(self.head_tilt_channel, center_tilt)
        while True:
            if not self.autonomous_mode or self.ultrasonic is None or self.sweeping:
                time.sleep(interval)
                continue
            distance = self.ultrasonic.get_distance()
            self.last_distance = distance
            self.distance_map[(center_pan, center_tilt)] = distance
            status = self.record_point(center_pan, center_tilt, distance)
            self.last_point_obstacle = status != '0'
            print("watch: pan =", center_pan, "tilt =", center_tilt, "distance =", distance, "status =", status)
            time.sleep(interval)

    def scan_surroundings(self, settle_time=0.6):
        # Sweeps the 3x3 pan/tilt grid, sampling both sensors at each of the 9 points to find
        # a way out. Only called when watch_forward sees something blocking the way ahead -
        # the head otherwise stays centered and still while walking.
        self.sweeping = True
        scan = {}
        try:
            for pan_angle in self.pan_points:
                if not self.autonomous_mode:
                    break
                self.servo.set_servo_angle(self.head_pan_channel, pan_angle)
                for tilt_angle in self.tilt_points:
                    if not self.autonomous_mode:
                        break
                    self.servo.set_servo_angle(self.head_tilt_channel, tilt_angle)
                    time.sleep(settle_time)
                    distance = self.ultrasonic.get_distance() if self.ultrasonic is not None else None
                    scan[(pan_angle, tilt_angle)] = distance
                    self.record_point(pan_angle, tilt_angle, distance)
            self.servo.set_servo_angle(self.head_pan_channel, 90)  # recenter
            self.servo.set_servo_angle(self.head_tilt_channel, 90)
            time.sleep(settle_time)
        finally:
            self.sweeping = False
        print("autonomous_patrol: scan =", scan)
        return scan

    def choose_evasion_direction(self, scan):
        # pan < 90 is one side, > 90 the other (tilt is ignored for left/right); which is
        # physically left vs right on the robot depends on how the sensor/servo is mounted -
        # swap the two branches below if it turns out to be picking the side with the wall.
        left_readings = [d for (pan, tilt), d in scan.items() if pan < 90 and d is not None]
        right_readings = [d for (pan, tilt), d in scan.items() if pan > 90 and d is not None]
        left_clearance = sum(left_readings) / len(left_readings) if left_readings else 0
        right_clearance = sum(right_readings) / len(right_readings) if right_readings else 0
        if max(left_clearance, right_clearance) < self.obstacle_distance:
            return 'back'  # both sides still blocked even at their clearest angle
        return 'left' if left_clearance >= right_clearance else 'right'

    def check_camera_obstacle(self, pan_angle, tilt_angle, deviation_threshold=35, ema_alpha=0.05, stale_after=4.0):
        # Crude vision signal to cover the ultrasonic sensor's blind spots: tracks a slow-moving
        # baseline of how bright the view normally looks AT THIS SPECIFIC angle, and flags a
        # likely obstacle when the current frame's bottom-center crop deviates sharply from it
        # (e.g. something suddenly fills that region, blocking or shadowing the floor). The
        # baseline has to be per-angle, not global - the camera is on the same pan/tilt servos
        # as the ultrasonic sensor, so "looking up" vs "looking at the floor" are naturally very
        # different brightnesses and would otherwise constantly look like a false obstacle.
        # This is a cheap heuristic, not real object detection - it can be fooled by lighting
        # changes, which is why the baseline only drifts to match the scene while nothing is flagged.
        if self.camera is None:
            return False
        try:
            if not self.camera.streaming:
                self.camera.start_stream()
            frame = self.camera.get_frame()
            if frame is None:
                return False
            # picamera2 can wedge (see the "Camera frontend has timed out!" driver error) and
            # keep handing back the exact same bytes forever. If we trusted that, a frame that
            # happened to freeze looking different from the baseline would flag an obstacle
            # permanently - the baseline only adapts while nothing is flagged, so it could never
            # self-correct. Once a frame has been identical for too long, stop trusting it.
            if frame == self.last_camera_frame:
                if time.time() - self.last_camera_frame_change_time > stale_after:
                    return False
            else:
                self.last_camera_frame = frame
                self.last_camera_frame_change_time = time.time()
            image = Image.open(io.BytesIO(frame)).convert('L')
            width, height = image.size
            crop = image.crop((width // 4, height * 2 // 3, width * 3 // 4, height))
            brightness = float(np.asarray(crop, dtype=np.float32).mean())
        except Exception as e:
            print("check_camera_obstacle:", e)
            return False
        key = (pan_angle, tilt_angle)
        baseline = self.camera_baseline.get(key)
        if baseline is None:
            self.camera_baseline[key] = brightness
            return False
        deviation = abs(brightness - baseline)
        obstacle = deviation > deviation_threshold
        if not obstacle:
            self.camera_baseline[key] = ema_alpha * brightness + (1 - ema_alpha) * baseline
        return obstacle

    def autonomous_patrol(self, stance_height=20, lift_height=70, evade_seconds=1.0,
                          forward_move=['CMD_MOVE', '1', '0', '25', '10', '0']):
        # Walk forward until the ultrasonic sensor sees an obstacle. On detection, flash the LED red,
        # and turn/back away for about evade_seconds before resuming forward.
        # Runs until a client connects (autonomous_mode becomes False).
        # Speed 10 (max) also shortens each stride's gait cycle, so the patrol loop re-checks
        # the sensor sooner too - faster walking and faster reaction come from the same change.
        # stance_height is capped at 20: that's the documented max for CMD_POSITION's z parameter
        # (the same parameter move_position uses below) - going past it risks the IK silently
        # clamping a leg angle to 0/180 for an uneven stance instead of an actually taller one.
        evasive_moves = {
            'left': ['CMD_MOVE', '1', '0', '0', '10', '-10'],
            'right': ['CMD_MOVE', '1', '0', '0', '10', '10'],
            'back': ['CMD_MOVE', '1', '0', '-25', '10', '0'],
        }
        self.move_position(0, 0, stance_height)  # stand taller for proper leg clearance while walking
        print("autonomous_patrol: standing joint angles =", self.current_angles)  # 0 or 180 here means a leg is clamped/maxed out
        if self.led is not None:
            self.set_led_color(0, 0, 0)
        while True:
            if not self.autonomous_mode:
                time.sleep(0.1)
                continue
            if self.paused:
                # Sensing (watch_forward) keeps running so the GUI stays live - just stop
                # issuing movement commands, holding wherever the legs currently are.
                self.consecutive_evasions = 0
                time.sleep(0.1)
                continue
            # continuous_scan already checks both sensors at whichever point it's currently
            # sampling and combines them into last_point_obstacle - no need to check again here.
            obstacle_detected = self.last_point_obstacle
            if obstacle_detected:
                if self.led is not None:
                    self.set_led_color(255, 0, 0)
                self.consecutive_evasions += 1
                if self.consecutive_evasions >= 3:
                    # Stuck in a corner: repeated small left/right nudges aren't escaping it,
                    # so force a longer, more deliberate back-away instead of repeating the
                    # same move that already failed twice.
                    direction = 'back'
                    duration = evade_seconds * 2
                    print("autonomous_patrol: stuck for", self.consecutive_evasions,
                          "rounds, forcing a longer back-away")
                else:
                    scan = self.scan_surroundings()
                    direction = self.choose_evasion_direction(scan)
                    duration = evade_seconds
                move = evasive_moves[direction]
                print("autonomous_patrol: obstacle detected, evading", direction)
                evade_until = time.time() + duration
                while time.time() < evade_until:
                    if not self.autonomous_mode:
                        break
                    self.run_gait(move, Z=lift_height)
                if self.led is not None:
                    self.set_led_color(0, 0, 0)
            else:
                self.consecutive_evasions = 0
                self.run_gait(forward_move, Z=lift_height)

    def relax(self, flag):
        if flag:
            self.servo.relax()
        else:
            self.set_leg_angles()

    def transform_coordinates(self, points):
        # Leg 1
        self.leg_positions[0][0] = points[0][0] * math.cos(54 / 180 * math.pi) + points[0][1] * math.sin(54 / 180 * math.pi) - 94
        self.leg_positions[0][1] = -points[0][0] * math.sin(54 / 180 * math.pi) + points[0][1] * math.cos(54 / 180 * math.pi)
        self.leg_positions[0][2] = points[0][2] - 14
        # Leg 2
        self.leg_positions[1][0] = points[1][0] * math.cos(0 / 180 * math.pi) + points[1][1] * math.sin(0 / 180 * math.pi) - 85
        self.leg_positions[1][1] = -points[1][0] * math.sin(0 / 180 * math.pi) + points[1][1] * math.cos(0 / 180 * math.pi)
        self.leg_positions[1][2] = points[1][2] - 14
        # Leg 3
        self.leg_positions[2][0] = points[2][0] * math.cos(-54 / 180 * math.pi) + points[2][1] * math.sin(-54 / 180 * math.pi) - 94
        self.leg_positions[2][1] = -points[2][0] * math.sin(-54 / 180 * math.pi) + points[2][1] * math.cos(-54 / 180 * math.pi)
        self.leg_positions[2][2] = points[2][2] - 14
        # Leg 4
        self.leg_positions[3][0] = points[3][0] * math.cos(-126 / 180 * math.pi) + points[3][1] * math.sin(-126 / 180 * math.pi) - 94
        self.leg_positions[3][1] = -points[3][0] * math.sin(-126 / 180 * math.pi) + points[3][1] * math.cos(-126 / 180 * math.pi)
        self.leg_positions[3][2] = points[3][2] - 14
        # Leg 5
        self.leg_positions[4][0] = points[4][0] * math.cos(180 / 180 * math.pi) + points[4][1] * math.sin(180 / 180 * math.pi) - 85
        self.leg_positions[4][1] = -points[4][0] * math.sin(180 / 180 * math.pi) + points[4][1] * math.cos(180 / 180 * math.pi)
        self.leg_positions[4][2] = points[4][2] - 14
        # Leg 6
        self.leg_positions[5][0] = points[5][0] * math.cos(126 / 180 * math.pi) + points[5][1] * math.sin(126 / 180 * math.pi) - 94
        self.leg_positions[5][1] = -points[5][0] * math.sin(126 / 180 * math.pi) + points[5][1] * math.cos(126 / 180 * math.pi)
        self.leg_positions[5][2] = points[5][2] - 14

    def restrict_value(self, value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value

    def map_value(self, value, from_low, from_high, to_low, to_high):
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

    def move_position(self, x, y, z):
        points = copy.deepcopy(self.body_points)
        for i in range(6):
            points[i][0] = self.body_points[i][0] - x
            points[i][1] = self.body_points[i][1] - y
            points[i][2] = -30 - z
            self.body_height = points[i][2]
            self.body_points[i][2] = points[i][2]
        self.transform_coordinates(points)
        self.set_leg_angles()

    def calculate_posture_balance(self, roll, pitch, yaw):
        position = np.asmatrix([0.0, 0.0, self.body_height]).T
        rpy = np.array([roll, pitch, yaw]) * math.pi / 180
        roll_angle, pitch_angle, yaw_angle = rpy[0], rpy[1], rpy[2]
        rotation_x = np.asmatrix([[1, 0, 0],
                             [0, math.cos(pitch_angle), -math.sin(pitch_angle)],
                             [0, math.sin(pitch_angle), math.cos(pitch_angle)]])
        rotation_y = np.asmatrix([[math.cos(roll_angle), 0, -math.sin(roll_angle)],
                             [0, 1, 0],
                             [math.sin(roll_angle), 0, math.cos(roll_angle)]])
        rotation_z = np.asmatrix([[math.cos(yaw_angle), -math.sin(yaw_angle), 0],
                             [math.sin(yaw_angle), math.cos(yaw_angle), 0],
                             [0, 0, 1]])
        rotation_matrix = rotation_x * rotation_y * rotation_z
        body_structure = np.asmatrix([[55, 76, 0],
                                [85, 0, 0],
                                [55, -76, 0],
                                [-55, -76, 0],
                                [-85, 0, 0],
                                [-55, 76, 0]]).T
        footpoint_structure = np.asmatrix([[137.1, 189.4, 0],
                                     [225, 0, 0],
                                     [137.1, -189.4, 0],
                                     [-137.1, -189.4, 0],
                                     [-225, 0, 0],
                                     [-137.1, 189.4, 0]]).T
        ab = np.asmatrix(np.zeros((3, 6)))
        foot_positions = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for i in range(6):
            ab[:, i] = position + rotation_matrix * footpoint_structure[:, i]
            foot_positions[i][0] = ab[0, i]
            foot_positions[i][1] = ab[1, i]
            foot_positions[i][2] = ab[2, i]
        return foot_positions

    def imu6050(self):
        if self.imu is None:
            print("imu6050: IMU not available (I2C not responsive)")
            return
        old_roll = 0
        old_pitch = 0
        points = self.calculate_posture_balance(0, 0, 0)
        self.transform_coordinates(points)
        self.set_leg_angles()
        time.sleep(2)
        self.imu.Error_value_accel_data, self.imu.Error_value_gyro_data = self.imu.calculate_average_sensor_data()
        time.sleep(1)
        while True:
            if self.command_queue[0] != "":
                break
            time.sleep(0.02)
            roll, pitch, yaw = self.imu.update_imu_state()
            roll = self.pid_controller.pid_calculate(roll)
            pitch = self.pid_controller.pid_calculate(pitch)
            points = self.calculate_posture_balance(roll, pitch, 0)
            self.transform_coordinates(points)
            self.set_leg_angles()

    def run_gait(self, data, Z=40, F=64):  # Example: data=['CMD_MOVE', '1', '0', '25', '10', '0']
        gait = data[1]
        x = self.restrict_value(int(data[2]), -35, 35)
        y = self.restrict_value(int(data[3]), -35, 35)
        if gait == "1":
            F = round(self.map_value(int(data[4]), 2, 10, 126, 22))
        else:
            F = round(self.map_value(int(data[4]), 2, 10, 171, 45))
        angle = int(data[5])
        z = Z / F
        delay = 0.01
        points = copy.deepcopy(self.body_points)
        xy = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
        for i in range(6):
            xy[i][0] = ((points[i][0] * math.cos(angle / 180 * math.pi) + points[i][1] * math.sin(angle / 180 * math.pi) - points[i][0]) + x) / F
            xy[i][1] = ((-points[i][0] * math.sin(angle / 180 * math.pi) + points[i][1] * math.cos(angle / 180 * math.pi) - points[i][1]) + y) / F
        if x == 0 and y == 0 and angle == 0:
            self.transform_coordinates(points)
            self.set_leg_angles()
        elif gait == "1":
            for j in range(F):
                for i in range(3):
                    if j < (F / 8):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][0] = points[2 * i + 1][0] + 8 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] + 8 * xy[2 * i + 1][1]
                        points[2 * i + 1][2] = Z + self.body_height
                    elif j < (F / 4):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][2] = points[2 * i + 1][2] - z * 8
                    elif j < (3 * F / 8):
                        points[2 * i][2] = points[2 * i][2] + z * 8
                        points[2 * i + 1][0] = points[2 * i + 1][0] - 4 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] - 4 * xy[2 * i + 1][1]
                    elif j < (5 * F / 8):
                        points[2 * i][0] = points[2 * i][0] + 8 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] + 8 * xy[2 * i][1]
                        points[2 * i + 1][0] = points[2 * i + 1][0] - 4 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] - 4 * xy[2 * i + 1][1]
                    elif j < (3 * F / 4):
                        points[2 * i][2] = points[2 * i][2] - z * 8
                        points[2 * i + 1][0] = points[2 * i + 1][0] - 4 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] - 4 * xy[2 * i + 1][1]
                    elif j < (7 * F / 8):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][2] = points[2 * i + 1][2] + z * 8
                    elif j < (F):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][0] = points[2 * i + 1][0] + 8 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] + 8 * xy[2 * i + 1][1]
                self.transform_coordinates(points)
                self.set_leg_angles()
                time.sleep(delay)
        elif gait == "2":
            number = [5, 2, 1, 0, 3, 4]
            for i in range(6):
                for j in range(int(F / 6)):
                    for k in range(6):
                        if number[i] == k:
                            if j < int(F / 18):
                                points[k][2] += 18 * z
                            elif j < int(F / 9):
                                points[k][0] += 30 * xy[k][0]
                                points[k][1] += 30 * xy[k][1]
                            elif j < int(F / 6):
                                points[k][2] -= 18 * z
                        else:
                            points[k][0] -= 2 * xy[k][0]
                            points[k][1] -= 2 * xy[k][1]
                    self.transform_coordinates(points)
                    self.set_leg_angles()
                    time.sleep(delay)

if __name__ == '__main__':
    pass