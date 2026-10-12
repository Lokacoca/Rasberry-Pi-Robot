#!/usr/bin/env python3
"""
Obstacle Avoidance Robot with Wall Following
- Front ultrasonic sensor for obstacle detection
- Rear ultrasonic sensor on a servo for wall tracking
- State machine for navigation
"""

import RPi.GPIO as GPIO
import time
import random
from enum import Enum

# ==================== PIN CONFIGURATION ====================

# Front HC-SR04
FRONT_TRIG = 23
FRONT_ECHO = 24

# Rear HC-SR04
REAR_TRIG = 5
REAR_ECHO = 6

# Servo carrying the rear sensor
SERVO_PIN = 18

# L298N motor driver
MOTOR_A_ENA = 25
MOTOR_A_IN1 = 17
MOTOR_A_IN2 = 27
MOTOR_B_ENB = 26
MOTOR_B_IN3 = 22
MOTOR_B_IN4 = 16

# ==================== SERVO POSITIONS ====================
# Angles map to pulse widths of 1000µs + (angle/180)*1000µs,
# so -90 = 500µs and 270 = 2500µs (outside the standard 0-180 range)
SERVO_CENTER = 90    # Rear sensor pointing backward
SERVO_LEFT = -90     # Rear sensor pointing left
SERVO_RIGHT = 270    # Rear sensor pointing right

# ==================== CONFIGURATION SETTINGS ====================

# Obstacle detection (front sensor)
FRONT_OBSTACLE_DISTANCE = 10   # cm - trigger avoidance
FRONT_VERIFICATION_TIME = 0.8  # seconds - verify obstacle
FRONT_REQUIRED_DETECTIONS = 3  # readings under the threshold needed to confirm

# Wall following (rear sensor)
WALL_FOLLOW_DISTANCE_MIN = 8      # cm - too close to wall
WALL_FOLLOW_DISTANCE_MAX = 20     # cm - too far from wall
WALL_FOLLOW_DISTANCE_TARGET = 10  # cm - ideal wall distance
WALL_LOST_THRESHOLD = 40          # cm - wall is gone
CORNER_CLEAR_TIME = 0.4           # seconds - drive past corner before turning
MIN_WALL_FOLLOW_TIME = 1.0        # seconds - minimum time to follow wall

# Motor speeds (0-100)
FORWARD_SPEED = 50
TURN_SPEED = 60
BACKUP_SPEED = 50
WALL_FOLLOW_SPEED = 45

# Timing
BACKUP_TIME = 0.6
TURN_90_TIME = 0.8   # Seconds of turning for 90 degrees
TURN_180_TIME = 1.6  # Seconds of turning for 180 degrees
STARTUP_DELAY = 10   # Countdown before starting

# ==================== ROBOT STATES ====================
class State(Enum):
    EXPLORING = 1           # Normal forward motion
    VERIFYING_OBSTACLE = 2  # Checking if obstacle is real
    WALL_FOLLOWING = 3      # Following alongside a wall
    BLOCKED_BOTH_SIDES = 4  # Hit wall while wall-following
    CLEARING_CORNER = 5     # Driving past corner before turning
    BACKING_UP = 6          # Reversing from obstacle

# ==================== GLOBAL STATE VARIABLES ====================
current_state = State.EXPLORING
wall_follow_direction = None  # Direction the robot turned to go around the obstacle
wall_follow_start_time = 0
current_heading = 0  # Cumulative rotation, for debugging

# ==================== SETUP ====================
def setup():
    """Initialize GPIO pins and settings"""
    print("=" * 70)
    print("ADVANCED WALL-FOLLOWING ROBOT - STARTING UP")
    print("=" * 70)
    print("\nSetting up GPIO pins...")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Front sensor
    GPIO.setup(FRONT_TRIG, GPIO.OUT)
    GPIO.setup(FRONT_ECHO, GPIO.IN)
    
    # Rear sensor
    GPIO.setup(REAR_TRIG, GPIO.OUT)
    GPIO.setup(REAR_ECHO, GPIO.IN)
    
    # Servo
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    
    # Motor driver
    GPIO.setup(MOTOR_A_ENA, GPIO.OUT)
    GPIO.setup(MOTOR_A_IN1, GPIO.OUT)
    GPIO.setup(MOTOR_A_IN2, GPIO.OUT)
    GPIO.setup(MOTOR_B_ENB, GPIO.OUT)
    GPIO.setup(MOTOR_B_IN3, GPIO.OUT)
    GPIO.setup(MOTOR_B_IN4, GPIO.OUT)
    
    # PWM objects
    global pwm_a, pwm_b, servo_pwm
    pwm_a = GPIO.PWM(MOTOR_A_ENA, 1000)
    pwm_b = GPIO.PWM(MOTOR_B_ENB, 1000)
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz for servo
    
    pwm_a.start(0)
    pwm_b.start(0)
    servo_pwm.start(0)
    
    set_servo_angle(SERVO_CENTER)
    time.sleep(0.5)
    
    print("✓ GPIO setup complete")
    print("✓ Front sensor configured")
    print("✓ Rear sensor configured")
    print("✓ Servo initialized (center position)")
    print("✓ Motor driver ready")
    print()

# ==================== SERVO CONTROL ====================
def set_servo_angle(angle):
    """
    Move the servo to an angle.
    Pulse width in microseconds = 1000 + (angle / 180) * 1000
    (0° = 1000µs, 90° = 1500µs, 180° = 2000µs)
    """
    pulse_width = 1000 + (angle / 180.0) * 1000
    
    # At 50Hz the period is 20ms (20000µs)
    duty_cycle = (pulse_width / 20000.0) * 100
    
    print(f"   [Servo Debug] Setting angle {angle}° (pulse: {pulse_width:.0f}µs, duty: {duty_cycle:.2f}%)")
    
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Give servo time to move
    servo_pwm.ChangeDutyCycle(0)  # Stop the signal to prevent jitter

def point_rear_sensor(direction):
    """
    Point rear sensor in a direction
    direction: 'left', 'right', or 'center'
    """
    if direction == 'left':
        set_servo_angle(SERVO_LEFT)
        print(f"   [Servo] Rear sensor now pointing LEFT")
    elif direction == 'right':
        set_servo_angle(SERVO_RIGHT)
        print(f"   [Servo] Rear sensor now pointing RIGHT")
    elif direction == 'center':
        set_servo_angle(SERVO_CENTER)
        print(f"   [Servo] Rear sensor now pointing CENTER (backward)")
    else:
        print(f"   [Servo] Warning: Unknown direction '{direction}'")

# ==================== SENSOR FUNCTIONS ====================
def measure_distance(trig_pin, echo_pin, sensor_name="sensor"):
    """
    Measure distance using an HC-SR04 sensor
    Returns: distance in cm (or -1 on timeout)
    """
    GPIO.output(trig_pin, False)
    time.sleep(0.01)
    
    # 10µs trigger pulse
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)
    
    # Wait for echo to go high
    pulse_start = time.time()
    timeout = pulse_start + 0.1
    
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return -1
    
    # Wait for echo to go low
    pulse_end = time.time()
    timeout = pulse_end + 0.1
    
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return -1
    
    # Speed of sound is 34300 cm/s; halve it for the round trip
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    
    return distance

def measure_front():
    """Measure distance with front sensor"""
    return measure_distance(FRONT_TRIG, FRONT_ECHO, "front")

def measure_rear():
    """Measure distance with rear sensor"""
    return measure_distance(REAR_TRIG, REAR_ECHO, "rear")

def verify_front_obstacle():
    """
    Verify front obstacle with multiple readings
    Returns: (confirmed, detection_count)
    """
    print(f"\n[Verify] Checking front obstacle for {FRONT_VERIFICATION_TIME}s...")
    
    detection_count = 0
    start_time = time.time()
    readings = []
    
    while (time.time() - start_time) < FRONT_VERIFICATION_TIME:
        dist = measure_front()
        
        if dist > 0:
            readings.append(dist)
            if dist < FRONT_OBSTACLE_DISTANCE:
                detection_count += 1
                print(f"   Reading {len(readings)}: {dist:.1f} cm - OBSTACLE ({detection_count}/{FRONT_REQUIRED_DETECTIONS})")
            else:
                print(f"   Reading {len(readings)}: {dist:.1f} cm - Clear")
        
        time.sleep(0.2)
    
    confirmed = detection_count >= FRONT_REQUIRED_DETECTIONS
    
    if confirmed:
        print(f"[Verify] ✗ OBSTACLE CONFIRMED ({detection_count} detections)")
    else:
        print(f"[Verify] ✓ False alarm ({detection_count} detections)")
    
    return confirmed, detection_count

# ==================== MOTOR CONTROL ====================
def stop_motors():
    """Stop both motors"""
    GPIO.output(MOTOR_A_IN1, False)
    GPIO.output(MOTOR_A_IN2, False)
    GPIO.output(MOTOR_B_IN3, False)
    GPIO.output(MOTOR_B_IN4, False)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)

def move_forward(speed=FORWARD_SPEED):
    """Move robot forward"""
    GPIO.output(MOTOR_A_IN1, False)
    GPIO.output(MOTOR_A_IN2, True)
    GPIO.output(MOTOR_B_IN3, False)
    GPIO.output(MOTOR_B_IN4, True)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def move_backward(speed=BACKUP_SPEED):
    """Move robot backward"""
    GPIO.output(MOTOR_A_IN1, True)
    GPIO.output(MOTOR_A_IN2, False)
    GPIO.output(MOTOR_B_IN3, True)
    GPIO.output(MOTOR_B_IN4, False)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def turn_left(speed=TURN_SPEED):
    """Turn robot left"""
    GPIO.output(MOTOR_A_IN1, False)
    GPIO.output(MOTOR_A_IN2, True)
    GPIO.output(MOTOR_B_IN3, True)
    GPIO.output(MOTOR_B_IN4, False)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def turn_right(speed=TURN_SPEED):
    """Turn robot right"""
    GPIO.output(MOTOR_A_IN1, True)
    GPIO.output(MOTOR_A_IN2, False)
    GPIO.output(MOTOR_B_IN3, False)
    GPIO.output(MOTOR_B_IN4, True)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def execute_turn(direction, degrees):
    """
    Turn robot in the given direction for a timed 90 or 180 degrees
    direction: 'left' or 'right'
    degrees: 90 or 180
    """
    global current_heading
    
    if degrees == 90:
        turn_time = TURN_90_TIME
        heading_change = 90
    elif degrees == 180:
        turn_time = TURN_180_TIME
        heading_change = 180
    else:
        print(f"[Turn] Warning: Unsupported turn angle {degrees}°")
        return
    
    print(f"[Turn] Turning {direction.upper()} {degrees}°...")
    
    if direction == 'left':
        turn_left(TURN_SPEED)
        current_heading -= heading_change
    else:
        turn_right(TURN_SPEED)
        current_heading += heading_change
    
    time.sleep(turn_time)
    stop_motors()
    time.sleep(0.2)
    
    current_heading = current_heading % 360
    
    print(f"[Turn] Complete (heading: {current_heading}°)")

# ==================== STATE MACHINE LOGIC ====================

def handle_exploring_state():
    """
    EXPLORING: drive forward and watch the front sensor
    """
    global current_state
    
    front_dist = measure_front()
    
    if front_dist > 0:
        print(f"[Exploring] Front: {front_dist:6.2f} cm", end="\r")
    
    if 0 < front_dist < FRONT_OBSTACLE_DISTANCE:
        print(f"\n[Exploring] Obstacle detected at {front_dist:.1f} cm")
        stop_motors()
        current_state = State.VERIFYING_OBSTACLE
        return
    
    move_forward(FORWARD_SPEED)
    time.sleep(0.1)

def handle_verifying_obstacle_state():
    """
    VERIFYING_OBSTACLE: confirm the obstacle is real
    Goes to WALL_FOLLOWING if confirmed, otherwise back to EXPLORING
    """
    global current_state, wall_follow_direction, wall_follow_start_time
    
    confirmed, count = verify_front_obstacle()
    
    if not confirmed:
        print("[State] False alarm - resuming exploration")
        current_state = State.EXPLORING
        return
    
    print("\n" + "=" * 70)
    print("INITIATING WALL-FOLLOWING BEHAVIOR")
    print("=" * 70)
    
    print("\n[WallFollow] Backing up...")
    current_state = State.BACKING_UP
    move_backward(BACKUP_SPEED)
    time.sleep(BACKUP_TIME)
    stop_motors()
    time.sleep(0.2)
    
    # Pick a random side to go around the obstacle
    wall_follow_direction = random.choice(['left', 'right'])
    print(f"[WallFollow] Chose to go around {wall_follow_direction.upper()} side")
    
    execute_turn(wall_follow_direction, 90)
    
    # The wall is on the side opposite the turn, so point the rear sensor there
    opposite_direction = 'right' if wall_follow_direction == 'left' else 'left'
    point_rear_sensor(opposite_direction)
    
    wall_follow_start_time = time.time()
    current_state = State.WALL_FOLLOWING
    print(f"[State] → WALL_FOLLOWING (wall on {opposite_direction} side)")

def handle_wall_following_state():
    """
    WALL_FOLLOWING: drive forward while tracking the wall with the rear sensor
    - Rear sensor detects when the wall ends
    - Front sensor detects new obstacles
    """
    global current_state
    
    follow_time = time.time() - wall_follow_start_time
    
    rear_dist = measure_rear()
    front_dist = measure_front()
    
    print(f"[WallFollow] Rear: {rear_dist:5.1f} cm | Front: {front_dist:5.1f} cm | Time: {follow_time:.1f}s", end="\r")
    
    # Front obstacle while following a wall
    if 0 < front_dist < FRONT_OBSTACLE_DISTANCE:
        print(f"\n[WallFollow] ⚠️  Front obstacle at {front_dist:.1f} cm while wall-following!")
        
        if follow_time > MIN_WALL_FOLLOW_TIME:
            stop_motors()
            current_state = State.BLOCKED_BOTH_SIDES
            return
        else:
            print(f"[WallFollow] Too soon to give up - continuing (need {MIN_WALL_FOLLOW_TIME}s)")
    
    # Is the wall still there?
    if rear_dist > 0:
        if rear_dist > WALL_LOST_THRESHOLD:
            print(f"\n[WallFollow] ✓ Wall lost at {rear_dist:.1f} cm - reached corner!")
            
            if follow_time > MIN_WALL_FOLLOW_TIME:
                stop_motors()
                current_state = State.CLEARING_CORNER
                return
            else:
                print(f"[WallFollow] Too soon - need {MIN_WALL_FOLLOW_TIME}s minimum")
        
        # Adjust speed based on distance to the wall
        if rear_dist < WALL_FOLLOW_DISTANCE_MIN:
            move_forward(WALL_FOLLOW_SPEED - 10)
        elif rear_dist > WALL_FOLLOW_DISTANCE_MAX:
            move_forward(WALL_FOLLOW_SPEED - 5)
        else:
            move_forward(WALL_FOLLOW_SPEED)
    else:
        # No valid rear reading - keep going
        move_forward(WALL_FOLLOW_SPEED)
    
    time.sleep(0.1)

def handle_blocked_both_sides_state():
    """
    BLOCKED_BOTH_SIDES: hit an obstacle while wall-following
    Turn around 180° and try the other side of the original obstacle
    """
    global current_state, wall_follow_direction, wall_follow_start_time
    
    print("\n" + "=" * 70)
    print("BLOCKED ON BOTH SIDES - TURNING AROUND")
    print("=" * 70)
    
    print("[Blocked] Backing up...")
    move_backward(BACKUP_SPEED)
    time.sleep(BACKUP_TIME)
    stop_motors()
    time.sleep(0.2)
    
    turn_direction = wall_follow_direction
    execute_turn(turn_direction, 180)
    
    # After a 180° turn the wall is on the opposite side, so flip the servo
    print("[Blocked] Adjusting rear sensor...")
    wall_side = 'right' if wall_follow_direction == 'left' else 'left'
    new_wall_side = 'left' if wall_side == 'right' else 'right'
    point_rear_sensor(new_wall_side)
    
    print(f"[Blocked] Now following wall on {new_wall_side} side")
    
    wall_follow_start_time = time.time()
    current_state = State.WALL_FOLLOWING

def handle_clearing_corner_state():
    """
    CLEARING_CORNER: drive forward past the corner before turning,
    so the robot doesn't clip it
    """
    global current_state, wall_follow_direction
    
    print(f"\n[Corner] Clearing corner - driving forward for {CORNER_CLEAR_TIME}s...")
    
    move_forward(FORWARD_SPEED)
    
    start_time = time.time()
    while (time.time() - start_time) < CORNER_CLEAR_TIME:
        front_dist = measure_front()
        if 0 < front_dist < FRONT_OBSTACLE_DISTANCE:
            print(f"\n[Corner] ⚠️  Hit obstacle while clearing corner!")
            stop_motors()
            current_state = State.VERIFYING_OBSTACLE
            return
        
        elapsed = time.time() - start_time
        print(f"[Corner] Clearing... {elapsed:.1f}/{CORNER_CLEAR_TIME}s", end="\r")
        time.sleep(0.1)
    
    stop_motors()
    print(f"\n[Corner] ✓ Corner cleared")
    
    # Turn back toward the original path (opposite of the first turn)
    return_direction = 'left' if wall_follow_direction == 'right' else 'right'
    
    print(f"[Corner] Turning {return_direction.upper()} to resume original path")
    execute_turn(return_direction, 90)
    
    point_rear_sensor('center')
    
    print("[State] → EXPLORING")
    print("=" * 70)
    print("RESUMING NORMAL NAVIGATION")
    print("=" * 70 + "\n")
    
    current_state = State.EXPLORING

# ==================== STARTUP ====================
def startup_countdown():
    """Countdown before the robot starts moving"""
    print("\n" + "=" * 70)
    print("⏰ STARTUP COUNTDOWN")
    print("=" * 70)
    print(f"\nRobot will start in {STARTUP_DELAY} seconds")
    print("Place your robot in the starting position now!\n")
    
    for i in range(STARTUP_DELAY, 0, -1):
        print(f"  Starting in {i:2d} seconds...", end="\r")
        time.sleep(1)
    
    print("\n")
    print("🚀 " + "=" * 64 + " 🚀")
    print("🚀  ROBOT ACTIVE - ADVANCED WALL-FOLLOWING MODE ENGAGED  🚀")
    print("🚀 " + "=" * 64 + " 🚀")
    print()

# ==================== MAIN LOOP ====================
def main_loop():
    """Main control loop: runs the handler for the current state"""
    global current_state
    
    startup_countdown()
    
    current_state = State.EXPLORING
    print("[State] → EXPLORING")
    print("Moving forward...\n")
    
    try:
        while True:
            if current_state == State.EXPLORING:
                handle_exploring_state()
            
            elif current_state == State.VERIFYING_OBSTACLE:
                handle_verifying_obstacle_state()
            
            elif current_state == State.WALL_FOLLOWING:
                handle_wall_following_state()
            
            elif current_state == State.BLOCKED_BOTH_SIDES:
                handle_blocked_both_sides_state()
            
            elif current_state == State.CLEARING_CORNER:
                handle_clearing_corner_state()
            
            else:
                print(f"[Error] Unknown state: {current_state}")
                current_state = State.EXPLORING
    
    except KeyboardInterrupt:
        print("\n\n" + "=" * 70)
        print("STOPPING ROBOT - User pressed Ctrl+C")
        print("=" * 70)
        stop_motors()
        cleanup()

# ==================== CLEANUP ====================
def cleanup():
    """Stop motors, recenter the servo and release GPIO"""
    print("\nCleaning up...")
    
    stop_motors()
    set_servo_angle(SERVO_CENTER)
    
    pwm_a.stop()
    pwm_b.stop()
    servo_pwm.stop()
    
    GPIO.cleanup()
    
    print("✓ Cleanup complete!")
    print("\nRobot stopped safely. Goodbye! 👋\n")

# ==================== RUN PROGRAM ====================
if __name__ == "__main__":
    setup()
    main_loop()