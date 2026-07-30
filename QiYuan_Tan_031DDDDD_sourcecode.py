# QY RSSP CW2

import RPi.GPIO as GPIO
import spidev
import time

# Pin Definitions (BCM numbering)
# -----------------------------
ULTRASONIC_TRIG = 20     # U.Trig
ULTRASONIC_ECHO = 23     # U.Echo
LIMIT_SWITCH_PIN  = 17   # Limit switch
START_BUTTON_PIN  = 22   # Start button
EMERGENCY_STOP_PIN = 27  # Emergency stop button
HOMING_BUTTON_PIN = 16   # Homing button
SERVO_PIN = 18           # Servo
LED_RED    = 5           # Red (Emergency/Obstacle-Unsafe)
LED_YELLOW = 6           # Yellow (No Obstacle-Safe)
LED_BLUE   = 13          # Blue (Task Completed)
LED_GREEN  = 19          # Green (Readiness/Homing Completed)

# SPI for MCP3008 (Analog sensors)
# -----------------------------
spi = spidev.SpiDev()
spi.open(0, 0)   # Bus 0, Device 0 (CE0)
spi.max_speed_hz = 1350000

# GPIO Setup
# -----------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
input_pins = [LIMIT_SWITCH_PIN, START_BUTTON_PIN, EMERGENCY_STOP_PIN, HOMING_BUTTON_PIN]
for pin in input_pins:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
output_pins = [ULTRASONIC_TRIG, SERVO_PIN, LED_RED, LED_YELLOW, LED_BLUE, LED_GREEN]
GPIO.setup(output_pins, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO, GPIO.IN)
for led in [LED_RED, LED_YELLOW, LED_BLUE, LED_GREEN]:
    GPIO.output(led, False)

# Setup servo PWM at 50Hz
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# Global variables for smoothing potentiometer reading and homing state
last_pot_angle = None
pot_samples = []
homing_active = False

# --------------------
# - Helper Functions -
# --------------------
def safe_sleep(duration):
    """Sleep in 0.1 sec increments while checking emergency and homing state."""
    interval = 0.1
    steps = int(duration / interval)
    for _ in range(steps):
        check_emergency()
        check_homing()
        time.sleep(interval)
    remainder = duration - steps * interval
    if remainder > 0:
        check_emergency()
        check_homing()
        time.sleep(remainder)

def read_channel(channel):
    """Read SPI data from MCP3008 on the specified channel (0-7)."""
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def get_distance_ldr():
    """Read the LDR sensor (CH1) and return a pseudo-distance in cm."""
    raw_value = read_channel(1)
    return (30 - (raw_value / 34))  # pseudo-distance in cm

def get_distance_ldr_filtered(num_samples=5):
    """Get a filtered LDR reading using a simple moving average."""
    samples = []
    for _ in range(num_samples):
        samples.append(get_distance_ldr())
        time.sleep(0.01)  # Short delay between samples
    filtered_distance = sum(samples) / len(samples)
    return filtered_distance

def get_distance_ultrasonic():
    """Trigger the ultrasonic sensor and measure distance in cm."""
    GPIO.output(ULTRASONIC_TRIG, False)
    safe_sleep(0.05)
    GPIO.output(ULTRASONIC_TRIG, True)
    safe_sleep(0.00001)
    GPIO.output(ULTRASONIC_TRIG, False)

    start_time = time.time()
    while GPIO.input(ULTRASONIC_ECHO) == 0:
        start_time = time.time()
        check_emergency()
        check_homing()
    while GPIO.input(ULTRASONIC_ECHO) == 1:
        stop_time = time.time()
        check_emergency()
        check_homing()
    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2
    return round(distance, 2)

def move_servo(angle):
    """Move the servo to a specified angle (use a simple duty cycle conversion)."""
    duty = (angle / 18) + 2
    servo.ChangeDutyCycle(duty)
    safe_sleep(0.5)
    servo.ChangeDutyCycle(0)


def update_servo_with_pot():
    """
    Read the potentiometer (CH0) and map its value (0–1023)
    to a servo angle between 45° and 180°. Update the servo position accordingly.
    Print the current mapped angle if it changes more than 2° compared to the last update.(Reduce Jitter)
    """
    global last_pot_angle, pot_samples
    pot_value = read_channel(0)
    pot_samples.append(pot_value)
    if len(pot_samples) > 5:
        pot_samples.pop(0)
    avg_value = sum(pot_samples) / len(pot_samples)
    angle = (avg_value / 1023.0) * (180 - 45) + 45  # Map from 45° to 180°
    if last_pot_angle is None or abs(angle - last_pot_angle) > 2:
        print(f"Current Angle: {angle:.1f} °")
        move_servo(angle)
        last_pot_angle = angle

def safety_check():
    """Check the ultrasonic sensor for safety (distance > 15 cm)."""
    distance = get_distance_ultrasonic()
    if distance > 15:
        return True, distance
    else:
        return False, distance

def perform_limit_switch_function():
    """If the servo in 0° touches the limit switch, execute the homing procedure."""
    if GPIO.input(LIMIT_SWITCH_PIN) == GPIO.LOW:
        print("Limit Switch is pressed.")
        safe_sleep(1)
        move_servo(45)
        GPIO.output(LED_GREEN, True)
        print("Green LED on. Current angle: 45°. Homing procedure is done.")
        safe_sleep(1)
        GPIO.output(LED_GREEN, False)

def handle_emergency():
    """
    Immediately halt operations. Turn off all LEDs except the red LED,
    which blinks continuously until the emergency button is pressed and then
    released to resume operation.
    """
    print("Emergency button activated. All operation stopped.")
    GPIO.output(LED_YELLOW, False)
    GPIO.output(LED_BLUE, False)
    GPIO.output(LED_GREEN, False)
    
    while GPIO.input(EMERGENCY_STOP_PIN) == GPIO.LOW:
        GPIO.output(LED_RED, True)
        time.sleep(0.1)
        GPIO.output(LED_RED, False)
        time.sleep(0.1)
    
    print("Waiting for emergency button press to resume...")
    while GPIO.input(EMERGENCY_STOP_PIN) == GPIO.HIGH:
        GPIO.output(LED_RED, True)
        time.sleep(0.1)
        GPIO.output(LED_RED, False)
        time.sleep(0.1)
    while GPIO.input(EMERGENCY_STOP_PIN) == GPIO.LOW:
        GPIO.output(LED_RED, True)
        time.sleep(0.1)
        GPIO.output(LED_RED, False)
        time.sleep(0.1)
    print("Emergency button released. Resuming operation.")

def check_emergency():
    if GPIO.input(EMERGENCY_STOP_PIN) == GPIO.LOW:
        handle_emergency()

def check_homing():
    """Check if the homing button is pressed, if not already in homing mode, execute the homing procedure."""
    global homing_active
    if GPIO.input(HOMING_BUTTON_PIN) == GPIO.LOW and not homing_active:
        homing_active = True
        print("Homing button pressed. Executing homing procedure.")
        move_servo(0)
        safe_sleep(0.5)
        perform_limit_switch_function()
        # Wait until the homing button is released to avoid retriggering
        while GPIO.input(HOMING_BUTTON_PIN) == GPIO.LOW:
            time.sleep(0.1)
        homing_active = False

def normal_operation():
    # Indicate that the system is ready.
    print("Green LED on. System ready.")
    GPIO.output(LED_GREEN, True)
    # While waiting for the start button, update the servo using the potentiometer.
    while GPIO.input(START_BUTTON_PIN) == GPIO.HIGH:
        check_emergency()
        check_homing()
        safe, d = safety_check()
        if not safe:
            return False
        update_servo_with_pot()
        safe_sleep(0.1)
    GPIO.output(LED_GREEN, False)
    
    # Step 1: Ultrasonic detection for pickup.
    while True:
        check_emergency()
        check_homing()
        safe, d = safety_check()
        if not safe:
            return False
        if 15 < d <= 20:
            safe_sleep(1.5)
            safe, d = safety_check()
            if safe and (15 < d <= 20):
                print(f"Object distance: {d}cm. Gripper picked and moving the object.")
                move_servo(90)  # Optionally adjust if needed.
                break
        safe_sleep(0.1)
    
    # Step 2: LDR detection for placement using filtered reading.
    while True:
        check_emergency()
        check_homing()
        safe, _ = safety_check()
        if not safe:
            return False
        ldr_distance = get_distance_ldr_filtered()
        if 15 < ldr_distance <= 20:
            safe_sleep(1.5)
            ldr_distance = get_distance_ldr_filtered()
            if 15 < ldr_distance <= 20:
                print(f"Object distance: {ldr_distance}cm. Gripper released, object is placed.")
                move_servo(135)
                safe_sleep(1.5)
                move_servo(0)
                GPIO.output(LED_BLUE, True)
                print("Blue LED on. Task is completed.")
                safe_sleep(1)
                GPIO.output(LED_BLUE, False)
                break
        safe_sleep(0.1)
    
    # Step 3: Homing procedure via limit switch.
    perform_limit_switch_function()
    return True

# -----------
# -Main Loop-
# -----------
try:
    while True:
        check_emergency()
        check_homing()
        safe, d = safety_check()
        if safe:
            GPIO.output(LED_YELLOW, True)
            print(f"Yellow LED on. Object distance: {d}cm. Safe to start.")
            cycle_result = normal_operation()
            GPIO.output(LED_YELLOW, False)
            if not cycle_result:
                continue
        else:
            for led in [LED_YELLOW, LED_GREEN, LED_BLUE]:
                GPIO.output(led, False)
            print(f"Red LED blinking. Object distance: {d}cm. Unsafe to start.")
            while True:
                check_emergency()
                check_homing()
                safe, d = safety_check()
                if safe:
                    break
                GPIO.output(LED_RED, True)
                safe_sleep(0.5)
                GPIO.output(LED_RED, False)
                safe_sleep(0.5)
            print("Please press homing button.")
            while GPIO.input(HOMING_BUTTON_PIN) == GPIO.HIGH:
                check_emergency()
                check_homing()
                safe_sleep(0.1)
            move_servo(0)
            print("Current Angle: 0 °. In Home position.")
            safe_sleep(0.5)
            perform_limit_switch_function()
            GPIO.output(LED_RED, False)
            safe_sleep(1)

# Ctrl+C
except KeyboardInterrupt:
    print("Program stopped by user.")

finally:
    servo.stop()
    GPIO.cleanup()
      