import curses
import time
import math
import Adafruit_PCA9685
import board
import busio

# PWM motor driver and pins
i2c = busio.I2C(board.SCL, board.SDA)
pca = Adafruit_PCA9685.PCA9685(i2c)
pca.frequency = 50

PWM_STEERING_PIN = 1
PWM_THROTTLE_PIN = 0

# Odometry constants
WHEEL_DIAMETER = 0.065 # meters
TICKS_PER_REV = 32
GEAR_RATIO = 120
METERS_PER_TICK = math.pi * WHEEL_DIAMETER / (TICKS_PER_REV * GEAR_RATIO)

# Odometry variables
x = 0.0
y = 0.0
theta = math.pi / 2.0  # start pointing straight up

# Setup GPIO pins for duty cycle output
PWM_PIN = 2
DIR_PIN = 3
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

pwm = GPIO.PWM(PWM_PIN, 50)
pwm.start(0)

# Set initial steering and throttle to neutral
pca.channels[PWM_STEERING_PIN].duty_cycle = 3072  # 1.5ms pulse
pca.channels[PWM_THROTTLE_PIN].duty_cycle = 3072  # 1.5ms pulse

def set_speed(speed):
    if speed >= 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)
        speed = -speed
    pca.channels[PWM_THROTTLE_PIN].duty_cycle = int(3072 + speed / 100 * (4096 - 3072))

def set_steering(duty_cycle):
    pca.channels[PWM_STEERING_PIN].duty_cycle = duty_cycle

def move_forward(speed):
    set_speed(speed)
    pwm.ChangeDutyCycle(speed)

def move_backward(speed):
    set_speed(-speed)
    pwm.ChangeDutyCycle(-speed)

def turn_left(duty_cycle):
    set_steering(duty_cycle)
    pwm.ChangeDutyCycle(0)

def turn_right(duty_cycle):
    set_steering(duty_cycle)
    pwm.ChangeDutyCycle(0)

# Initialize curses for keyboard input
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

try:
    while True:
        char = screen.getch()
        if char == ord('q'):
            break
        elif char == curses.KEY_UP:
            move_forward(50)
            # Update odometry
            x += METERS_PER_TICK * math.cos(theta)
            y += METERS_PER_TICK * math.sin(theta)
        elif char == curses.KEY_DOWN:
            move_backward(50)
            # Update odometry
            x -= METERS_PER_TICK * math.cos(theta)
            y -= METERS_PER_TICK * math.sin(theta)