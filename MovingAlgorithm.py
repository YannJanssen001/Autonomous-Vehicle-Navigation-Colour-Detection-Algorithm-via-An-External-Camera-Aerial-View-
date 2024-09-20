#!/usr/bin/env python3
# File name   : move.py
# Description : Control Motor
# Product     : GWR
# Website     : www.gewbot.com
# Author      : William
# Date        : 2019/07/24
import time
import RPi.GPIO as GPIO

# motor_EN_A: Pin7  |  motor_EN_B: Pin11
# motor_A:  Pin8,Pin10    |  motor_B: Pin13,Pin12

Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 26
Motor_A_Pin2  = 21
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 1
Dir_backward  = 0

left_forward  = 1
left_backward = 0

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0

def motorStop():#Motor stops
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)
	GPIO.output(Motor_B_EN, GPIO.LOW)


def setup():#Motor initialization
	global pwm_A, pwm_B
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(Motor_A_EN, GPIO.OUT)
	GPIO.setup(Motor_B_EN, GPIO.OUT)
	GPIO.setup(Motor_A_Pin1, GPIO.OUT)
	GPIO.setup(Motor_A_Pin2, GPIO.OUT)
	GPIO.setup(Motor_B_Pin1, GPIO.OUT)
	GPIO.setup(Motor_B_Pin2, GPIO.OUT)

	motorStop()
	try:
		pwm_A = GPIO.PWM(Motor_A_EN, 1000)
		pwm_B = GPIO.PWM(Motor_B_EN, 1000)
	except:
		pass


def motor_left(status, direction, speed):#Motor 2 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_B_Pin1, GPIO.LOW)
		GPIO.output(Motor_B_Pin2, GPIO.LOW)
		GPIO.output(Motor_B_EN, GPIO.LOW)
	else:
		if direction == Dir_backward:
			GPIO.output(Motor_B_Pin1, GPIO.HIGH)
			GPIO.output(Motor_B_Pin2, GPIO.LOW)
			pwm_B.start(100)
			pwm_B.ChangeDutyCycle(speed)
		elif direction == Dir_forward:
			GPIO.output(Motor_B_Pin1, GPIO.LOW)
			GPIO.output(Motor_B_Pin2, GPIO.HIGH)
			pwm_B.start(0)
			pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_A_Pin1, GPIO.LOW)
		GPIO.output(Motor_A_Pin2, GPIO.LOW)
		GPIO.output(Motor_A_EN, GPIO.LOW)
	else:
		if direction == Dir_forward:#
			GPIO.output(Motor_A_Pin1, GPIO.HIGH)
			GPIO.output(Motor_A_Pin2, GPIO.LOW)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
		elif direction == Dir_backward:
			GPIO.output(Motor_A_Pin1, GPIO.LOW)
			GPIO.output(Motor_A_Pin2, GPIO.HIGH)
			pwm_A.start(0)
			pwm_A.ChangeDutyCycle(speed)
	return direction


def move(speed, direction, turn, radius=0.6):   # 0 < radius <= 1  
	#speed = 100
	if direction == 'forward':
		if turn == 'right':
			motor_left(0, left_backward, int(speed*radius))
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(0, right_backward, int(speed*radius))
		else:
			motor_left(1, left_forward, speed)
			motor_right(1, right_forward, speed)
	elif direction == 'backward':
		if turn == 'right':
			motor_left(0, left_forward, int(speed*radius))
			motor_right(1, right_backward, speed)
		elif turn == 'left':
			motor_left(1, left_backward, speed)
			motor_right(0, right_forward, int(speed*radius))
		else:
			motor_left(1, left_backward, speed)
			motor_right(1, right_backward, speed)
	elif direction == 'no':
		if turn == 'right':
			motor_left(1, left_backward, speed)
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(1, right_backward, speed)
		else:
			motorStop()
	else:
		pass


def destroy():
	motorStop()
	GPIO.cleanup()             # Release resource

import math
# Open the file in read mode
with open('/home/Mars2024/pixel_distances.txt', 'r') as file:
    # Read the data from the file
    data = file.read()

# Split the data into x_distance, y_distance, and total_distance
x_distance, y_distance, total_distance = map(float, data.split(','))

# Now you can use x_distance, y_distance, and total_distance in your script


class RobotMovement:
    def __init__(self, start=(0, 0), end=(0, 0)):
        self.positions = [start]  # Start at the given position
        self.end = end  # End at the given position
        self.data = [
            {"t": 0.0, "d": 0, "pixel_d": 0},
            {"t": 0.5, "d": 13.7, "pixel_d": 226},
            {"t": 1.0, "d": 54.9, "pixel_d": 674},
            {"t": 1.5, "d": 79.1, "pixel_d": 1185},
            {"t": 2.0, "d": 107, "pixel_d": 1673},
            {"t": 2.5, "d": 135.9, "pixel_d": 2130},
            {"t": 3.0, "d": 165.3, "pixel_d": 2351},
            {"t": 3.5, "d": 194.7, "pixel_d": 2886},
            {"t": 4.0, "d": 217.3, "pixel_d": 3035},
            {"t": 4.5, "d": 247.7, "pixel_d": 3170},
        ]
        self.gradients = self.calculate_gradients(self.data)
        setup()

    def calculate_gradients(self, data):
        gradients = []
        for i in range(1, len(data)):
            dt = data[i]["t"] - data[i-1]["t"]
            dpixel_d = data[i]["pixel_d"] - data[i-1]["pixel_d"]
            gradients.append({"t": data[i]["t"], "pixel_d_slope": dpixel_d/dt})
        return gradients

    def interpolate(self, pixel_d):
        for i in range(1, len(self.data)):
            if self.data[i]["pixel_d"] >= pixel_d:
                # Interpolate between the two data points
                t1 = self.data[i-1]["t"]
                t2 = self.data[i]["t"]
                pixel_d1 = self.data[i-1]["pixel_d"]
                pixel_d2 = self.data[i]["pixel_d"]
                dt = ((pixel_d - pixel_d1) / (pixel_d2 - pixel_d1)) * (t2 - t1)
                return t1 + dt
        return None

    def move_forward(self, t):
        x, y = self.positions[-1]  # Get the last position
        pixel_d = self.interpolate(t)  # Calculate the pixel distance moved
        if pixel_d is not None:
            self.positions.append((x, y + pixel_d))  # Move forward by the pixel distance moved
        move(50, 'forward', 'no', 1)
        time.sleep(t)
        motorStop()

    def move_backward(self, t):
        x, y = self.positions[-1]  # Get the last position
        pixel_d = self.interpolate(t)  # Calculate the pixel distance moved
        if pixel_d is not None:
            self.positions.append((x, y - pixel_d))  # Move backward by the pixel distance moved
        move(50, 'backward', 'no', 1)
        time.sleep(t)
        motorStop()
		
    def turn_right(self, degrees):
        move(100, 'no', 'right', 1)
        if degrees == 90:
            time.sleep(1.9)
            motorStop()
        elif degrees == 45:
            time.sleep(0.65)
            motorStop()

    def turn_left(self, degrees):
        move(100, 'no', 'right', 1)
        if degrees == 90:
            time.sleep(1.9)
            motorStop()
        elif degrees == 45:
            time.sleep(0.65)
            motorStop()

    def turn_axis(self, degrees):
        xo, xf = self.positions[-1][0], self.end[0]
        if xf > xo:
            self.turn_right(degrees)
        elif xf < xo:
            self.turn_left(degrees)
        
    def move_axis(self, x, y):
        # Store the initial x and y positions
        initial_x = self.positions[-1][0]
        initial_y = self.positions[-1][1]

        # Calculate the pixel distance to move in the y direction
        pixel_d_y = abs(y - initial_y)

        # Use interpolation to estimate the time required to move the given pixel distance
        dt_y = self.interpolate(pixel_d_y)

        # Move forward or backward depending on the sign of y
        if dt_y is not None:
            if y > initial_y:
                self.move_forward(dt_y)
            elif y < initial_y:
                self.move_backward(dt_y)
        else:
            print("Error: pixel distance is outside the range of the data")

        # Calculate the pixel distance to move in the x direction
        pixel_d_x = abs(x - initial_x)

        # Use interpolation to estimate the time required to move the given pixel distance
        dt_x = self.interpolate(pixel_d_x)

        # Turn right or left depending on the sign of x and then move forward
        if dt_x is not None:
            if x > initial_x:
                self.turn_axis(90)  # Turn right
                self.move_forward(dt_x+0.2)
            elif x < initial_x:
                self.turn_axis(-90)  # Turn left
                self.move_forward(dt_x)
        else:
            print("Error: pixel distance is outside the range of the data")

if __name__ == '__main__':
    try:

	# Create a RobotMovement instance with the robot's center as the start point and the object's center as the end point
        robot = RobotMovement((0,0), end=(x_distance, y_distance))

        # Use the robot's methods to move it
        robot.move_axis(x_distance, y_distance)


        destroy()
    except KeyboardInterrupt:
        destroy()