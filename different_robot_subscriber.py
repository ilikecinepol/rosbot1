#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# from std_msgs.msg import Float32
from rosbot1.msg import car_motion_message
import RPi.GPIO as GPIO

in1 = 16
in2 = 26
in3 = 20
in4 = 21
left_speed_pin = 13
right_speed_pin = 12

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pins = [in1, in2, in3, in4, speed_pin, servo_pin]
GPIO.setup(pins, GPIO.OUT)

left_speed = GPIO.PWM(left_speed_pin, 50)
left_speed.start(0)
right_speed = GPIO.PWM(right_speed_pin, 50)
right_speed.start(0)


def limiter(val, min=-100, max=100):
    return min if val < min else max if val > max else val


def drive(speed, turn):
    if speed > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.HIGH)

    left_speed.ChangeDutyCycle(speed + (turn * 7))
    right_speed.ChangeDutyCycle(speed - (turn * 7))


def chatter_callback(message):
    speed = -1 * message.speed * 100
    angle = message.angle * 10
    if angle == 0.0:
        angle = 9
    elif angle > 0:
        angle = 11
    else:
        angle = 7

    drive(speed, angle)


rospy.init_node('car_driving_subscriber', anonymous=True)

rospy.Subscriber('robot_speed_topic', car_motion_message, chatter_callback)
rospy.spin()
