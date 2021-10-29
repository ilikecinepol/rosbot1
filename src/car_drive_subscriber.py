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
speed_pin = 13
servo_pin = 12

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pins = [in1, in2, in3, in4, speed_pin, servo_pin]
GPIO.setup(pins, GPIO.OUT)

pwm_speed = GPIO.PWM(speed_pin, 50)
pwm_speed.start(0)
pwm_servo = GPIO.PWM(servo_pin, 50)
pwm_servo.start(0)


def drive(speed, turn):
    if speed > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)

    pwm_speed.ChangeDutyCycle(abs(speed))
    pwm_servo.ChangeDutyCycle(abs(turn))


def chatter_callback(message):
    speed = -1 * message.speed * 100
    angle = message.angle * 10
    # rospy.loginfo(speed)
    # print(speed)

    drive(speed, angle)


rospy.init_node('car_driving_subscriber', anonymous=True)

rospy.Subscriber('robot_speed_topic', car_motion_message, chatter_callback)
rospy.spin()
