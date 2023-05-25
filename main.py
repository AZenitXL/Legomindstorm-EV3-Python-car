#!/usr/bin/env pybricks-micropython
import time
from threading import Thread
import sys
import math
import struct


# from pybricks import ev3brick as brick
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
 

# A helper function for converting stick values (0 - 255)
# to more usable numbers (-100 - 100)
def scale(val, src, dst):
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

def engine():

    SPORT = 1 # 1 for Sport; 0 for Eco

    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)

    right_trigger = 0
    left_trigger = 0

    infile_path = "/dev/input/event4"
    in_file = open(infile_path, "rb")
    FORMAT = 'llHHI'    
    EVENT_SIZE = struct.calcsize(FORMAT)
    event = in_file.read(EVENT_SIZE)

    while event:
        (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(FORMAT, event)
        if ev_type == 3 and code == 5:
            right_trigger = value
        if ev_type == 3 and code == 2:
            left_trigger = value
        if ev_type == 1 and code == 311 and value == 1:
            SPORT = 1
        if ev_type == 1 and code == 310 and value == 1:
            SPORT = 0

        if SPORT == 1:
            forward = scale(right_trigger, (0,255), (0,100))
            reverse = scale(left_trigger, (0,255), (0,-100))
        else:
            forward = scale(right_trigger, (0,255), (0,60))
            reverse = scale(left_trigger, (0,255), (0,-60))

        left_motor.dc(forward + reverse)
        right_motor.dc(forward + reverse)

        event = in_file.read(EVENT_SIZE)

def steering():
    
    steering = Motor(Port.D)

    left_stick_y = 124
    SPORT = 1 # 1 for Sport; 0 for Eco

    infile_path = "/dev/input/event4"
    in_file = open(infile_path, "rb")
    FORMAT = 'llHHI'    
    EVENT_SIZE = struct.calcsize(FORMAT)
    event = in_file.read(EVENT_SIZE)

    while event:
        (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(FORMAT, event)
        if ev_type == 3 and code == 0:
            left_stick_y = value
        if ev_type == 1 and code == 311 and value == 1:
            SPORT = 1
        if ev_type == 1 and code == 310 and value == 1:
            SPORT = 0

        if SPORT == 1:
            angle = scale(left_stick_y, (0,255), (18,-18))
        else:
           angle = scale(left_stick_y, (0,255), (80,-80))

        steering.run_target(9999, angle, then=Stop.HOLD, wait=False)

        event = in_file.read(EVENT_SIZE)

def audio():
    
    robot = EV3Brick()
    motor_start = r"/home/robot/Ferrari/motor_start.wav"
    horn = r"/home/robot/Ferrari/horn_1.wav"

    infile_path = "/dev/input/event4"
    in_file = open(infile_path, "rb")
    FORMAT = 'llHHI'    
    EVENT_SIZE = struct.calcsize(FORMAT)
    event = in_file.read(EVENT_SIZE)

    while event:
        (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(FORMAT, event)
        if ev_type == 1 and code == 305 and value == 0:
            robot.speaker.say("eat my dust")
        if ev_type == 1 and code == 304 and value == 0:
            robot.speaker.play_file(motor_start)
        if ev_type == 1 and code == 308 and value == 0:
            robot.speaker.play_file(horn)
        if ev_type == 1 and code == 307 and value == 0:
            robot.speaker.say("U suck")
        if ev_type == 1 and code == 311 and value == 1:
            robot.speaker.say("Sport mode")
        if ev_type == 1 and code == 310 and value == 1:
            robot.speaker.say("Eco mode") 


        event = in_file.read(EVENT_SIZE)


tAction = Thread(target=engine, name='tAction')
tAudio = Thread(target=audio, name='tAudio')
tSteering = Thread(target=steering, name='tSteering')

tAction.start()
tAudio.start()
tSteering.start()
