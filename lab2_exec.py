#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])
midposition = [164.83*pi/180.0, -91.27*pi/180.0, 116.03*pi/180.0, -115.42*pi/180.0, -93.42*pi/180.0, -14.32*pi/180.0]

# Hanoi tower location 1
Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""
Q11 = [142.07*pi/180.0, -56.26*pi/180.0, 123.53*pi/180.0, -160.96*pi/180.0, -94.23*pi/180.0, -39.08*pi/180.0]
Q12 = [142.09*pi/180.0, -64.36*pi/180.0, 122.62*pi/180.0, -151.95*pi/180.0, -94.16*pi/180.0, -39.15*pi/180.0]
Q13 = [142.12*pi/180.0, -71.73*pi/180.0, 120.29*pi/180.0, -142.27*pi/180.0, -94.10*pi/180.0, -39.23*pi/180.0]
Q21 = [162.99*pi/180.0, -58.30*pi/180.0, 126.71*pi/180.0, -159.12*pi/180.0, -93.59*pi/180.0, -15.65*pi/180.0]
Q22 = [163.02*pi/180.0, -66.75*pi/180.0, 125.62*pi/180.0, -149.58*pi/180.0, -93.52*pi/180.0, -15.71*pi/180.0]
Q23 = [163.02*pi/180.0, -74.31*pi/180.0, 123.13*pi/180.0, -139.55*pi/180.0, -93.47*pi/180.0, -15.79*pi/180.0]
Q31 = [188.25*pi/180.0, -53.55*pi/180.0, 112.99*pi/180.0, -148.61*pi/180.0, -95.36*pi/180.0, 7.17*pi/180.0]
Q32 = [188.25*pi/180.0, -60.48*pi/180.0, 111.89*pi/180.0, -140.49*pi/180.0, -95.30*pi/180.0, 7.11*pi/180.0]
Q33 = [188.29*pi/180.0, -66.40*pi/180.0, 109.60*pi/180.0, -132.31*pi/180.0, -95.27*pi/180.0, 7.03*pi/180.0]

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def gripper_callback(msg):
    global digital_in_0
    global analog_in_0

    digital_in_0 = msg.DIGIN
    analog_in_0 = msg.AIN0

############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while at_goal == 0:

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if spin_count > SPIN_RATE*5:

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while at_goal == 0:

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if spin_count >  SPIN_RATE*5:

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    ### Hint: Use the Q array to map out your towers by location and "height".
    error = 0

    rospy.loginfo("Finding the block...")
    # move the arm to grip the block
    move_arm(pub_cmd, loop_rate, Q[start_loc-1][start_height-1], 4.0, 4.0)
    time.sleep(0.5)
    gripper(pub_cmd,loop_rate,suction_on)
    time.sleep(1.0)
    if not digital_in_0:
        error = 1
        gripper(pub_cmd,loop_rate,suction_off)
        rospy.loginfo("Fail to grip the block")
        return error
    
    #################################need an intermediate position###########################

    rospy.loginfo("Moving to the current destination...")
    move_arm(pub_cmd,loop_rate,midposition,4.0,4.0)
    # move the are to the destination
    move_arm(pub_cmd,loop_rate,Q[end_loc-1][end_height-1],4.0,4.0)
    time.sleep(0.5)
    gripper(pub_cmd,loop_rate,suction_off)
    time.sleep(1.0)

    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    gripper_in = rospy.Subscriber('ur3/gripper_input',gripper_input,gripper_callback)

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = False
    start_point = 0
    mid_point = 0
    des_point = 0

    while not input_done:
        input_start = input("Enter the start point of the tower <Either 1 2 3 > ")
        print("You entered " + input_start + "\n")

        input_des = input(" Entering the destination of the tower <Either 1 2 3>")
        print("Your entered" + input_des + "\n")

        if int(input_start) < 1:
            print("please enter a number in 1-3")
        elif int(input_des) > 3 or int(input_des) == int(input_start):
            print("please enter a dest in 1-3 and different from the start point")
        else:
            input_done = True

        start_point = int(input_start)
        des_point = int(input_des)
        mid_point = 6 - start_point - des_point




    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while rospy.is_shutdown():
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input
    move_arm(pub_command,loop_rate,home,4.0,4.0)
    if move_block(pub_command,loop_rate,start_point,3,des_point,1):
        gripper(pub_command,loop_rate,suction_off)
        rospy.loginfo("error, arm is halt")
        return 1
    move_arm(pub_command,loop_rate,midposition,4.0,4.0)
    if move_block(pub_command,loop_rate,start_point,2,mid_point,1):
        gripper(pub_command, loop_rate, suction_off)
        rospy.loginfo("error, arm is halt")
        return 1
    move_arm(pub_command,loop_rate,midposition,4.0,4.0)
    if move_block(pub_command,loop_rate,des_point,1,mid_point,2):
        gripper(pub_command, loop_rate, suction_off)
        rospy.loginfo("error, arm is halt")
        return 1
    move_arm(pub_command,loop_rate,midposition,4.0,4.0)
    if move_block(pub_command, loop_rate, start_point,1,des_point,1):
        gripper(pub_command, loop_rate, suction_off)
        rospy.loginfo("error, arm is halt")
        return 1
    move_arm(pub_command,loop_rate,midposition,4.0,4.0)
    if move_block(pub_command,loop_rate,mid_point,2,start_point,1):
        gripper(pub_command, loop_rate, suction_off)
        rospy.loginfo("error, arm is halt")
        return 1
    move_arm(pub_command,loop_rate,midposition,4.0,4.0)
    if move_block(pub_command, loop_rate,mid_point,1,des_point,2):
        gripper(pub_command, loop_rate, suction_off)
        rospy.loginfo("error, arm is halt")
        return 1
    move_arm(pub_command,loop_rate,midposition,4.0,4.0)
    if move_block(pub_command,loop_rate,start_point,1,des_point,3):
        gripper(pub_command, loop_rate, suction_off)
        rospy.loginfo("error, arm is halt")
        return 1
    move_arm(pub_command,loop_rate,midposition,4.0,4.0)

    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
