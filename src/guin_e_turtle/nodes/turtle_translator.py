#!/usr/bin/env python

################################################################################################################
##
##  //   Title : turtle_translator.py
##  //   Author : Paul Pavish
##  //   Website : www.basicsexplained.com/creator
##  //   YT Channel : https://www.youtube.com/channel/UCavN7aolUmBbfcKbDbydvMA
##  //   LinkedIn : https://www.linkedin.com/in/paulpavish/
##  //
##  //   Kindly attribute to Author on any marvelous project you come up with using this piece of code.
##  //   Also show your support on the Author's Youtube Channel. Thankyou.
##  //
##  //   This Python Script is a ROS Node for the GuiN-E Bot V.1.0.0
##  //   This is the 'Turtle2GuiNE' node of the 'guin_e_turtle' package
##  //   This node subscribes to 'turtlesim' package's 'cmd_vel' and 'pose' topics
##  //   This node converts the subscribed data to a 'std_msgs.msg/UInt16MultiArray' type
##  //   The converted data is published to the GuiN-E Bot through '/GuiNE_Bot/Motor' topic.
##  //   The GuiN-E bot Subscribes to this publisher through the help of 'rosserial_ardino' library on device
##  //      - and 'rosserial_python_node' in this package (TCP)
##  //
##  //  This script is purposefully constructed for easy future updation for upcoming GuiN-E Bot versions.
##
################################################################################################################

__author__ = "paul.pavish@gmail.com (Paul Pavish)"


# Importing modules, methods & message types
import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import UInt16MultiArray

# Boolean value to flag turtle's state change
reset = False

# Function to redifine GuiN-E Bot's base motor control values depending on turtle topics
def motor(lin, ang):

    Speed_Val = 200
    M_C = [Speed_Val, 0, 0, 0, 0]

    if lin == 2 and ang == 0:
        M_C[1] = 1
        M_C[2] = 0
        M_C[3] = 1
        M_C[4] = 0
        return M_C
    elif lin == -2 and ang == 0:
        M_C[1] = 0
        M_C[2] = 1
        M_C[3] = 0
        M_C[4] = 1
        return M_C
    elif ang == 2 and lin == 0:
        M_C[1] = 0
        M_C[2] = 1
        M_C[3] = 1
        M_C[4] = 0
        return M_C
    elif ang == -2 and lin == 0:
        M_C[1] = 1
        M_C[2] = 0
        M_C[3] = 0
        M_C[4] = 1
        return M_C
    elif lin == 0 and ang == 0:
        return M_C
        
    

# Subscriber Callback function/method for '/turtle1/cmd_vel' topic
def callback1(turtle):
    
    Motor_Data = UInt16MultiArray()
    Motor_Data.data = motor(turtle.linear.x, turtle.angular.z)
    pub.publish(Motor_Data)
    global reset
    reset = True


# Subscriber Callback function/method for '/turtle1/pose' topic
def callback2(pose):

    lin_vel = pose.linear_velocity
    ang_vel = pose.angular_velocity
    halt = UInt16MultiArray()
    global reset
    if reset:
        if lin_vel == 0 and ang_vel == 0 :
            halt.data = motor(lin_vel, ang_vel)
            pub.publish(halt)
            reset = False

# Subscriber listening Main Function
def listen():

    sub1 = rospy.Subscriber('/turtle1/cmd_vel', Twist, callback1)
    sub2 = rospy.Subscriber('/turtle1/pose', Pose, callback2)
    rospy.spin()



# Defining MAIN method/function
if __name__ == '__main__':

# Node Initialization
    rospy.init_node('Turtle2GuiNE')
# Defining Publisher to GuiN-E Bot
    pub = rospy.Publisher('/GuiNE_Bot/Motor', UInt16MultiArray, queue_size = 10)

    try:
        # Main Method
        listen()

    except rospy.ROSInterruptException:
        pass
    
