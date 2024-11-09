#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from time import sleep

def move_rectangle():
    rospy.init_node('move_rectangle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)

    move_cmd = Twist()
    linear_speed = 2.5
    angular_speed = 1.57

    for _ in range(2):
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        sleep(2)

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 1.57
        pub.publish(move_cmd)
        sleep(1)

        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        sleep(2)

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 1.57
        pub.publish(move_cmd)
        sleep(2)

        move_cmd.linear.x = linear_speed + 3
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        sleep(2)

    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Finished moving in a rectangle")

if __name__ == '__main__':
    try:
        move_rectangle()
    except rospy.ROSInterruptException:
        pass
