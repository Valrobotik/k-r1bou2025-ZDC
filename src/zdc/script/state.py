#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int8

rospy.init_node("state")
state_publisher = rospy.Publisher("state", Int8, queue_size=1)

while not rospy.is_shutdown():
    state = int(input("Enter a state: "))
    state_publisher.publish(state)