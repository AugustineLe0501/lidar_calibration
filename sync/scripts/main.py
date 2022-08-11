#!/usr/bin/env python
import rospy
import time
import roslib;
import rosbag

from geometry_msgs.msg import *
from sensor_msgs.msg import *

import rospkg
import message_filters

def callback(laserScan, jointState):
    with rosbag.Bag('/home/augustine/sync_ws/src/sync/testing.bag', 'a') as bag:
        bag.write('/rear_sensors/hokuyo/scan', laserScan)
        bag.write('/rear_sensormotor/joint_states', jointState)

def Main():
    rospy.init_node('sync',anonymous=True)

    laser_sub = message_filters.Subscriber('rear_sensors/hokuyo/scan', LaserScan)
    motor_sub = message_filters.Subscriber('/rear_sensormotor/joint_states', JointState)

    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, motor_sub], 10,0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        Main()
    except rospy.ROSInterruptException:
        pass

