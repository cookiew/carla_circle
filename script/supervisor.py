#!/usr/bin/env python
# author: mingyuw@stanford.edu

"""
supervisor node, to switch the desired trajectory of entering vehicle from entering route to
circle route
"""

import rospy
import tf
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String

from carla_ros_bridge_msgs.msg import CarlaEgoVehicleControl
from scipy.spatial import KDTree
import numpy as np
import timeit

class odom_state(object):
    def __init__(self):
        self.time = None
        self.x = None
        self.y = None
        self.yaw = None
        self.vx = None
        self.vy = None
        self.speed = 0

    def update_vehicle_state(self, odom_msg):
        self.time = odom_msg.header.stamp.to_sec()
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        ori_quat = (odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w)
        ori_euler = tf.transformations.euler_from_quaternion(ori_quat)
        self.yaw = ori_euler[2]
        self.vx = odom_msg.twist.twist.linear.x
        self.vy = odom_msg.twist.twist.linear.y
        self.speed = np.sqrt(self.vx**2 + self.vy**2) * 0.7 + self.speed * 0.3    # add a basic low pass filter .....

    def get_position(self):
        return [self.x, self.y]

    def get_pose(self):
        return [self.x, self.y, self.yaw]

    def get_velocity(self):
        return [self.vs, self.vy]

    def get_speed(self):
        return self.speed

class SupervisorController:
    def __init__(self):
        rospy.init_node("supervisor", anonymous=True)

        rolename = rospy.get_param("~rolename")

        # state information
        self.stateReady = False
        self.state = odom_state()

        self.current_stage = None


        # subscribers, publishers
        rospy.Subscriber("/carla/" + rolename + "/odometry", Odometry, self.odom_cb)
        # rospy.Subscriber("command/trajectory", MultiDOFJointTrajectory, self.desired_waypoints_cb)
        # self.command_pub = rospy.Publisher("/carla/" + rolename + "/ackermann_cmd", AckermannDrive, queue_size=10)
        self.stage_pub = rospy.Publisher("/carla/" + rolename + "/ctrl_stage", String, queue_size=10)
        self.ctrl_timer = rospy.Timer(rospy.Duration(0.2), self.timer_cb)

    def odom_cb(self, msg):
        self.state.update_vehicle_state(msg)
        if not self.stateReady:
            self.stateReady = True

    def timer_cb(self, event):
        if self.stateReady:
            pos_x, pos_y = self.state.get_position()
            vel = self.state.get_speed()

            if np.linalg.norm([pos_x, pos_y]) < 25 or self.current_stage == "CIRCLE_MODE":
                self.stage_pub.publish(String("CIRCLE_MODE"))
                self.current_stage = "CIRCLE_MODE"
            else:
                self.stage_pub.publish(String("ENT_MODE"))
                self.current_stage = "ENT_MODE"

if __name__ == '__main__':
    try:
        sup = SupervisorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
