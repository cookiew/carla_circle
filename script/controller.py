#!/usr/bin/env python
# author: mingyuw@stanford.edu

"""
given desired waypoints, this ros node sends the ackermann steering command for
the ego vehicle to follow the trajectory
TODO: send acceleration instead of speed directly
"""

import rospy
import tf
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from ackermann_msgs.msg import AckermannDrive

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
        self.speed = None

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
        self.speed = np.sqrt(self.vx**2 + self.vy**2)
        # print("this is the current speed", self.speed)

    def get_position(self):
        return [self.x, self.y]

    def get_pose(self):
        return [self.x, self.y, self.yaw]

    def get_velocity(self):
        return [self.vs, self.vy]

    def get_speed(self):
        return self.speed

class AckermannController:
    MAC_ACCELERATION = 1.0
    def __init__(self):
        rospy.init_node("controller", anonymous=True)

        # # retrieve ros parameters
        self.time_step = rospy.get_param("~time_step")
        self.traj_steps = rospy.get_param("~plan_steps")
        ctrl_freq = rospy.get_param("~ctrl_freq")

        # state information
        self.stateReady = False
        self.state = odom_state()

        # path tracking information
        self.pathReady = False
        self.path = np.zeros(shape=(self.traj_steps, 2))   # absolute position
        self.path_tree = KDTree(self.path)
        self.vel_path = np.zeros(shape=(self.traj_steps, 2))

        # # PID controller parameter
        self.pid_str_prop = rospy.get_param("~str_prop")

        # subscribers, publishers
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("/carla/ego_vehicle/desired_waypoints", MultiDOFJointTrajectory, self.desired_waypoints_cb)
        self.command_pub = rospy.Publisher("/carla/ego_vehicle/ackermann_cmd", AckermannDrive, queue_size=10)
        self.ctrl_timer = rospy.Timer(rospy.Duration(1.0/ctrl_freq), self.timer_cb)

    def desired_waypoints_cb(self, msg):

        for i, pt in enumerate(msg.points):
            self.path[i, 0] = pt.transforms[0].translation.x
            self.path[i, 1] = pt.transforms[0].translation.y
            self.vel_path[i, 0] = pt.velocities[0].linear.x
            self.vel_path[i, 1] = pt.velocities[0].linear.y

        self.path_tree = KDTree(self.path)
        if not self.pathReady:
            self.pathReady = True


    def odom_cb(self, msg):
        self.state.update_vehicle_state(msg)
        if not self.stateReady:
            self.stateReady = True

    def timer_cb(self, event):
        cmd_msg = AckermannDrive()

        # cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.steering_angle = 0
        cmd_msg.steering_angle_velocity = 0
        cmd_msg.speed = 0
        cmd_msg.acceleration = 0
        cmd_msg.jerk = 0

        if self.pathReady and self.stateReady:
            pos_x, pos_y = self.state.get_position()

            # pick target w/o collision avoidance, closest point on the traj and one point ahead
            _, idx = self.path_tree.query([pos_x, pos_y])
            if idx < self.traj_steps - 2:
                target_pt = self.path_tree.data[idx + 2, :]
                target_vel = self.vel_path[idx + 2, :]
            else:
                target_pt = self.path_tree.data[1, :]
                target_vel = self.vel_path[1, :]
                # print("at the end of the desired waypoits!!!")

            target_speed = np.linalg.norm(target_vel)

            steer = self.compute_ackermann_cmd(target_pt)
            cmd_msg.steering_angle = steer
            cmd_msg.speed = target_speed
            if self.state.get_speed() - target_speed > 1.0:
                cmd_msg.acceleration = (target_speed - self.state.get_speed()) / (self.time_step * 2)
            elif target_speed - self.state.get_speed() > 1.0:
                cmd_msg.acceleration = np.min([3, (target_speed - self.state.get_speed()) / (self.time_step * 2)])
            else:
                cmd_msg.acceleration = 0
        self.command_pub.publish(cmd_msg)

    def compute_ackermann_cmd(self, target_pt):
        pos_x, pos_y, yaw = self.state.get_pose()
        if np.linalg.norm([target_pt[0] - pos_x, target_pt[1] - pos_y]) < 1:
            print("target point too close!!!!!!!!")
            return 0.2

        egoOri = np.array([np.cos(yaw), np.sin(yaw), 0])
        rel_pos = np.array([target_pt[0] - pos_x, target_pt[1] - pos_y, 0])
        rel_pos_norm = np.linalg.norm(rel_pos)

        rel_pos_unit = rel_pos / np.linalg.norm(rel_pos)
        rot = np.cross(rel_pos_unit, egoOri)
        steer = -rot[2] * self.pid_str_prop

        return steer


if __name__ == '__main__':
    try:
        controller = AckermannController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass