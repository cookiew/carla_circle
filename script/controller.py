#!/usr/bin/env python
# author: mingyuw@stanford.edu

"""
given desired waypoints, this ros node sends the ackermann steering command for
the ego vehicle to follow the trajectory
"""

import rospy
import tf
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from ackermann_msgs.msg import AckermannDrive

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
        self.speed = np.sqrt(self.vx**2 + self.vy**2)

    def get_position(self):
        return [self.x, self.y]

    def get_pose(self):
        return [self.x, self.y, self.yaw]

    def get_velocity(self):
        return [self.vs, self.vy]

    def get_speed(self):
        return self.speed

class AckermannController:
    def __init__(self):
        rospy.init_node("controller", anonymous=True)

        # # retrieve ros parameters
        self.time_step = rospy.get_param("~time_step")
        self.traj_steps = rospy.get_param("~plan_steps")
        ctrl_freq = rospy.get_param("~ctrl_freq")
        rolename = rospy.get_param("~rolename")

        # state information
        self.stateReady = False
        self.state = odom_state()

        # path tracking information
        self.pathReady = False
        self.path = np.zeros(shape=(self.traj_steps, 2))   # absolute position
        self.path_tree = KDTree(self.path)
        self.vel_path = np.zeros(shape=(self.traj_steps, 2))

        self.steer_cache = None

        # # PID controller parameter
        self.pid_str_prop = rospy.get_param("~str_prop")

        # subscribers, publishers
        rospy.Subscriber("/carla/" + rolename + "/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("command/trajectory", MultiDOFJointTrajectory, self.desired_waypoints_cb)
        self.command_pub = rospy.Publisher("/carla/" + rolename + "/ackermann_cmd", AckermannDrive, queue_size=10)
        self.vehicle_cmd_pub = rospy.Publisher("/carla/" + rolename + "/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=10)
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
                # str_idx = idx + 2
            else:
                target_pt = self.path_tree.data[-1, :]
                target_vel = self.vel_path[-1, :]
                # str_idx = self.vel_path.shape[0] - 1
                print("CONTROLLER: at the end of the desired waypoits!!!")

            target_speed = np.linalg.norm(target_vel)


            # set control values for ackermann_cmd
            # str_target = target_pt
            # while (np.linalg.norm([str_target[0] - pos_x, str_target[1] - pos_y]) < 3) and str_idx < self.traj_steps - 1:
            #     str_idx += 1
            #     str_target = self.path_tree.data[str_idx, :]
            #     print(" str target changed from vel target!!!!!!!!!!!!", str_target, "    ", target_pt)
            steer = self.compute_ackermann_cmd(target_pt)
            cmd_msg.steering_angle = steer
            cmd_msg.speed = target_speed
            if self.state.get_speed() - target_speed > 0.0:
                cmd_msg.acceleration = (target_speed - self.state.get_speed()) / (self.time_step * 2) - 5
            elif target_speed - self.state.get_speed() > 0.2:
                cmd_msg.acceleration = np.min([3, (target_speed - self.state.get_speed()) / (self.time_step * 2)])
            else:
                cmd_msg.acceleration = 0

            # control values for CarlaEgoVehicleControl
            # vehicle_cmd_msg.steer = -steer
            # # print("target %2.2f"% target_speed, " ego %2.2f"%self.state.get_speed())
            # if self.state.get_speed() - target_speed > 1.0:
            #     # print(" in braking mode ")
            #     vehicle_cmd_msg.throttle = 0
            #     vehicle_cmd_msg.brake = 1
            # elif self.state.get_speed() - target_speed > 0.0:
            #     # print(" in less throttle mode ")
            #     vehicle_cmd_msg.throttle = 0.57 +  (target_speed - self.state.get_speed()) * 1
            # elif target_speed - self.state.get_speed() > 0.2:
            #     # print(" in acceleration  mode ")
            #     vehicle_cmd_msg.throttle = np.clip(0.57 + (target_speed - self.state.get_speed()) * 0.3, 0, 1)
            # else:
            #     # print(" apply forward base throttlr")
            #     vehicle_cmd_msg.throttle = 0.57

        # self.vehicle_cmd_pub.publish(vehicle_cmd_msg)
        self.command_pub.publish(cmd_msg)


    def compute_ackermann_cmd(self, target_pt):
        pos_x, pos_y, yaw = self.state.get_pose()
        # if np.linalg.norm([target_pt[0] - pos_x, target_pt[1] - pos_y]) < 1:
        #     print("target point too close!!!!!!!!")
        #     if self.steer_cache:
        #         return self.steer_cache
        #     else:
        #         return 0.0

        egoOri = np.array([np.cos(yaw), np.sin(yaw), 0])
        rel_pos = np.array([target_pt[0] - pos_x, target_pt[1] - pos_y, 0])
        rel_pos_norm = np.linalg.norm(rel_pos)

        rel_pos_unit = rel_pos / np.linalg.norm(rel_pos)
        rot = np.cross(rel_pos_unit, egoOri)
        steer = -rot[2] * self.pid_str_prop

        self.steer_cache = steer
        return steer


if __name__ == '__main__':
    try:
        controller = AckermannController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
