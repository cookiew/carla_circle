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

class AckermannController:
    def __init__(self):
        rospy.init_node("fancy_controller", anonymous=True)

        # # retrieve ros parameters
        self.time_step = rospy.get_param("~time_step")
        self.traj_steps = rospy.get_param("~plan_steps")
        ctrl_freq = rospy.get_param("~ctrl_freq")
        self.rolename = rospy.get_param("~rolename")
        self.ref_speed = rospy.get_param("~ref_speed")

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
        # self.pid_str_prop = rospy.get_param("~str_prop")
        self.pid_param = rospy.get_param("/controller_pid")
        self.throttle = 0.0


        self.control_stage = None

        # subscribers, publishers
        rospy.Subscriber("/carla/" + self.rolename + "/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("command/trajectory", MultiDOFJointTrajectory, self.desired_waypoints_cb)
        rospy.Subscriber("/carla/" + self.rolename + "/ctrl_stage", String, self.control_stage_cb)
        self.command_pub = rospy.Publisher("/carla/" + self.rolename + "/ackermann_cmd", AckermannDrive, queue_size=10)
        self.vehicle_cmd_pub = rospy.Publisher("/carla/" + self.rolename + "/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=10)
        self.ctrl_timer = rospy.Timer(rospy.Duration(1.0/ctrl_freq), self.timer_cb)

    def control_stage_cb(self, msg):
        self.control_stage = msg

    def desired_waypoints_cb(self, msg):
        if self.rolename == "stanford_ego":
            # only consider is for the entering vehicle
            if self.control_stage == "CIRCLE_MODE":
                print(" her haha is ready? ", self.stateReady,  self.state.get_position())
                radi = 20.0
                if self.stateReady:
                    pos_x, pos_y = self.state.get_position()
                    ang = zeros(shape(self.traj_steps + 1,))
                    ang[0] = np.arctan2(pos_y, pos_x)
                    ang_inc = self.ref_speed * self.time_step/ radi

                    for i in range(self.traj_steps):
                        self.path[i, 0] = radi * np.cos(ang[i])
                        self.path[i, 1] = radi * np.sin(ang[i])
                        self.vel_path[i, 0] = self.ref_speed
                        ang[i + 1] = ang[i] + ang_inc
                    print(rospy.get_name(), "this is path ", self.path)
                    self.path_tree = KDTree(self.path)
                    if not self.pathReady:
                        self.pathReady = True

            else:
                print(" her hahawwwww is ready? ", self.stateReady,  self.state.get_position())
                for i, pt in enumerate(msg.points):
                    self.path[i, 0] = pt.transforms[0].translation.x
                    self.path[i, 1] = pt.transforms[0].translation.y
                    self.vel_path[i, 0] = pt.velocities[0].linear.x
                    self.vel_path[i, 1] = pt.velocities[0].linear.y
                    print(rospy.get_name(), "this is path ", self.path)
                self.path_tree = KDTree(self.path)
                if not self.pathReady:
                    self.pathReady = True
        else:
            print(" her haha is rwwwweady? ", self.stateReady,  self.state.get_position())
            print(" what do i get ", msg.points)
            for i, pt in enumerate(msg.points):
                self.path[i, 0] = pt.transforms[0].translation.x
                self.path[i, 1] = pt.transforms[0].translation.y
                self.vel_path[i, 0] = pt.velocities[0].linear.x
                self.vel_path[i, 1] = pt.velocities[0].linear.y
            print(rospy.get_name(), "this is path ", self.path)
            self.path_tree = KDTree(self.path)
            if not self.pathReady:
                self.pathReady = True



    def odom_cb(self, msg):
        self.state.update_vehicle_state(msg)
        if not self.stateReady:
            self.stateReady = True

    def timer_cb(self, event):
        if self.pathReady and self.stateReady:
            pos_x, pos_y = self.state.get_position()

            # pick target w/o collision avoidance, closest point on the traj and one point ahead
            _, idx = self.path_tree.query([pos_x, pos_y])
            if idx < self.traj_steps - 1:
                target_pt = self.path_tree.data[idx + 1, :]
                target_vel = self.vel_path[idx + 1, :]
            else:
                target_pt = self.path_tree.data[-1, :]
                target_vel = self.vel_path[-1, :]
                print("CONTROLLER: at the end of the desired waypoits!!!")

            target_speed = np.linalg.norm(target_vel)
            steer = self.compute_ackermann_cmd(target_pt)

            # control values for CarlaEgoVehicleControl
            vehicle_cmd_msg = CarlaEgoVehicleControl()
            vehicle_cmd_msg.steer = -steer
            # print("target %2.2f"% target_speed, " ego %2.2f"%self.state.get_speed())
            if self.state.get_speed() - target_speed > 0.2:
                print(rospy.get_name(), " in braking mode ")
                vehicle_cmd_msg.throttle = 0
                vehicle_cmd_msg.brake = np.clip((self.state.get_speed() - target_speed) * self.pid_param["brake_prop"], \
                                                0., 1.)
            elif self.state.get_speed() - target_speed > 0.0:
                print(rospy.get_name(), " in less throttle mode ")
                vehicle_cmd_msg.throttle = self.pid_param["throttle_base"] \
                                            + (target_speed - self.state.get_speed()) * 1
            elif target_speed - self.state.get_speed() > 0.1:
                # print(rospy.get_name(), " in acceleration  mode ")
                vehicle_cmd_msg.throttle = np.clip(self.pid_param["throttle_base"] + \
                                            (target_speed - self.state.get_speed()) * self.pid_param["speed_prop"], \
                                            0., 1.)
            else:
                # print(" apply forward base throttlr")
                vehicle_cmd_msg.throttle = self.pid_param["throttle_base"]

            self.vehicle_cmd_pub.publish(vehicle_cmd_msg)
        # self.command_pub.publish(cmd_msg)


    def compute_ackermann_cmd(self, target_pt):
        pos_x, pos_y, yaw = self.state.get_pose()

        egoOri = np.array([np.cos(yaw), np.sin(yaw), 0])
        rel_pos = np.array([target_pt[0] - pos_x, target_pt[1] - pos_y, 0])
        rel_pos_norm = np.linalg.norm(rel_pos)

        rel_pos_unit = rel_pos / np.linalg.norm(rel_pos)
        rot = np.cross(rel_pos_unit, egoOri)
        steer = -rot[2] * self.pid_param["steer_prop"]

        self.steer_cache = steer
        return steer


if __name__ == '__main__':
    try:
        controller = AckermannController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
