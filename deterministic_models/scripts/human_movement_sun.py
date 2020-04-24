#!/usr/bin/env python
"""
Control the movement of the human which is represented by a red color column.
Yongming Qin, Xiaoshan Sun
2020/04/11
"""
from __future__ import print_function
import numpy as np
import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import time

class Human(object):
    def __init__(self):
        # Get human's pose using service /gazebo/get_model_state
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # Set human's pose using service /gazebo/set_model_state
        #   The velocity will continue on the model object.
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.T = 0.5
        self.initialize_human_start_position()

    def set_state(self, model_name, px, py, pz, vx, vy, vz):
        msg_model_state = ModelState()
        msg_model_state.model_name = model_name
        msg_model_state.reference_frame = "world"
        msg_model_state.pose.position.x = px
        msg_model_state.pose.position.y = py
        msg_model_state.pose.position.z = pz
        msg_model_state.pose.orientation.x = 0
        msg_model_state.pose.orientation.y = 0
        msg_model_state.pose.orientation.z = 0
        msg_model_state.pose.orientation.w = 1

        msg_model_state.twist.linear.x = vx
        msg_model_state.twist.linear.y = vy
        msg_model_state.twist.linear.z = vz
        msg_model_state.twist.angular.x = 0
        msg_model_state.twist.angular.y = 0
        msg_model_state.twist.angular.z = 0

        rospy.wait_for_service("/gazebo/set_model_state", timeout=2)
        res_set = self.set_model_state(msg_model_state)

    def initialize_human_start_position(self):
        # use random start position in x-y plane
        init_x = np.random.uniform(4, 5)
        init_y = np.random.uniform(-1, 1)
        self.set_state("human_column", init_x, init_y, 0.25, 0, 0, 0)
        # self.set_state("human_column", -2, 0, 0.25, 0, 0, 0)

    def set_velocity(self, model_name, vx, vy, vz):
        rospy.wait_for_service("/gazebo/get_model_state")
        state = self.get_model_state(model_name, "world")
        # print("position:\t" + str(state.pose.position.x) + "\t\t" + str(state.pose.position.y))
        # print("velocity:\t" + str(state.twist.linear.x) + "\t\t" + str(state.twist.linear.y))

        msg_model_state = ModelState()
        msg_model_state.model_name = model_name
        msg_model_state.reference_frame = "world"
        msg_model_state.pose = state.pose
        msg_model_state.twist.linear.x = vx
        msg_model_state.twist.linear.y = vy
        msg_model_state.twist.linear.z = vz
        self.set_model_state(msg_model_state)
        
    # An example movement. Constant velocity
    def move_forward(self):
        while not rospy.is_shutdown():
            self.set_velocity("human_column", 0.2, 0, 0)
            time.sleep(self.T) #QIN
        
    def move_deterministic(self):
        tmp_velocity = np.random.uniform(0, 0.3)
        tmp_angle = np.random.uniform(0, np.pi)
        tmp_angular_velocity = 0.2 * np.random.uniform(-np.pi, np.pi)

        while not rospy.is_shutdown():
            vx = tmp_velocity * np.cos(tmp_angle)
            vy = tmp_velocity * np.sin(tmp_angle)
            self.set_velocity("human_column", vx, vy, 0)
            # update angle
            tmp_angle += self.T * tmp_angular_velocity
            time.sleep(self.T) #QIN

    def move_probablistic(self):
        tmp_velocity = np.random.uniform(0, 0.3)
        tmp_angle = np.random.uniform(0, np.pi)
        tmp_angular_velocity = np.random.uniform(-0.15, 0.15)

        while not rospy.is_shutdown():
            print("tmp_velocity: ", tmp_velocity, "tmp_angular_vel: ", tmp_angular_velocity,\
                  " tmp_angle: ", tmp_angle)
            vx = tmp_velocity * np.cos(tmp_angle)
            vy = tmp_velocity * np.sin(tmp_angle)
            self.set_velocity("human_column", vx, vy, 0)
            # update velocity
            uncertainty_velo = np.random.normal(0, 0.02)
            tmp_velocity += uncertainty_velo
            tmp_velocity = self.truncate(tmp_velocity, 0, 0.2)
            # update angular velocity
            uncertainty_angu = np.random.normal(0, 0.001 * np.pi)
            tmp_angular_velocity += uncertainty_angu
            tmp_angular_velocity = self.truncate(tmp_angular_velocity, -0.15, 0.15)
            # update angle
            tmp_angle += self.T * tmp_angular_velocity
            tmp_angle %= 2*np.pi
            time.sleep(self.T) #QIN

    def truncate(self, x, min, max):
        if (x < min):
            return min
        if (x > max):
            return max
        return x


if __name__ == "__main__":
    rospy.init_node("human")
    human = Human()
    # human.move_forward()
    # human.move_deterministic()
    human.move_probablistic()
