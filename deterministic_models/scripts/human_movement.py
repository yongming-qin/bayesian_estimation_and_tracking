#!/usr/bin/env python
"""
Control the movement of the human which is represented by a red color column.
Yongming Qin
2020/04/11
"""
from __future__ import print_function
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
        #TODO rand = np.random.uniform(-2, 2)
        self.set_state("human_column", -2, 0, 0.25, 0, 0, 0)

    def set_velocity(self, model_name, vx, vy, vz):
        rospy.wait_for_service("/gazebo/get_model_state")
        state = self.get_model_state(model_name, "world")
        print("velocity:\t" + str(state.pose.position.x) + "\t\t" + str(state.pose.position.y))

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
            time.sleep(0.1) #QIN
        
if __name__ == "__main__":
    rospy.init_node("human")
    human = Human()
    human.move_forward()