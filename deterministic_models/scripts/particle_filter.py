#!/usr/bin/env python
"""
The particle filter to estimate the position of the human walker.
The input of the PF is the position from the color detection and point cloud calculation.
We know the motion model of the human walker.
Yongming Qin
2020/04/17
"""
from __future__ import print_function
import numpy as np
import math
from numpy import linalg as la
from numpy.random import random
import scipy.stats # pdf
import time # time.sleep()
# from filterpy.monte_carlo import residual_resample # for Particle Fitler resampling
# from filterpy.monte_carlo import stratified_resample # for Particle Fitler resampling


import rospy
import tf # euler_from_quaternion(quaternion)
from geometry_msgs.msg import Pose

# Ground truth from Gazebo
from gazebo_msgs.srv import GetModelState



class ParticleFilter(object):
    def __init__(self):
        ground_truth = GroundTruth()
        rospy.Subscriber("/color_position", Pose, self.callback)
        pub = rospy.Publisher("pose_estimation", Pose, queue_size=1)
        self.pose = Pose()
        self.pose_pre = Pose()
        self.pose_estimation = Pose()
        self.T = 0.5
        self.N_PARTICLES = 1000

        self.init_variables()

        while not rospy.is_shutdown():
            # Prediction
            for i in range(self.N_PARTICLES):
                self.s[i] = self.s_pre[i] + np.random.normal(0, 0.01)
                self.omega[i] = self.omega_pre[i] + np.random.normal(0, 0.01 * np.pi)
                self.x[i] = self.x_pre[i] + self.s_pre[i] * math.cos(self.theta_pre[i]) * self.T
                self.y[i] = self.y_pre[i] + self.s_pre[i] * math.sin(self.theta_pre[i]) * self.T
                self.theta[i] = self.theta_pre[i] + self.omega_pre[i] * self.T
            time.sleep(self.T)

            # Correction
            z_pose = self.pose # Observation
            ground_truth.get_human_odom() # Ground truth
            
            for i in range(self.N_PARTICLES):
                prob_x = scipy.stats.norm(0,0.1).pdf(z_pose.position.x - self.x[i])
                prob_y = scipy.stats.norm(0,0.1).pdf(z_pose.position.y - self.y[i])
                self.prob[i] = prob_x * prob_y
                # print(prob_x, prob_y)
                # Resample
            # indices = residual_resample(self.prob)
            # print(self.prob[0:5])
            total_prob = sum(self.prob)
            if (total_prob < 0.1):
                self.init_variables()
                print("--------Restart--------------")
                continue
            indices = self.stratified_resample(self.prob/total_prob)
            print("Finished one step Correction.")

            for i in range(self.N_PARTICLES):
                self.s_pre[i] = self.s[indices[i]]
                self.omega_pre[i] = self.omega[indices[i]]
                self.x_pre[i] = self.x[indices[i]]
                self.y_pre[i] = self.y[indices[i]]
                self.theta_pre[i] = self.theta[indices[i]]

            estimation_x = sum(self.x_pre) / self.N_PARTICLES
            estimation_y = sum(self.y_pre) / self.N_PARTICLES
            self.pose_estimation.position.x = estimation_x
            self.pose_estimation.position.y = estimation_y
            pub.publish(self.pose_estimation)
            
            print("variables: ", end="")
            print(ground_truth.x, ground_truth.y, z_pose.position.x, z_pose.position.y,\
                  estimation_x, estimation_y)
            print("differences: ", end="")
            print(la.norm([ground_truth.x - z_pose.position.x, ground_truth.y - z_pose.position.y]),\
            la.norm([ground_truth.x - estimation_x, ground_truth.y - estimation_y]))


    def callback(self, msg):
        self.pose = msg

    def init_variables(self):
        time.sleep(self.T)
        self.pose_pre = self.pose

        # initiate the state variable at step 0
        time.sleep(self.T) # Another measurement
        vec_0 = [self.pose.position.x - self.pose_pre.position.x,\
                 self.pose.position.y - self.pose_pre.position.y]
        print("first observation: ", end="")
        print(self.pose_pre.position.x, self.pose_pre.position.y,\
              self.pose.position.x, self.pose.position.y)
        while (not rospy.is_shutdown()) and la.norm(vec_0) < 0.05:
            print("Human walker is standing still. Waiting to estimate the heading direction.")
            self.pose_pre = self.pose
            time.sleep(self.T) # Another measurement
            vec_0 = [self.pose.position.x - self.pose_pre.position.x,\
                     self.pose.position.y - self.pose_pre.position.y]
            print("second observation: ", end="")
            print(self.pose_pre.position.x, self.pose_pre.position.y,\
                  self.pose.position.x, self.pose.position.y)

        s_0 = la.norm(vec_0) / self.T
        omega_0 = 0
        x_0 = self.pose.position.x
        y_0 = self.pose.position.y
        theta_0 = math.atan2(vec_0[0], vec_0[1])

        # state variables of previous step
        self.s_pre = np.full(self.N_PARTICLES, s_0)
        self.omega_pre = np.full(self.N_PARTICLES, omega_0)
        self.x_pre = np.full(self.N_PARTICLES, x_0)
        self.y_pre = np.full(self.N_PARTICLES, y_0)
        self.theta_pre = np.full(self.N_PARTICLES, theta_0)
        self.prob = np.full(self.N_PARTICLES, 0.0)

        self.s = np.copy(self.s_pre)
        self.omega = np.copy(self.omega_pre)
        self.x = np.copy(self.x_pre)
        self.y = np.copy(self.y_pre)
        self.theta = np.copy(self.theta_pre)

    def stratified_resample(self, weights):
        """ Performs the stratified resampling algorithm used by particle filters.

        This algorithms aims to make selections relatively uniformly across the
        particles. It divides the cumulative sum of the weights into N equal
        divisions, and then selects one particle randomly from each division. This
        guarantees that each sample is between 0 and 2/N apart.

        Parameters
        ----------
        weights : list-like of float
            list of weights as floats

        Returns
        -------

        indexes : ndarray of ints
            array of indexes into the weights defining the resample. i.e. the
            index of the zeroth resample is indexes[0], etc.
        """

        N = len(weights)
        # make N subdivisions, and chose a random position within each one
        positions = (random(N) + range(N)) / N

        indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(weights)
        i, j = 0, 0
        while i < N:
            # print("i: %d, j: %d" % (i, j))
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        return indexes
        
 


class GroundTruth(object):
    def __init__(self):
        # Get human's pose using service /gazebo/get_model_state
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def get_human_odom(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        self.state = self.get_model_state("human_column", "world")
        self.x = self.state.pose.position.x
        self.y = self.state.pose.position.y





if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
