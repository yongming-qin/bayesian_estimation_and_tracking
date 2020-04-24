#!/usr/bin/env python
"""
The particle filter to estimate the position of the human walker.
The input of the PF is the position from the color detection and point cloud calculation.
We know the motion model of the human walker.
Yongming Qin
2020/04/17
2020/04/20: v2 Use the code from package filterpy
"""
from __future__ import print_function

import rospy
import tf # euler_from_quaternion(quaternion)
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState # Ground truth from Gazebo

import numpy as np
from numpy.random import randn
from numpy.random import uniform
import scipy.stats
# from filterpy.monte_carlo import residual_resample # for Particle Fitler resampling
from filterpy.monte_carlo import stratified_resample # for Particle Fitler resampling
from filterpy.monte_carlo import systematic_resample

from matplotlib import pyplot as plt

class ParticleFilter(object):
    def __init__(self, plot_particles=True):
        self.ground_truth = GroundTruth()
        self.pose = Pose()
        self.pose_estimation = Pose()
        rospy.Subscriber("/color_position", Pose, self.callback)
        pub = rospy.Publisher("pose_estimation", Pose, queue_size=1)
        self.T = 0.2
        self.N = 2000

        rospy.sleep(2) # wait for messages
        initial_pos = [self.pose.position.x, self.pose.position.y]
        

        # Cretate particles and weights
        particles = self.create_particles(initial_pos, N=self.N)
        weights = np.ones(self.N)/self.N

        if plot_particles:
            alpha = .02
            if self.N > 5000:
                alpha *= np.sqrt(5000)/np.sqrt(self.N)
            plt.scatter(particles[:20,3], particles[:20,2], alpha=alpha, color='g')
        xs = []

        plt.ion()
        for _ in range(10000):
            self.predict(particles)

            rospy.sleep(self.T)

            measurement = np.array([self.pose.position.x, self.pose.position.y])

            # plt.axis([-10, 10, -1, 10])
            # plt.scatter(particles[:, 2], particles[:, 3], color='k', marker=',', s=1)
            # plt.draw()
            # plt.pause(0.1)
            # plt.clf()
            self.update(particles, weights, measurement)
            

            indexes = stratified_resample(weights)
            self.resample_from_index(particles, weights, indexes)

            plt.axis([-10, 10, -1, 10])
            plt.scatter(-particles[:, 3], particles[:, 2], color='r', marker=',', s=1)
            plt.draw()
            plt.pause(0.001)
            plt.clf()

            # if self.neff(weights) < self.N/2:
            #     indexes = systematic_resample(weights)
            #     self.resample_from_index(particles, weights, indexes)
            #     assert np.allclose(weights, 1.0/self.N)
            mu, var = self.estimate(particles, weights)
            xs.append(mu)

            if True:
                print("variables: ", end="")
                gt = np.array([self.ground_truth.x, self.ground_truth.y])
                print(gt, measurement, mu[2:4])
                print("differences: ", end="")
                print(np.linalg.norm(gt - measurement), np.linalg.norm(gt - mu[2:4]))

            # if plot_particles:
                
                
            
            # p1 = plt.scatter(self.ground_truth.x, self.ground_truth.y, marker='+', color='k', s=180, lw=3)
            # p2 = plt.scatter(mu[0], mu[1], marker='s', color='r')

        # xs = np.array(xs)
        # plt.plot(xs[:, 0], xs[:, 1])
        # plt.show()
        

    def callback(self, msg):
        self.pose = msg
        self.ground_truth.get_human_odom()

    def create_particles(self, initial_pos, N):
        particles = np.empty((N,5))
        particles[:,0] = uniform(0, 0.3, size=N)
        particles[:,1] = uniform(-0.15, 0.15, size=N)
        particles[:,2] = initial_pos[0] + (randn(N) * 0.3)
        particles[:,3] = initial_pos[1] + (randn(N) * 0.3)
        particles[:,4] = uniform(0, np.pi, size=N)
        return particles

    def predict(self, particles, std=[1, 0.01*np.pi]):
        """ Move according to control input u (heading change, veclocity)
        with noise Q (std heading change, std velocity)"""
        N = len(particles)
        particles[:, 2] += particles[:, 0] * np.cos(particles[:, 1]) * self.T
        particles[:, 3] += particles[:, 0] * np.sin(particles[:, 1]) * self.T
        particles[:, 4] += particles[:, 1] * self.T
        particles[:, 4] %= 2 * np.pi

        # next speed
        particles[:, 0] += randn(N) * std[0]
        # next angular speed
        particles[:, 1] += randn(N) * std[1]
        
    def update(self, particles, weights, measurement):
        distance = np.linalg.norm(particles[:, 2:4] - measurement, axis=1)
        weights *= scipy.stats.norm.pdf(distance, 0, 1)

        weights += 1.e-300
        weights /= np.sum(weights)
    
    def neff(self, weights):
        return 1. / np.sum(np.square(weights))

    def resample_from_index(self, particles, weights, indexes):
        particles[:] = particles[indexes]
        weights[:] = weights[indexes]
        weights.fill(1.0 / len(weights))

    def estimate(self, particles, weights):
        """ Return mean and variance of the weighted particles."""
        pos = particles[:,:]
        mean = np.average(pos, weights=weights, axis=0)
        var = np.average( (pos - mean)**2, weights=weights, axis=0)
        return mean, var



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
