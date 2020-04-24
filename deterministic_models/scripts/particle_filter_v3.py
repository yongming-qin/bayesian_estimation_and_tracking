#!/usr/bin/env python
"""
The particle filter to estimate the position of the human walker.
The input of the PF is the position from the color detection and point cloud calculation.
We know the motion model of the human walker.
Yongming Qin
2020/04/17
2020/04/20: v2 Use the code from package filterpy
2020/04/21: v3 Make the robot move
"""
from __future__ import print_function

import time
import rospy
import tf # euler_from_quaternion(quaternion)
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import GetModelState, SetModelState # Ground truth from Gazebo
from gazebo_msgs.msg import ModelState

import numpy as np
from numpy.random import randn
from numpy.random import uniform
import scipy.stats
# from filterpy.monte_carlo import residual_resample # for Particle Fitler resampling
from filterpy.monte_carlo import stratified_resample # for Particle Fitler resampling
from filterpy.monte_carlo import systematic_resample

from matplotlib import pyplot as plt

class ParticleFilter(object):
    def __init__(self, controller, plot_particles=True):
        self.controller = controller
        self.pose_local = np.array([0, 0])
        self.pose_world = np.array([0,0])
        rospy.Subscriber("/color_position", Pose, self.callback)
        self.T = 0.2
        self.N = 2000

        rospy.sleep(2) # wait for messages
        initial_pos = self.measurement_world
        
        # Cretate particles and weights
        particles = self.create_particles(initial_pos, N=self.N)
        weights = np.ones(self.N)/self.N

        if plot_particles:
            fig = plt.figure()
            ax1 = fig.add_subplot(211)
            ax1.set_xlim(-10,10)
            ax1.set_ylim(-1,10)
            ax1.set_title("human position estimation")
            ax1.scatter(-particles[:, 3], particles[:, 2], color='r', marker=',', s=1)

            ax2 = fig.add_subplot(212, autoscale_on=True)
            ax2.set_title("entropy of the particles")
            line1, = plt.plot([],[], 'y-')

        for step in range(10000):
            self.predict(particles)

            mu, var = self.estimate(particles, weights)
            
            if not np.allclose(self.pose_local, [0,0]):
                self.controller.pid(mu[2:4])
            else:
                self.controller.pid([0,0])

            rospy.sleep(self.T)

            self.update(particles, weights, self.measurement_world)

            if plot_particles:
                line1.set_xdata(np.append(line1.get_xdata(), step))
                line1.set_ydata( np.append(line1.get_ydata(), scipy.stats.entropy(weights)) )
            
            indexes = stratified_resample(weights)

            self.resample_from_index(particles, weights, indexes)

            if plot_particles:
                ax1.set_xlim(-10,10)
                ax1.set_ylim(-1,10)
                ax1.set_title("human position estimation" + " \u2b07")
                ax1.scatter(-particles[:, 3], particles[:, 2], color='r', marker=',', s=1)
                
                ax2.relim()
                ax2.autoscale_view()

                plt.pause(0.0001)
                ax1.cla()

            mu, var = self.estimate(particles, weights)
            
            if False:
                print("variables: ", end="")
                gt = self.controller.pos_human
                print(gt, self.measurement_world, mu[2:4])
                print("differences: ", end="")
                print(np.linalg.norm(gt - self.measurement_world), np.linalg.norm(gt - mu[2:4]))

        

    def callback(self, msg):
        self.pose_local[0] = msg.position.x
        self.pose_local[1] = msg.position.y
        self.controller.get_human_odom()
        self.controller.get_turtle_odom()
        self.pose_world[0] = np.cos(self.controller.theta_turtle) * self.pose_local[0] -\
                             np.sin(self.controller.theta_turtle) * self.pose_local[1]
        self.pose_world[1] = np.sin(self.controller.theta_turtle) * self.pose_local[0] +\
                            np.cos(self.controller.theta_turtle) * self.pose_local[1]
        self.measurement_world = self.controller.pos_turtle_world + self.pose_world


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

class Controller(object):
    def __init__(self):
        # Get human's pose using service /gazebo/get_model_state
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.wait_for_service("/gazebo/set_model_state", timeout=2)
        msg_model_state = ModelState()
        msg_model_state.model_name = "turtlebot3_burger"
        msg_model_state.reference_frame = "world"
        msg_model_state.pose.position.x = 0
        msg_model_state.pose.position.y = 0
        msg_model_state.pose.position.z = 0
        msg_model_state.pose.orientation.x = 0
        msg_model_state.pose.orientation.y = 0
        msg_model_state.pose.orientation.z = 0
        msg_model_state.pose.orientation.w = 1

        msg_model_state.twist.linear.x = 0
        msg_model_state.twist.linear.y = 0
        msg_model_state.twist.linear.z = 0
        msg_model_state.twist.angular.x = 0
        msg_model_state.twist.angular.y = 0
        msg_model_state.twist.angular.z = 0
        rospy.wait_for_service("/gazebo/set_model_state", timeout=2)
        res_set = self.set_model_state(msg_model_state)
        print("set success? ", res_set)

        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0

        for _ in range(10):
            self.pub.publish(self.vel)
            rospy.sleep(0.1)
        self.integral = 0
        self.goal_world = np.array([0,0])

    def get_human_odom(self):
        rospy.wait_for_service("/gazebo/get_model_state", timeout=0.05)
        self.state_human = self.get_model_state("human_column", "world")
        self.pos_human = np.array([self.state_human.pose.position.x, self.state_human.pose.position.y])
    
    def get_turtle_odom(self):
        rospy.wait_for_service("/gazebo/get_model_state", timeout=0.05)
        self.state_turtle = self.get_model_state("turtlebot3_burger", "world")
        self.pos_turtle_world = np.array([self.state_turtle.pose.position.x, self.state_turtle.pose.position.y])
        q = self.state_turtle.pose.orientation
        _, _, self.theta_turtle = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def pid(self, goal_local):
        self.get_turtle_odom()
        if not np.allclose(goal_local, np.array([0,0])):
            self.goal_world = goal_local
        goal = self.goal_world - self.pos_turtle_world
        print("goal_world: ", self.goal_world, " goal_local: ", goal_local,
              " pose_turtle_world", self.pos_turtle_world)

        dist = np.linalg.norm(self.goal_world)
        if dist > 0.1:
            theta_d = np.arctan2(goal[1], goal[0]) #QIN sequence
            print("theta: ", self.theta_turtle, " theta_d: ", theta_d)
            
            def angle_dif(direction, orientation):
                dif = direction - orientation
                while (dif > np.pi):
                    dif -= 2*np.pi
                while (dif < -np.pi):
                    dif += 2*np.pi
                return dif
            
            error = angle_dif(theta_d, self.theta_turtle)
            self.integral += error
            if self.integral > 10:
                self.integral = 0
            elif self.integral < -10:
                self.integral = 0
            self.vel.angular.z = 0.3 * error + 0.01 * self.integral
            self.vel.linear.x = 2 * dist
            if (self.vel.linear.x > 0.3):
                self.vel.linear.x = 0.3
        # elif dist > 0.2 and dist <= 0.5:
        #     self.vel.angular.z = 0
        #     self.vel.linear.x = 0
        # else:
        #     self.vel.angular.z = 0
        #     self.vel.linear.x = -0.5

        self.pub.publish(self.vel)






if __name__ == "__main__":
    rospy.init_node("particle_filter")
    controller = Controller()
    pf = ParticleFilter(controller=controller)
