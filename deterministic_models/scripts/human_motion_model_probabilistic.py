#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import Marker


def move():
    pub = rospy.Publisher("human", Marker, queue_size=1)
    rospy.init_node("human", anonymous=True)
    T = 1 # seconds
    rate = rospy.Rate(int(1/T))

    mu, sigma = 1.0, 0.2 # mean and standard deviation
    action_observed = [ [1, 0], [1, 0], [0, -1], [0, -1], [1, 0], [1, 0], [1, 0], [-1, 0] ]
    x_human = 0; y_human = 0;

    count = 0

    def pub_marker(x, y):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.ARROW
        marker.action = marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0; marker.scale.y = 0.2; marker.scale.z = 0.2

        marker.color.r = 0.0; marker.color.g = 1.0;
        marker.color.b = 0.0; marker.color.a = 1.0

        pub.publish(marker)

    pub_marker(x_human, y_human)
    rate.sleep()
    pub_marker(x_human, y_human)
    rate.sleep()
    for action in action_observed:
        rospy.loginfo("moving " + str(action))
        speed = np.random.normal(mu,sigma)
        T = T * speed
        x_human += action[0] * T; y_human += action[1] * T;
        pub_marker(x_human, y_human)
        count += 1
        rate.sleep()
        raw_input("Press Enter to continue...")

if __name__ == "__main__":
    try:
        move()
    except rospy.ROSInterruptException:
        pass