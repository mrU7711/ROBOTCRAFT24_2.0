#!/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

class SquareTest:
    def __init__(self):
        rospy.init_node('square_test', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.front_dist_sub = rospy.Subscriber('/front_distance', Range, self.front_distance_callback)
        self.right_dist_sub = rospy.Subscriber('/right_distance', Range, self.right_distance_callback)
        self.left_dist_sub = rospy.Subscriber('/left_distance', Range, self.left_distance_callback)
        
        self.rate = rospy.Rate(10)  # 10Hz
        self.current_pose = None
        self.side_length = 3  # meters
        self.collision_threshold = 0.15  # meters

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def front_distance_callback(self, msg):
        if msg.range < self.collision_threshold:
            rospy.logwarn("Collision risk! The robot is %.2f meters from an obstacle on the front side", msg.range)

    def right_distance_callback(self, msg):
        if msg.range < self.collision_threshold:
            rospy.logwarn("Collision risk! The robot is %.2f meters from an obstacle on the right side", msg.range)

    def left_distance_callback(self, msg):
        if msg.range < self.collision_threshold:
            rospy.logwarn("Collision risk! The robot is %.2f meters from an obstacle on the left side", msg.range)

    def move_straight(self, distance):
        #rospy.loginfo("moving strainght");
        move_cmd = Twist()
        move_cmd.linear.x = 1.0  # Move forward at 1.0 m/s
        duration = distance / move_cmd.linear.x
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()
        self.cmd_vel_pub.publish(Twist())  # Stop the robot

    def rotate(self, angle):
        #rospy.loginfo("rotating");
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 1.0  # Rotate at 1.0 rad/s
        duration = angle / rotate_cmd.angular.z
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(rotate_cmd)
            self.rate.sleep()
        self.cmd_vel_pub.publish(Twist())  # Stop the robot

    def run(self):
        for _ in range(400):
            self.move_straight(self.side_length)
            self.rotate(1.57)  # 90 degrees in radians

if __name__ == '__main__':
    try:
        square_test = SquareTest()
        square_test.run()
    except rospy.ROSInterruptException:
        pass
