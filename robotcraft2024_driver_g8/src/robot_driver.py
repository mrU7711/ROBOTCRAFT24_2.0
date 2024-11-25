#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import tf
from tf.transformations import quaternion_from_euler

# Publishers for IR sensors
front_ir_pub = None
right_ir_pub = None
left_ir_pub = None

def pose_callback(msg):
    # Publish Odometry message on /odom topic
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"

    # Set the position
    odom.pose.pose.position.x = msg.x
    odom.pose.pose.position.y = msg.y
    odom.pose.pose.position.z = 0.0
    quat = quaternion_from_euler(0, 0, msg.theta)
    odom.pose.pose.orientation.x = quat[0]
    odom.pose.pose.orientation.y = quat[1]
    odom.pose.pose.orientation.z = quat[2]
    odom.pose.pose.orientation.w = quat[3]

    # Set the velocity (optional, zero here)
    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = 0.0
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.angular.z = 0.0

    # Publish the Odometry message
    odom_pub.publish(odom)

    # Broadcast the transform over tf
    odom_trans = tf.TransformBroadcaster()
    odom_trans.sendTransform(
        (msg.x, msg.y, 0.0),
        quat,
        rospy.Time.now(),   
        "base_link",
        "odom"
    )

def ir_front_callback(msg):
    ir_msg = Range()
    ir_msg.header.stamp = rospy.Time.now()
    ir_msg.header.frame_id = "front_ir"
    ir_msg.radiation_type = 1  # Infrared
    ir_msg.field_of_view = 0.034906585  # 2 degrees in radians
    ir_msg.min_range = 0.1
    ir_msg.max_range = 0.8
    ir_msg.range = msg.data
    front_ir_pub.publish(ir_msg)

def ir_right_callback(msg):
    ir_msg = Range()
    ir_msg.header.stamp = rospy.Time.now()
    ir_msg.header.frame_id = "right_ir"
    ir_msg.radiation_type = 1  # Infrared
    ir_msg.field_of_view = 0.034906585  # 2 degrees in radians
    ir_msg.min_range = 0.1
    ir_msg.max_range = 0.8
    ir_msg.range = msg.data
    right_ir_pub.publish(ir_msg)

def ir_left_callback(msg):
    ir_msg = Range()
    ir_msg.header.stamp = rospy.Time.now()
    ir_msg.header.frame_id = "left_ir"
    ir_msg.radiation_type = 1  # Infrared
    ir_msg.field_of_view = 0.034906585  # 2 degrees in radians
    ir_msg.min_range = 0.1
    ir_msg.max_range = 0.8
    ir_msg.range = msg.data
    left_ir_pub.publish(ir_msg)

if __name__ == '__main__':
    rospy.init_node('robot_driver', anonymous=True)

    # Initialize the odom publisher
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)

    # Initialize IR sensor publishers
    front_ir_pub = rospy.Publisher('ir_front_sensor', Range, queue_size=50)
    right_ir_pub = rospy.Publisher('ir_right_sensor', Range, queue_size=50)
    left_ir_pub = rospy.Publisher('ir_left_sensor', Range, queue_size=50)

    # Subscribe to the /pose topic
    rospy.Subscriber('pose', Pose2D, pose_callback)

    # Subscribe to the distance topics
    rospy.Subscriber('front_distance', Range, ir_front_callback)
    rospy.Subscriber('right_distance', Range, ir_right_callback)
    rospy.Subscriber('left_distance', Range, ir_left_callback)

    # Spin to process callbacks
    rospy.spin()
