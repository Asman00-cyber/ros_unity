#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def callback(msg):
    print("asd")
    rospy.loginfo("received coordinates from Unity:")
    rospy.loginfo("X: %.2f, Y: %.2f, Z: %.2f", msg.x, msg.y, msg.z)
    print(f"unity coordinates -> X: {msg.x:.2f}, Y: {msg.y:.2f}, Z: {msg.z:.2f}")

def listener():
    # Initialize the ROS node
    rospy.init_node('listener', anonymous=True)
    
    # Create subscriber for the topic that Unity will publish to
    rospy.Subscriber('/voxel_coordinate', Point,callback)
    
    rospy.loginfo("Coordinate listener started. Waiting for Unity coordinates...")
    print("=== ROS Coordinate Listener Active ===")
    print("Listening for coordinates on topic: /voxel_coordinate")
    print("Waiting for Unity to send data...")
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Coordinate listener node terminated.")
        print("Coordinate listener stopped.")