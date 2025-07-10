#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import json 
import os

json_target_coordinates = []
json_file_path = "/workspace/src/target_coordinates.json"

def callback(msg):
    global json_target_coordinates
    rospy.loginfo("received coordinates from Unity:")
    rospy.loginfo("X: %.2f, Y: %.2f, Z: %.2f", msg.x, msg.y, msg.z)
    print(f"unity coordinates -> X: {msg.x:.2f}, Y: {msg.y:.2f}, Z: {msg.z:.2f}")

    target_coordinate_json_format ={
        "x":float(msg.x),
        "y":float(msg.y),
        "z":float(msg.z)
    }
    json_target_coordinates.append(target_coordinate_json_format)

    save_to_json()

def save_to_json():
    try:
        with open(json_file_path, 'w') as f:
            json.dump(json_target_coordinates, f, indent=2)
        print(f"Coordinates saved to {json_file_path}")
    except Exception as e:
        rospy.logerr(f"Error saving to JSON: {e}")
        print(f"Error saving to JSON: {e}")


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