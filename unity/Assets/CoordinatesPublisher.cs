using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class CoordinatesPublisher : MonoBehaviour
{
    ROSConnection ros; // intialize a ROSConnection type variable
    public string topicName = "/voxel_coordinate"; // the topic name is used as the "folder name" which contains the data to be sent. To this is where the ROS binds

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance(); // create a ROS TCP protocol for the communicaton
        ros.RegisterPublisher<PointMsg>(topicName); // registers the publisher to bind to the topic name
    }

    public void PublishCoordinates(Vector3Int coordinates) // function which holds the coordinates to be sent to ROS
    {
        PointMsg msg = new PointMsg //msg of class type PointMsg(Only accepts three floating point values , x,y,z suitable for our situtation)
        {
            x = coordinates.x,
            y = coordinates.y,
            z = coordinates.z
        };

        ros.Publish(topicName, msg); // send the message containing the data over to ROS 
        Debug.Log($"Published coordinates to ROS: {coordinates}"); // debugg print 
    }
}
