"# ROS-Unity Communication" 


Unity side:
You need to install the ROS TCP Connector package from Unity-Technologies!

I created two game objects one named Publish and the other called RosConnector 
-> Ros Connector contains the ROSConnection script from the ROS TCP Connector package responsible for binding to the ROS noetic server
-> Publish object contains a script made from scratch called CoordinatesPublisher.cs which is responsible for sending points msg coordinates from the clicked points of the DICOM images
-> On the SliceClickSwitcher.cs i initialized a variable:

public CoordinatesPublisher voxelpublisher; // call the coordinate publisher class
    // Store clicked pixel coordinates and slice indices for each plane

I also included these commands :
        if(voxelpublisher!=null)
        {
            voxelpublisher.PublishCoordinates(finalCoordinate);
        }
        else
        {
            Debug.LogWarning($"Coordinate publisher not set in SliceClickSwitcher\n");
        }
which are responsible for taking the finalcoordinates and publish them to the ROS noetic container environment


ROS side:

Once you clone the ros part of this repo , some packages will donwload automatically. One of this is the ROS-TCP-Endpoint from Unity Technologies responsible for setting the ROS noetic as a subscriber(server)

In order to open the ROS as a server you need to open three bash terminals and run these commands in this specific order:

Terminal 1:
source /opt/ros/noetic/setup.bash
roscore

Terminal 2:
source /workspace/devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000

Terminal 3:
source /workspace/devel/setup.bash
cd /workspace/src/receiver/scripts
python3 receiver.py

I made a receiver.py python code in order to set the Ros as a server and acquire the x,y,z coordinates clicked from Unity 




IMPORTANT: in order to start the whole communication process you first need to run the Ros side and then click play on the Unity side to click the specific coordinates of the DICOM image!



