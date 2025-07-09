"# ROS-Unity Communication"



## Unity side:

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



Debug.LogWarning($"Coordinate publisher not set in SliceClickSwitcher\\n");



}



which are responsible for taking the finalcoordinates and publish them to the ROS noetic container environment



## ROS side:

Step 1: Open VS code and open the folder ros of this repository.



Step 2: Then on the terminal type: docker compose up



Step 3: Now the server is running on your docker desktop. Click on the left side of VS code and click Attach to an existing container



Step 4: Install the ros-tcp-endpoint package from Unity -Technologies by following this series of commands:


cd /workspace/src



git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git



cd /workspace



rosdep update



rosdep install --from-paths src --ignore-src -r -y



catkin\_make



source devel/setup.bash



rospack find ros\_tcp\_endpoint





Step 5: Open the three terminals



Terminal 1:



source /opt/ros/noetic/setup.bash



roscore



Terminal 2:



source /workspace/devel/setup.bash



roslaunch ros\_tcp\_endpoint endpoint.launch tcp\_ip:=127.0.0.1 tcp\_port:=10000



Terminal 3:



source /workspace/devel/setup.bash



cd /workspace/src/receiver/scripts



python3 receiver.py





I made a receiver.py python code in order to set the Ros as a server and acquire the x,y,z coordinates clicked from Unity





IMPORTANT: in order to start the whole communication process you first need to run the Ros side and then the Unity side.





