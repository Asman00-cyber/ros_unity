# ROS-Unity Communication
This project facilitates communication between a Unity-based UI and a ROS Noetic server using the ROS TCP Connector.

## Unity Side
**Installation**:

- You need to install the ROS TCP Connector package from Unity-Technologies via the Unity Package Manager.


**Scene Setup**

- I created three game objects in the hierarchy:

    **RosConnector**: Contains the ROSConnection script responsible for binding to the ROS Noetic server.

    IP Address: 127.0.0.1

    Port: 10000

    **Publish**: Contains a custom script CoordinatesPublisher.cs. This is responsible for sending point coordinates derived from clicked DICOM images.

    **ReceiveTrajectory**: Contains a script responsible for receiving planned trajectories back from ROS.





**Script Modifications** (BiopsyClickManager.cs)

The following additions were made to the BiopsyClickManager class:

- **Variables**:


    public CoordinatesPublisher coordinatesPublisher; 

    public Vector3? xClick; // Changed from private to public

    public Vector3? yClick; // Changed from private to public

    public Vector3? zClick; // Changed from private to public

- **Functions**: 
    The PublishPointsToROS() function was added after DrawLine() and is called within the Update() function:
```
void PublishPointsToROS()
{
    if (coordinatesPublisher != null)
    {
        coordinatesPublisher.PublishPoints();
        Debug.Log("Points published to ROS!");
    }
    else
    {
        Debug.LogWarning("Coordinates publisher not set!");
    }
}
```

## ROS Side
**Setup Instructions** 

1. Open VS Code: Open the folder ros/ros-devcontainer-vscode from this repository.

2. Docker: Open Docker Desktop.

3. Launch Container: In the terminal, run: docker compose up.

4. Attach Container: In VS Code, click "Attach to an existing container" and select workspace-1.

5. Install ROS-TCP-ENDPOINT: (Skip if already in src)

    Bash

    cd /workspace/src

    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

    cd /workspace

    rosdep update

    
    rosdep install --from-paths src --ignore-src -r -y
    
    catkin_make
    
    source devel/setup.bash

    rospack find ros_tcp_endpoint

**Receiver Script**:

A **receiver.py** script (located in /src/scripts) acts as the server to acquire coordinates from Unity and sends planned trajectories from MoveIT back to the Unity UI.

**Execution**

Open three separate terminals and run the following:

- Terminal 1 (Master):

    source /opt/ros/noetic/setup.bash

    roscore


- Terminal 2 (Endpoint):

    source /workspace/devel/setup.bash

    roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10000

- Terminal 3 (Receiver):


    source /workspace/devel/setup.bash

    cd /workspace/src/scripts

    python3 receiver.py


**Important Configuration Notes**

*Startup Order*: For the process to work properly, you must start the ROS side first, followed by the Unity side.

*Port Forwarding*: In the docker-compose.yml file within the ros-devcontainer-vscode folder, ensure the ports section under workspace includes 10000:10000.