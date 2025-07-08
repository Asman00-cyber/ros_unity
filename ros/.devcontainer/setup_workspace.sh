#!/bin/bash
set -e

# Check if ROS TCP Endpoint is already cloned
if [ ! -d "/workspace/src/ROS-TCP-Endpoint" ]; then
    echo "Setting up ROS workspace..."
    cd /workspace/src
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
    
    cd /workspace
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    
    echo "ROS workspace setup complete!"
fi

# Source the workspace
source /workspace/devel/setup.bash