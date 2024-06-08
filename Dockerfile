FROM ros:noetic-ros-core

ARG DEBIAN_FRONTEND=noninteractive

# Install apt packages
RUN apt-get update && apt-get install -y \
    git \
    nano \
    tmux \
    terminator

RUN apt-get update && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update --fix-missing  && apt-get install -y \
    build-essential \
    cmake \
    python3-catkin-tools \
    python3-osrf-pycommon  \
    ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
    ros-noetic-dynamixel-sdk \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rosbridge-suite \
    ros-noetic-web-video-server \
    ros-noetic-image-transport-plugins

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Create ROS workspace
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /root/catkin_ws; catkin_make"

# Add ROS setup to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set environment variables for ROS
ENV ROS_PACKAGE_PATH=/root/catkin_ws/src:$ROS_PACKAGE_PATH

# Launch web_video_server
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch web_video_server web_video_server.launch"]
