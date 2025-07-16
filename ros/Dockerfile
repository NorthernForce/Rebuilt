ARG ROS_DISTRO="jazzy"
FROM ros:${ROS_DISTRO}

RUN apt update -y && apt upgrade -y
RUN apt install -y iputils-ping python3-pip
RUN apt install -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-robot-localization

WORKDIR /robot
COPY . .

# TODO: rosdep setup, colcon build step, and start cmd