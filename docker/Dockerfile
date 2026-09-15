# ROS 2 Jazzy (Ubuntu 24.04) development image for smart_charge_robot.
#
# Gazebo (ros-gz) is intentionally NOT installed: the target host has 1.9 GB
# RAM / 2 vCPUs / no display, which cannot host Gazebo Harmonic + Nav2 + RViz2.
# The project therefore uses a minimal Nav2-compatible simulator fallback
# (smart_charge_simulation). See docs/architecture.md for the decision record.
FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-tf-transformations \
    ros-jazzy-robot-localization \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-storage-default-plugins \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ubuntu/ws
