FROM osrf/ros:humble-desktop-full-jammy
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ackermann-msgs ros-humble-gazebo-ros-pkgs ros-humble-rviz2 \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros \
    python3-pytest python3-yaml python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
