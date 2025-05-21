# ROS 2 Jazzy Dockerfile for robot_simple project
FROM ros:jazzy-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace

# Copy source code
COPY ./src ./src

# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build 

# Create a non-root user
RUN useradd -m robot_user && \
    chown -R robot_user:robot_user /workspace

USER robot_user

# Source the workspace and set entrypoint
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/ros_entrypoint.sh"]
# Launch the robot system node by default
CMD ["bash", "-c", ". /opt/ros/jazzy/setup.bash && . /workspace/install/setup.bash && ros2 launch robot_simple robot_system.launch.py"]
