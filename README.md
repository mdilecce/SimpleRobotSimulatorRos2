# robot_simple ROS 2 Project


## Simple Robot Simulator ROS2

This project simulates a robot moving in a 2D space using ROS 2. It consists of two main nodes:

- **Robot Node**: Handles the robot's movement, position, and kinematics in the simulated environment.
- **Battery Node**: Simulates the robot's battery, including energy consumption, charging, and status reporting.

## Techniques Used

- **ROS 2 Components**: Both nodes are implemented as composable components for modularity.
- **Action Servers/Clients**: Used for long-running tasks such as moving to a target position and charging the battery.
      For the battery while chargin the server check constantly the position of the robot if it is in the charging base
- **Service Servers/Clients**: For checking and requesting energy from the battery node.
- **Parameter Event Handlers**: Dynamic parameter updates (e.g., robot base position, physical parameters) at runtime.
- **TF2**: For managing and broadcasting coordinate transforms between the robot and world frames.
- **Timers**: Periodic publishing of robot and battery status.


## Running with Docker

This project uses ROS 2 Jazzy and can be run in a Docker container for easy setup.

### Build the Docker image

```bash
docker build -t robot_simple:v01 .
```

### Run the Docker container with two terminals

Open two terminals and run the following:

#### Terminal 1: Start Node and See ROS 2 logs (rclcpp logger output)

```bash
docker run --rm -it --tag robot_simple robot_simple:v01
```
#### Terminal 2: Move the robot (send actions)

In a second terminal, attach to the running container and use send ROS2 CLI commands. 

First attach to the container a new terminal:

```bash
docker exec -it robot_simple bash
```

Once inside the container, initalize the bash:
```bash
source ./install/setup.bash
```

 you can move the robot using:

```bash
ros2 action send_goal /move_to_target robot_interfaces/action/MoveToTarget "{target_position_x: 20.0, target_position_y: 0.0}"
```


- Terminal 1 is for monitoring the ROS 2 logger output in real time (using ros2 log echo).
- Terminal 2 is for sending actions to move the robot.

You can open more terminals with `docker exec -it robot_simple bash` to interact with the running container.

---

## Notes
- Make sure your host user has permission to access Docker.
- The `--network host` option is used for ROS 2 discovery. If you are on Mac/Windows, you may need to adjust networking.
- The workspace is mounted with `-v $(pwd)/src:/workspace/src` for live code editing.
