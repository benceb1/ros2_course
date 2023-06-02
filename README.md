# TurtleBot Project in ROS2

## About

The TurtleController class is a ROS2 node that controls the movement of a turtlebot. It subscribes to topics such as laser scan (/scan), IMU (/imu), and odometry (/odom), and publishes twist commands (/cmd_vel) to control the turtlebot's velocity and rotation.

The class provides three main methods:

    go_straight(speed, distance): Moves the turtlebot straight at a given speed for a specified distance. It calculates the time required to reach the destination and publishes twist messages accordingly. It also considers laser scan data to avoid obstacles while moving.

    turn(omega, angle): Rotates the turtlebot with a specified angular velocity (omega) for a given angle. It calculates the time required to complete the rotation and publishes twist messages accordingly.

    stop(): Stops the turtlebot by publishing a twist message with zero velocities.

The main() function initializes the ROS2 node, creates an instance of TurtleController, and demonstrates the usage of the go_straight() and turn() methods. Finally, it cleans up and shuts down the node.

Please note that this code assumes it is being used in a ROS2 environment and relies on specific message types (LaserScan, Imu, Odometry, and Twist) and topics (/scan, /imu, /odom, and /cmd_vel). Make sure to provide the appropriate message types and topic names based on your specific ROS2 setup.

## Usage
(The description is made for an already working and installed system)

### In first terminal
1. Set up turtlebot model
```
$ export TURTLEBOT3_MODEL=burger
```
2. Set up Gazebo model path
```
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
prefix turtlebot3_gazebo \
`/share/turtlebot3_gazebo/models/
```
3. Launch Gazebo with simulation world
```
$ ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### In second terminal

Set up turtlebot model
```
$ export TURTLEBOT3_MODEL=burger
```

How to *build* and use the package.

    cd ~/ros2_ws
    colcon build --symlink-install

Run our program
```
$ ros2 run ros2_course turtlesim_controller
```
˙
