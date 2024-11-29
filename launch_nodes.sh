#!/bin/bash

# Launch gcode_manager in a new terminal
gnome-terminal -- bash -c "ros2 run gcode_serial gcode_manager --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=115200; exec bash"

# Wait for the user to prepare the printer
echo "Press Enter once the printer is prepared..."
read -p ""

# Launch gh_subscriber in a new terminal
gnome-terminal -- bash -c "ros2 run gh_ros gh_subscriber; exec bash"

# Launch ros_to_gh in a new terminal
gnome-terminal -- bash -c "ros2 run gh_ros ros_to_gh; exec bash"

# Launch rosbridge_server in a new terminal
gnome-terminal -- bash -c "ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash"

# Send Home command so everything is fine
echo "Press Enter to send the G28 command to /grasshopper_input topic..."
read -p ""
ros2 topic pub --once /grasshopper_input std_msgs/String '{"data": "G28"}'
