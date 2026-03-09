# me380_driver

ROS2 Humble node that subscribes to `sensor_msgs/msg/JointState` and sends motor commands to the ME380 robot over serial (e.g. to an ESP32 running `driver_firmware`).

## URDF (me380_robot.urdf.xacro)

A 6-DOF arm description is in `urdf/me380_robot.urdf.xacro`, using the same classical DH parameters as `IK_solver.cpp`. Joint names are `joint1`–`joint6` to match the driver.

**Changing joint lengths and offsets:** Edit the properties at the top of the xacro file: `dh_d1`, `dh_d4`, `dh_d6`, `dh_a1`, `dh_a2`.

**Replacing boxes with your STLs:** In each `<link>`, replace the `<visual><geometry><box .../></geometry></visual>` block with a `<mesh filename="package://me380_driver/meshes/linkX.stl" scale="1 1 1"/>` and put STLs in `me380_driver/meshes/`.

## Setup

- **Raspberry Pi**: ROS2 Humble, this package in a colcon workspace.
- **ESP32/Arduino**: Flash `driver_firmware` so it reads the same serial protocol.

Connect Pi and ESP32 via USB. Device is usually `/dev/ttyUSB0` or `/dev/ttyACM0`. Add user to `dialout`: `sudo usermod -aG dialout $USER`.

## Build

```bash
cd ~/ros2_ws
cp -r /path/to/ME380/me380_driver src/
colcon build --packages-select me380_driver
source install/setup.bash
```

## Run

**C++ node (default):**
```bash
ros2 launch me380_driver driver.launch.py
ros2 launch me380_driver driver.launch.py serial_port:=/dev/ttyACM0
```

**Python node:** Requires `pip install pyserial`.
```bash
ros2 launch me380_driver driver.launch.py use_python:=true
ros2 launch me380_driver driver_python.launch.py
```

**Direct run:**
```bash
ros2 run me380_driver driver_node --ros-args -p serial_port:=/dev/ttyUSB0
ros2 run me380_driver driver_node_py.py --ros-args -p serial_port:=/dev/ttyUSB0
```

## Publishing JointState

Example (radians):

```bash
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState \
  "{ header: { frame_id: '' }, name: ['joint1','joint2','joint3','joint4','joint5','gripper'], position: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0] }"
```

Gripper: `position[5] = 0` → open, `position[5] > 0.5` → closed.
