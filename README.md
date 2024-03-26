# dynamixel_control

The [`ros2_control`](https://github.com/ros-controls/ros2_control) implementation for any kind of [ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/) robots.

- `dynamixel_hardware`: the [`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp) implementation for the multiple ROBOTIS Dynamixel servos.
- `open_manipulator_x_description`: the reference implementation of the `ros2_control` robot using [ROBOTIS OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/).

The `dynamixel_hardware` package is hopefully compatible any configuration of ROBOTIS Dynamixel servos thanks to the `ros2_control`'s flexible architecture.

## Set up

First [install ROS 2 Humble on Ubuntu 22.04](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Then follow the instruction below.

```shell
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-plotjuggler-ros
pip3 install openpyxl
pip3 install roboticstoolbox-python==0.11.0
pip3 install spatialgeometry==0.2.0 && pip3 install spatialmath-python==0.11
pip3 install colored==1.4.2
mkdir -p ~/ros/humble && cd ~/ros/humble
git clone https://github.com/SamKaiYang/dynamixel_hardware_control.git src
vcs import src < src/dynamixel_hardware_control/dynamixel_control.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. install/setup.bash
```
## Read Dynamixel motor 

H54-200-S500-R Goal Torque unit 16.11328 [mA]
H54-100-S500-R Goal Torque unit 16.11328 [mA]
H42-20-S300-R Goal Torque unit 4.02832 [mA]

# TODO fixed at dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp
```  
float DynamixelWorkbench::convertValue2Current(uint8_t id, int16_t value)
{
  float current = 0;
  float CURRENT_UNIT = 2.69f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102

  model_info = getModelInfo(id);
  if (model_info == NULL) return false;
  CURRENT_UNIT = 16.11328f;
  // current = (int16_t)value * CURRENT_UNIT;
  // return current;
  // if (getProtocolVersion() == 1.0f)
  // {
  //   current = (int16_t)value * CURRENT_UNIT;
  //   return current;
  // }
  // else if (getProtocolVersion() == 2.0f)
  // {
  //   if (strncmp(getModelName(id), "PRO-L", strlen("PRO-L")) == 0 ||
  //       strncmp(getModelName(id), "PRO-M", strlen("PRO-M")) == 0 ||
  //       strncmp(getModelName(id), "PRO-H", strlen("PRO-H")) == 0)
  //   {
  //     CURRENT_UNIT = 16.11328f;
  //     current = (int16_t)value * CURRENT_UNIT;
  //     return current;
  //   }
  //   else if (strncmp(getModelName(id), "PRO-PLUS", strlen("PRO-PLUS")) == 0)
  //   {
  //     CURRENT_UNIT = 1.0f;
  //     current = (int16_t)value * CURRENT_UNIT;
  //     return current;
  //   }
  //   else
  //   {
  //     CURRENT_UNIT = 16.11328f;
  //     current = (int16_t)value * CURRENT_UNIT;
  //     return current;
  //   }
  // }

  current = (int16_t)value * CURRENT_UNIT;

  return current;
}

float DynamixelWorkbench::convertValue2Current(int16_t value)
{
  float current = 0;
  const float CURRENT_UNIT = 16.11328f; //Unit : mA, Ref : http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#goal-current102
  // CURRENT_UNIT = 16.11328f;
  current = (int16_t)value * CURRENT_UNIT;

  return current;
}
```
## Demo with real robot

## Video can see:
MoveIt:https://www.youtube.com/watch?v=tgAdRwSPZqU
Trajectory:https://www.youtube.com/watch?v=KQFqeoA4rBA&feature=youtu.be

## confirm USB
```
ls /dev/tty*
sudo chmod 777 /dev/ttyUSB0
```

### Demo MoveIt 
```
ros2 launch single_arm_6dof_moveit_config demo.launch.py 
```
### Demo trajectory
```
# open terminal 1
ros2 launch single_arm_6dof_description single_arm_6dof.launch.py 

# open terminal 2
ros2 launch my_package traj_action.launch.py 
```
### Configure Dynamixel motor parameters

Update the `usb_port`, `baud_rate`, and `joint_ids` parameters on [`open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro`](https://github.com/youtalk/dynamixel_control/blob/main/open_manipulator_x_description/urdf/open_manipulator_x.ros2_control.xacro#L9-L12) to correctly communicate with Dynamixel motors.
The `use_dummy` parameter is required if you don't have a real OpenManipulator-X.

Note that `joint_ids` parameters must be splited by `,`.

```xml
<hardware>
  <plugin>dynamixel_hardware/DynamixelHardware</plugin>
  <param name="usb_port">/dev/ttyUSB0</param>
  <param name="baud_rate">1000000</param>
  <!-- <param name="use_dummy">true</param> -->
</hardware>
```
