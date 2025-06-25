The correct command to control in gazebo via keyboard:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_cont/cmd_vel -p stamped:=true

Things outside the scope of the tutorial:
1. Writing a custom hardware interface
2. Writing a controller plugin for not a diff drive (may be able to use diff drive or may be existing ones out there)
3. Make a controller xacro that uses the controller plugin and lists all the params

Checklist for simulator to real robot:
1. check encoder accuracy (if using them)
2. Check that the wheel radius is accurate (it's like the only thing that needs to be REALLY accurate)
3. Check that the wheel separation is accurate
4. Check sensors work

Setting up a joystick device:
1. Download the drivers
2. ros2 run joy joy_enumerate_devices
3. ros2 run joy joy_node # <-- Run in first terminal
   ros2 topic echo /joy # <-- Run in second terminal
4. OR instead of 3 you can do ros2 run joy_tester test_joy to open a gui
5. Create a joystick.yaml file (will be shown below)
6. Launch a joy_node with the config as joystick.yaml
https://github.com/ros-drivers/joystick_drivers/tree/ros2/joy <-- ROS2 Joystick information
joy_node:
  ros__parameters:
    device_id: 0
    deadzone: 0.05
    autorepeat_rate: 20.0

teleop_node:
  ros__parameters:
    
    axis_linear:  # Left thumb stick vertical
      x: 1
    scale_linear:
      x: 0.5
    scale_linear_turbo:
      x: 1.0

    axis_angular:  # Left thumb stick horizontal
      yaw: 0
    scale_angular:
      yaw: 0.5
    scale_angular_turbo:
      yaw: 1.0

    require_enable_button: true
    enable_button: 6  # Left shoulder button
    enable_turbo_button: 7  # Right shoulder button
End of example yaml

Example teleop node
teleop_node = Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
            )

Advanced teleop stuff:
https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/adv-teleop