# Ulurover Robotical Arm

5/6 DoF robotical arm implementation on ROS2 for Ulurover. This README clarify the progress of the arm.

## Packages

- **panda_description:** URDF export of robotical arm.
- **panda_moveit_config:** Launch files and controllers of robotical arm which was generated using moveit_servo.
- **moveit_servo:** Fork of moveit_servo package for ROS2 Humble.
- **panda_servo_control:** Joy/keyboard teleop controller for the robotical arm.

## To-Do List

- [x] Generate moveit_config for Panda robotical arm using Moveit2 setup assistant.
- [x] Be able to plan & execute robotical ar0m poses.
- [x] Be able to adjust poses/trajectories using C++.
- [ ] Use moveit servo messages to manage robotical arm realtime.
    - [x] There will be three modes: IDLE, POSE, TRAJECTORY
    - [x] Keyboard control.
    - [ ] Joystick control.
    - [x] The latency should be too low between the key press and Rviz simulation.
    - [ ] Gripper control.
- [x] Test robo arm controls with obstacles.
- [ ] Gazebo simulations for the tasks.
- [ ] Create hardware communication flow using micro-ROS over UART.
- [ ] Integrate real robotical arm of Ulurover to test with hardware.
- [ ] Integrate [IKFast](https://moveit.picknik.ai/humble/doc/examples/ikfast/ikfast_tutorial.html) kinematics solver plugin.
- [ ] Create PWM motor drivers for STM32.
- [ ] Manufacturing and testing of robotical arm.
    - [ ] Manufacturing part.
    - [ ] Test robotical arm axis simply using pots.
- [ ] Integrate servo messages to MCU to operate robotical arm same as simulation.

## Tasks for Arm

 Collaboration Mission is the main mission for robotical arm. Here are the tasks for Anatoilan Rover Challange:

- Disconnect cables and bring them back to the base.
- Press the numpad buttons using the gripper of the arm.
- Install the fuel pipe.
- Patch base using hook and loop tape.
- Brush using the gripper.
- Mount antenna.
- Weld the antenna (LOL).
