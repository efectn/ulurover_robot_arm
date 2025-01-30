# Ulurover Robotical Arm

5/6 DoF robotical arm implementation on ROS2 for Ulurover. This README clarify the progress of the arm.

## (Planned) Packages

- **panda_description:** URDF export of robotical arm.
- **panda_moveit_config:** Launch files and controllers of robotical arm which was generated using moveit_servo.
- **panda_servo_controller:** Joy/keyboard teleop controller of the robotical arm.

## To-Do List

- [x] Generate moveit_config for Panda robotical arm using Moveit2 setup assistant.
- [x] Be able to plan & execute robotical ar0m poses.
- [x] Be able to adjust poses/trajectories using C++.
- [ ] Use moveit servo messages to manage robotical arm realtime.
    - [ ] There will be three modes: IDLE, POSE, TRAJECTORY
    - [ ] The latency should be too low.
- [ ] Create hardware communication flow using micro-ROS over UART.
- [ ] Integrate real robotical arm of Ulurover to test with hardware.
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
