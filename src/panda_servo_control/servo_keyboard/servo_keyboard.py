import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from enum import Enum
import termios, sys
import tty, select
import signal
from rclpy.parameter import Parameter as Param
from rclpy.action import ActionClient

# Welcome message
msg = """
Teleop control for the robotical arm. You can control the robot in 3 modes:
0: IDLE
1: JOINT
2: CARTESIAN

Use the following keys to control the robot:
0: Change mode to IDLE
1: Change mode to JOINT
2: Change mode to CARTESIAN
w: Move forward in x direction (CARTESIAN mode)
s: Move backward in x direction (CARTESIAN mode)
a: Move left in y direction (CARTESIAN mode)
d: Move right in y direction (CARTESIAN mode)
r: Move up in z direction (CARTESIAN mode)
f: Move down in z direction (CARTESIAN mode)
l: Change velocity control mode (LINEAR/ANGULAR)

3: Increase joint 1 angle (JOINT mode)
4: Decrease joint 2 angle (JOINT mode)
5: Increase joint 3 angle (JOINT mode)
6: Decrease joint 4 angle (JOINT mode)
7: Increase joint 5 angle (JOINT mode)
8: Decrease joint 6 angle (JOINT mode)
9: Increase joint 7 angle (JOINT mode)
v: reverse the direction of the joint (JOINT mode)

g: Close the gripper
h: Open the gripper

q: Quit the program
"""

# Arm specific parameters
ALPHA = 0.5
GRIPPER_MAX = 0.05
FRAME_ID = 'panda_link0'
ROBOT_JOINTS = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

KEY_MAPPING_CARTESIAN = {
    'w': [ALPHA, 0.0, 0.0],
    's': [-ALPHA, 0.0, 0.0],
    'a': [0.0, ALPHA, 0.0],
    'd': [0.0, -ALPHA, 0.0],
    'r': [0.0, 0.0, ALPHA],
    'f': [0.0, 0.0, -ALPHA],
}

KEY_MAPPING_JOINT = {
    '3': [ALPHA, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    '4': [0.0, ALPHA, 0.0, 0.0, 0.0, 0.0, 0.0],
    '5': [0.0, 0.0, ALPHA, 0.0, 0.0, 0.0, 0.0],
    '6': [0.0, 0.0, 0.0, ALPHA, 0.0, 0.0, 0.0],
    '7': [0.0, 0.0, 0.0, 0.0, ALPHA, 0.0, 0.0],
    '8': [0.0, 0.0, 0.0, 0.0, 0.0, ALPHA, 0.0],
    '9': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ALPHA],
}

# From https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Controller modes
class ControllerMode(Enum):
    IDLE = 0
    JOINT = 1
    CARTESIAN = 2

# Velocity control modes
class VelocityControl(Enum):
    LINEAR = 0
    ANGULAR = 1

class ServoControl(Node):
    controller_mode = ControllerMode.CARTESIAN # Default mode
    vel_mode = VelocityControl.LINEAR # Default velocity control mode
    joint_multiplier = 1.0 # Default multiplier for joint control
    gripper = 0.0

    def change_controller_mode(self, mode : ControllerMode):
        self.controller_mode = mode
        self.get_logger().info('Mode changed to: {}'.format(mode))  

    def change_vel_mode(self):
        self.vel_mode = VelocityControl((self.vel_mode.value+1)%2)
        self.get_logger().info('Velocity control mode changed to: {}'.format(self.vel_mode))   

    def log_debug(self, msg):
        if self.debug:
            self.get_logger().info(msg)

    def send_gripper_cmd(self, process : list[2], goal : GripperCommand.Goal):
        val_pos = 0.0005 * (process[0])
        val_neg = -0.0005 * (process[1])
        self.gripper += val_pos + val_neg
        if self.gripper >= GRIPPER_MAX or self.gripper <= 0.0:
            return

        goal.command.position = self.gripper
        goal.command.max_effort = 50.0 # arbitrary value
        self.gripper_action.send_goal_async(goal)

        return

    def __init__(self):
        super().__init__('servo_keyboard',                     
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        # Check parameters
        self.debug = bool(self.get_parameter_or('debug', Param('debug', rclpy.Parameter.Type.BOOL, False)).value)

        # Create publishers
        self.cartesian_servo_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_servo_publisher = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)

        # Initialize action clients
        self.gripper_action = ActionClient(self, GripperCommand, "/panda_hand_controller/gripper_cmd")
        gripper_srv_status = self.gripper_action.wait_for_server(timeout_sec=5.0)
        if not gripper_srv_status:
            self.get_logger().error("Gripper action server not available")
            raise RuntimeError("Gripper action server not available")

        # Print info message
        self.get_logger().info(msg)

    def stop(self):
        self.get_logger().info("Exiting...")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def run(self):
        signal.signal(signal.SIGINT, self.stop)
        
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                twistMsg = TwistStamped()
                twistMsg.header.stamp = self.get_clock().now().to_msg()
                twistMsg.header.frame_id = FRAME_ID

                jointMsg = JointJog()
                jointMsg.header.stamp = self.get_clock().now().to_msg()
                jointMsg.header.frame_id = FRAME_ID

                sendTwistMsg = False
                sendJointMsg = False

                # Handle key press
                key = get_key(settings)
                if key == '0':
                    self.change_controller_mode(ControllerMode.IDLE)
                elif key == '1':
                    self.change_controller_mode(ControllerMode.JOINT)
                elif key == '2':
                    self.change_controller_mode(ControllerMode.CARTESIAN)
                elif key == 'l':
                    self.change_vel_mode()
                elif key == 'v':
                    self.joint_multiplier *= -1
                    self.get_logger().info('Joint multiplier changed to: {}'.format(self.joint_multiplier))
                elif key == 'q' or key == '\x03':
                    break
                elif key == 'g' or key == 'h':
                    goal = GripperCommand.Goal()
                    self.send_gripper_cmd([key == 'h', key == 'g'], goal)
                elif key in KEY_MAPPING_CARTESIAN and self.controller_mode == ControllerMode.CARTESIAN:
                    vectors : Vector3 = twistMsg.twist.linear
                    if self.vel_mode == VelocityControl.ANGULAR:
                        vectors = twistMsg.twist.angular

                    vectors.x = KEY_MAPPING_CARTESIAN[key][0]
                    vectors.y = KEY_MAPPING_CARTESIAN[key][1]
                    vectors.z = KEY_MAPPING_CARTESIAN[key][2]
                    sendTwistMsg = True
                elif key in KEY_MAPPING_JOINT and self.controller_mode == ControllerMode.JOINT:
                    jointMsg.velocities = [x * self.joint_multiplier for x in KEY_MAPPING_JOINT[key]]
                    jointMsg.joint_names = ROBOT_JOINTS
                    sendJointMsg = True
                else:
                    if key != '':
                        self.log_debug("Invalid key: {}".format(key))
                
                # Send relavanet message
                if self.controller_mode == ControllerMode.CARTESIAN and sendTwistMsg:
                    self.cartesian_servo_publisher.publish(twistMsg)
                    self.log_debug('Publishing: {}'.format(twistMsg.twist))
                elif self.controller_mode == ControllerMode.JOINT and sendJointMsg:
                    self.joint_servo_publisher.publish(jointMsg)
                    self.log_debug('Publishing: {}'.format(jointMsg.velocities))
        except Exception as e:
            self.get_logger().error('Error: {}'.format(e))
        finally:
            self.stop()
    
def main(args=None):
    rclpy.init(args=args)
    node = ServoControl()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
