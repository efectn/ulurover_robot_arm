import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from enum import Enum
import termios, sys
import tty, select
import signal
from rclpy.parameter import Parameter as Param

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

3: Increase joint 1 angle (JOINT mode)
4: Decrease joint 2 angle (JOINT mode)
5: Increase joint 3 angle (JOINT mode)
6: Decrease joint 4 angle (JOINT mode)
7: Increase joint 5 angle (JOINT mode)
8: Decrease joint 6 angle (JOINT mode)
9: Increase joint 7 angle (JOINT mode)
v: reverse the direction of the joint (JOINT mode)

q: Quit the program
"""

# Arm specific parameters
alpha = 0.5
frame_id = 'panda_link0'
robot_joints = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

key_mapping_cartesian = {
    'w': [alpha, 0.0, 0.0],
    's': [-alpha, 0.0, 0.0],
    'a': [0.0, alpha, 0.0],
    'd': [0.0, -alpha, 0.0],
    'r': [0.0, 0.0, alpha],
    'f': [0.0, 0.0, -alpha],
}

key_mapping_joint = {
    '3': [alpha, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    '4': [0.0, alpha, 0.0, 0.0, 0.0, 0.0, 0.0],
    '5': [0.0, 0.0, alpha, 0.0, 0.0, 0.0, 0.0],
    '6': [0.0, 0.0, 0.0, alpha, 0.0, 0.0, 0.0],
    '7': [0.0, 0.0, 0.0, 0.0, alpha, 0.0, 0.0],
    '8': [0.0, 0.0, 0.0, 0.0, 0.0, alpha, 0.0],
    '9': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, alpha],
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

class ServoControl(Node):
    mode = ControllerMode.CARTESIAN # Default mode
    joint_multiplier = 1.0 # Default multiplier for joint control

    def change_mode(self, mode : ControllerMode):
        self.mode = mode
        self.get_logger().info('Mode changed to: {}'.format(mode))     

    def log_debug(self, msg):
        if self.debug:
            self.get_logger().info(msg)   

    def __init__(self):
        super().__init__('servo_keyboard',                     
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        # Check parameters
        self.debug = bool(self.get_parameter_or('debug', Param('debug', rclpy.Parameter.Type.BOOL, False)).value)

        # Create publishers
        self.cartesian_servo_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_servo_publisher = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)

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
                twistMsg.header.frame_id = frame_id

                jointMsg = JointJog()
                jointMsg.header.stamp = self.get_clock().now().to_msg()
                jointMsg.header.frame_id = frame_id

                sendTwistMsg = False
                sendJointMsg = False

                # Handle key press
                key = get_key(settings)
                if key == '0':
                    self.change_mode(ControllerMode.IDLE)
                elif key == '1':
                    self.change_mode(ControllerMode.JOINT)
                elif key == '2':
                    self.change_mode(ControllerMode.CARTESIAN)
                elif key == 'v':
                    self.joint_multiplier *= -1
                    self.log_debug('Joint multiplier changed to: {}'.format(self.joint_multiplier))
                elif key == 'q' or key == '\x03':
                    break
                elif key in key_mapping_cartesian and self.mode == ControllerMode.CARTESIAN:
                    twistMsg.twist.linear.x = key_mapping_cartesian[key][0]
                    twistMsg.twist.linear.y = key_mapping_cartesian[key][1]
                    twistMsg.twist.linear.z = key_mapping_cartesian[key][2]
                    sendTwistMsg = True
                elif key in key_mapping_joint and self.mode == ControllerMode.JOINT:
                    jointMsg.velocities = [x * self.joint_multiplier for x in key_mapping_joint[key]]
                    jointMsg.joint_names = robot_joints
                    sendJointMsg = True
                else:
                    if key != '':
                        self.log_debug("Invalid key: {}".format(key))
                
                # Send relavanet message
                if self.mode == ControllerMode.CARTESIAN and sendTwistMsg:
                    self.cartesian_servo_publisher.publish(twistMsg)
                    self.log_debug('Publishing: {}'.format(twistMsg.twist))
                elif self.mode == ControllerMode.JOINT and sendJointMsg:
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
