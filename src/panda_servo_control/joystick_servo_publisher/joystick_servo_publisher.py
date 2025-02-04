import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import threading
import time
from enum import Enum
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

# Axis and button mapping
class Axis:
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    RIGHT_STICK_X = 2
    RIGHT_STICK_Y = 3
    D_PAD_X = 4
    D_PAD_Y = 5

class Button:
    A = 2
    B = 1
    X = 3
    Y = 0
    LEFT_BUMPER = 4
    RIGHT_BUMPER = 5
    MENU = 9 # start
    HOME = 8 # select
    LEFT_STICK_CLICK = 9
    RIGHT_STICK_CLICK = 10
    LEFT_TRIGGER = 6
    RIGHT_TRIGGER = 7

class ControllerMode(Enum):
    IDLE = 0
    JOINT = 1
    CARTESIAN = 2

# Configurations
JOY_TOPIC = "/joy"
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
JOINT_TOPIC = "/servo_node/delta_joint_cmds"
EEF_FRAME_ID = "panda_hand"
BASE_FRAME_ID = "panda_link0"
ROBOT_JOINTS = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
AXIS_DEFAULTS = {Button.LEFT_TRIGGER: 0.0, Button.RIGHT_TRIGGER: 0.0} # for xbox controller these values have to be 1.0

def convert_joy_to_cartesian_cmd(axes, buttons, msg : TwistStamped):
    msg.twist.linear.z = axes[Axis.RIGHT_STICK_Y]
    msg.twist.linear.y = axes[Axis.RIGHT_STICK_X]

    # this is necessary for Xbox controller since the triggers are not zeroed when not pressed
    # i don't think it is necessary for other controllers but it doesn't hurt
    lin_x_right = -0.5 * (buttons[Button.RIGHT_TRIGGER] - AXIS_DEFAULTS[Button.RIGHT_TRIGGER])
    lin_x_left = 0.5 * (buttons[Button.LEFT_TRIGGER] - AXIS_DEFAULTS[Button.LEFT_TRIGGER])
    msg.twist.linear.x = lin_x_right + lin_x_left

    msg.twist.angular.y = axes[Axis.LEFT_STICK_Y]
    msg.twist.angular.x = axes[Axis.LEFT_STICK_X]
    
    #msg.twist.angular.z = float(buttons[Button.RIGHT_BUMPER] - buttons[Button.LEFT_BUMPER])
    msg.twist.angular.z = 0.5 * axes[Axis.D_PAD_X]

    return

def convert_joy_to_joint_cmd(axes, buttons, msg : JointJog):
    msg.joint_names = ROBOT_JOINTS
    msg.velocities = [0.0] * 7
    msg.velocities[0] = 0.5 * axes[Axis.LEFT_STICK_X] # Joint 1
    msg.velocities[1] = 0.5 * axes[Axis.LEFT_STICK_Y] # Joint 2
    msg.velocities[2] = 0.5 * axes[Axis.RIGHT_STICK_X] # Joint 3
    msg.velocities[3] = 0.5 * axes[Axis.RIGHT_STICK_Y] # Joint 4
    msg.velocities[4] = 0.5 * axes[Axis.D_PAD_X] # Joint 5
    msg.velocities[5] = 0.5 * axes[Axis.D_PAD_Y] # Joint 6

    joint_7_right = -0.5 * (buttons[Button.RIGHT_TRIGGER] - AXIS_DEFAULTS[Button.RIGHT_TRIGGER])
    joint_7_left = 0.5 * (buttons[Button.LEFT_TRIGGER] - AXIS_DEFAULTS[Button.LEFT_TRIGGER])
    msg.velocities[6] = joint_7_right + joint_7_left # Joint 7
    
    return

class JoyToServoPub(Node):
    mode = ControllerMode.CARTESIAN
    def __init__(self):
        super().__init__("joystick_servo_publisher")
        self.frame_to_publish = BASE_FRAME_ID

        # Initialize subscribers and publishers
        self.joy_sub = self.create_subscription(Joy, JOY_TOPIC, self.joy_cb, 10)
        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self.joint_pub = self.create_publisher(JointJog, JOINT_TOPIC, 10)
        self.collision_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        # Check for the servo node and start it if it is not running
        self.servo_start_client = self.create_client(Trigger, "/servo_node/start_servo")
        self.servo_start_client.wait_for_service(timeout_sec=1.0)
        self.servo_start_client.call_async(Trigger.Request())

        # Start a thread to publish the collision scene (useful for testing, should be disabled on real robot)
        self.collision_pub_thread = threading.Thread(target=self.publish_collision_scene)
        self.collision_pub_thread.start()

    def publish_collision_scene(self):
        time.sleep(3)

        collision_object = CollisionObject()
        collision_object.header.frame_id = BASE_FRAME_ID
        collision_object.id = "box"

        table_1 = SolidPrimitive()
        table_1.type = SolidPrimitive.BOX
        table_1.dimensions = [0.4, 0.6, 0.03]

        table_1_pose = Pose()
        table_1_pose.position.x = 0.6
        table_1_pose.position.y = 0.0
        table_1_pose.position.z = 0.4

        table_2 = SolidPrimitive()
        table_2.type = SolidPrimitive.BOX
        table_2.dimensions = [0.6, 0.4, 0.03]

        table_2_pose = Pose()
        table_2_pose.position.x = 0.0
        table_2_pose.position.y = 0.5
        table_2_pose.position.z = 0.25

        collision_object.primitives = [table_1, table_2]
        collision_object.primitive_poses = [table_1_pose, table_2_pose]
        collision_object.operation = CollisionObject.ADD

        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True

        self.collision_pub.publish(planning_scene)

    def joy_cb(self, msg : Joy):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_to_publish

        joint_msg = JointJog()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = self.frame_to_publish

        # Change mode
        if msg.buttons[Button.Y]:
            self.mode = ControllerMode.IDLE
            self.get_logger().info("Mode changed to: {}".format(self.mode))
            return
        elif msg.buttons[Button.B]:
            self.mode = ControllerMode.JOINT
            self.get_logger().info("Mode changed to: {}".format(self.mode))
            return
        elif msg.buttons[Button.A]:
            self.mode = ControllerMode.CARTESIAN
            self.get_logger().info("Mode changed to: {}".format(self.mode))
            return

        if self.mode == ControllerMode.IDLE: # Do nothing if mode is IDLE
            pass
        elif self.mode == ControllerMode.JOINT: # Control joint velocities if mode is JOINT
            convert_joy_to_joint_cmd(msg.axes, msg.buttons, joint_msg)
            self.joint_pub.publish(joint_msg)
        elif self.mode == ControllerMode.CARTESIAN: # Control linear and angular velocities if mode is CARTESIAN
            convert_joy_to_cartesian_cmd(msg.axes, msg.buttons, twist_msg)
            self.twist_pub.publish(twist_msg)
        
        return

def main(args=None):
    rclpy.init(args=args)
    node = JoyToServoPub()
    
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info("Exiting...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
