import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import threading
import time
from enum import Enum
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

class ControllerMode(Enum):
    IDLE = 0
    JOINT = 1
    CARTESIAN = 2

class JoyToServoPub(Node):
    def axis_deadzone(self, axis_value):
        deadzone = self.get_gamepad_property("deadzone")
        if abs(axis_value) < deadzone:
            return 0.0
        return axis_value

    def get_btn_axis(self, msg : Joy, name : str):
        func_type = self.get_parameter(f'gamepad.functions.{name}.type').value
        func_index = self.get_parameter(f'gamepad.functions.{name}.index').value

        if func_type == "axis":
            return self.axis_deadzone(msg.axes[func_index])
        elif func_type == "button":
            return msg.buttons[func_index]
        else:
            return None
    
    def get_default_btn_axis(self, name : str):
        return self.get_parameter(f'gamepad.functions.{name}.default').value
    
    def get_topic(self, name : str):
        return self.get_parameter(f'topics.{name}').value
    
    def get_robot_property(self, name : str):
        return self.get_parameter(f'robot.{name}').value
    
    def get_gamepad_property(self, name : str):
        return self.get_parameter(f'gamepad.{name}').value
        
    def convert_joy_to_cartesian_cmd(self, joy_msg : Joy, msg : TwistStamped):
        msg.twist.linear.z = self.get_btn_axis(joy_msg, "cartesian_z_axis_linear")
        msg.twist.linear.y = self.get_btn_axis(joy_msg, "cartesian_y_axis_linear")

        # this is necessary for Xbox controller since the triggers are not zeroed when not pressed
        # i don't think it is necessary for other controllers but it doesn't hurt
        lin_x_right = -self.get_gamepad_property("velocity_multiplier") * (self.get_btn_axis(joy_msg, "cartesian_x_axis_linear_right") - self.get_default_btn_axis("cartesian_x_axis_linear_right"))
        lin_x_left = self.get_gamepad_property("velocity_multiplier") * (self.get_btn_axis(joy_msg, "cartesian_x_axis_linear_left") - self.get_default_btn_axis("cartesian_x_axis_linear_left"))
        msg.twist.linear.x = lin_x_right + lin_x_left

        msg.twist.angular.y = self.get_btn_axis(joy_msg, "cartesian_x_axis_angular")
        msg.twist.angular.x = self.get_btn_axis(joy_msg, "cartesian_y_axis_angular")
    
        #msg.twist.angular.z = float(buttons[Button.RIGHT_BUMPER] - buttons[Button.LEFT_BUMPER])
        msg.twist.angular.z = self.get_gamepad_property("velocity_multiplier") * self.get_btn_axis(joy_msg, "cartesian_z_axis_angular")

        return

    def convert_joy_to_joint_cmd(self, joy_msg : Joy, msg : JointJog):
        msg.joint_names = self.get_robot_property("joint_names")
        msg.velocities = [0.0] * 7
        msg.velocities[0] = self.get_gamepad_property("velocity_multiplier") * self.get_btn_axis(joy_msg, "joint_1") # Joint 1
        msg.velocities[1] = self.get_gamepad_property("velocity_multiplier") * self.get_btn_axis(joy_msg, "joint_2") # Joint 2
        msg.velocities[2] = self.get_gamepad_property("velocity_multiplier") * self.get_btn_axis(joy_msg, "joint_3") # Joint 3
        msg.velocities[3] = self.get_gamepad_property("velocity_multiplier") * self.get_btn_axis(joy_msg, "joint_4") # Joint 4
        msg.velocities[4] = self.get_gamepad_property("velocity_multiplier") * self.get_btn_axis(joy_msg, "joint_5") # Joint 5
        msg.velocities[5] = self.get_gamepad_property("velocity_multiplier") * self.get_btn_axis(joy_msg, "joint_6") # Joint 6

        joint_7_right = -self.get_gamepad_property("velocity_multiplier") * (self.get_btn_axis(joy_msg, "joint_7_right") - self.get_default_btn_axis("joint_7_right"))
        joint_7_left = self.get_gamepad_property("velocity_multiplier") * (self.get_btn_axis(joy_msg, "joint_7_left") - self.get_default_btn_axis("joint_7_left"))
        msg.velocities[6] = joint_7_right + joint_7_left # Joint 7
    
        return
    
    def init_params(self):
        # Declare topics
        self.declare_parameters(
            namespace='',
            parameters=[
                ('topics.joy_topic', '/joy'),
                ('topics.planning_scene_topic', '/planning_scene'),
                ('topics.gripper_cmd_topic', '/panda_hand_controller/gripper_cmd'),
                ('topics.servo_start_topic', '/servo_node/start_servo'),
                ('topics.cartesian_command_in_topic', '/servo_node/delta_twist_cmds'),
                ('topics.joint_command_in_topic', '/servo_node/delta_joint_cmds'),
            ]
        )

        # Declare robot parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot.end_effector_frame', 'panda_hand'),
                ('robot.base_frame', 'panda_link0'),
                ('robot.joint_names', [
                    'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                    'panda_joint5', 'panda_joint6', 'panda_joint7'
                ]),
                ('robot.gripper.joint_name', 'panda_finger_joint1'),
                ('robot.gripper.max_opening', 0.04),
                ('robot.gripper.min_opening', 0.0),
            ]
        )
        
        # Declare gamepad parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gamepad.deadzone', 0.25),
                ('gamepad.velocity_multiplier', 0.5),
                ('gamepad.gripper_velocity_multiplier', 0.0005),
            ]
        )

        # Declare gamepad functions
        functions = {
            'mode_idle': ("button", 0),
            'mode_cartesian': ("button", 2),
            'mode_joint': ("button", 1),
            'gripper_open': ("button", 5),
            'gripper_close': ("button", 4),
            'cartesian_x_axis_angular': ("axis", 0),
            'cartesian_y_axis_angular': ("axis", 1),
            'cartesian_z_axis_angular': ("axis", 4),
            'cartesian_x_axis_linear_right': ("button", 7, 0.0),
            'cartesian_x_axis_linear_left': ("button", 6, 0.0),
            'cartesian_y_axis_linear': ("axis", 2),
            'cartesian_z_axis_linear': ("axis", 3),
            'joint_1': ("axis", 0),
            'joint_2': ("axis", 1),
            'joint_3': ("axis", 2),
            'joint_4': ("axis", 3),
            'joint_5': ("axis", 4),
            'joint_6': ("axis", 5),
            'joint_7_right': ("button", 7, 0.0),
            'joint_7_left': ("button", 6, 0.0),
        }
        
        for key, value in functions.items():
            if len(value) == 2:
                param_type, index = value
                self.declare_parameter(f'gamepad.functions.{key}.type', param_type)
                self.declare_parameter(f'gamepad.functions.{key}.index', index)
            elif len(value) == 3:
                param_type, index, default = value
                self.declare_parameter(f'gamepad.functions.{key}.type', param_type)
                self.declare_parameter(f'gamepad.functions.{key}.index', index)
                self.declare_parameter(f'gamepad.functions.{key}.default', default)

    def __init__(self):
        super().__init__("joystick_servo_publisher")

        # Initialize parameters
        self.init_params()

        # Initialize variables
        self.mode = ControllerMode.CARTESIAN
        self.frame_to_publish = self.get_robot_property("base_frame")
        self.gripper = self.get_robot_property("gripper.min_opening")

        # Initialize subscribers and publishers
        self.joy_sub = self.create_subscription(Joy, self.get_topic("joy_topic"), self.joy_cb, 10)
        self.twist_pub = self.create_publisher(TwistStamped, self.get_topic("cartesian_command_in_topic"), 10)
        self.joint_pub = self.create_publisher(JointJog, self.get_topic("joint_command_in_topic"), 10)
        self.collision_pub = self.create_publisher(PlanningScene, self.get_topic("planning_scene_topic"), 10)

        # Initialize action clients
        self.gripper_action = ActionClient(self, GripperCommand, self.get_topic("gripper_cmd_topic"))
        gripper_srv_status = self.gripper_action.wait_for_server(timeout_sec=5.0)
        if not gripper_srv_status:
            self.get_logger().error("Gripper action server not available")
            raise RuntimeError("Gripper action server not available")

        # Check for the servo node and start it if it is not running
        self.servo_start_client = self.create_client(Trigger, self.get_topic("servo_start_topic"))
        servo_srv_status = self.servo_start_client.wait_for_service(timeout_sec=5.0)
        if not servo_srv_status:
            self.get_logger().error("Servo service not available")
            raise RuntimeError("Servo service not available")
        self.servo_start_client.call_async(Trigger.Request())

        # Start a thread to publish the collision scene (useful for testing, should be disabled on real robot)
        self.collision_pub_thread = threading.Thread(target=self.publish_collision_scene)
        self.collision_pub_thread.start()

    def convert_joy_to_gripper_cmd(self, msg : Joy, goal : GripperCommand.Goal):
        val_pos = self.get_gamepad_property("gripper_velocity_multiplier") * self.get_btn_axis(msg, "gripper_open")
        val_neg = -self.get_gamepad_property("gripper_velocity_multiplier") * self.get_btn_axis(msg, "gripper_close")
        self.gripper += val_pos + val_neg
        if self.gripper >= self.get_robot_property("gripper.max_opening") or self.gripper <= self.get_robot_property("gripper.min_opening"):
            return

        goal.command.position = self.gripper
        goal.command.max_effort = 50.0 # arbitrary value
        self.gripper_action.send_goal_async(goal)

        return

    def publish_collision_scene(self):
        time.sleep(3)

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.get_robot_property("base_frame")
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
        if self.get_btn_axis(msg, "mode_idle"):
            self.mode = ControllerMode.IDLE
            self.get_logger().info("Mode changed to: {}".format(self.mode))
            return
        elif self.get_btn_axis(msg, "mode_joint"):
            self.mode = ControllerMode.JOINT
            self.get_logger().info("Mode changed to: {}".format(self.mode))
            return
        elif self.get_btn_axis(msg, "mode_cartesian"):
            self.mode = ControllerMode.CARTESIAN
            self.get_logger().info("Mode changed to: {}".format(self.mode))
            return

        if self.mode == ControllerMode.IDLE: # Do nothing if mode is IDLE
            pass
        elif self.mode == ControllerMode.JOINT: # Control joint velocities if mode is JOINT
            self.convert_joy_to_joint_cmd(msg, joint_msg)
            self.joint_pub.publish(joint_msg)
        elif self.mode == ControllerMode.CARTESIAN: # Control linear and angular velocities if mode is CARTESIAN
            self.convert_joy_to_cartesian_cmd(msg, twist_msg)
            self.twist_pub.publish(twist_msg)

        # Control gripper
        if self.get_btn_axis(msg, "gripper_open") or self.get_btn_axis(msg, "gripper_close"):
            gripper_goal = GripperCommand.Goal()
            self.convert_joy_to_gripper_cmd(msg, gripper_goal)
        
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
