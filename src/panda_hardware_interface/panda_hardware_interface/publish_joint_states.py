import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
import math
import struct
import zlib
import time
import serial

# Data frame structure
# [start_bit][payload_length (2 byte)][payload][CRC32 (4 byte)][end_bit]
#
# Payload structure (repeated for each joint):
# [name_len (1 byte)][name (name_len byte)][position (8 byte)]

FRAME_START_BIT = 0x80
FRAME_END_BIT = 0x81

class PublishJointStates(Node):
    old_message = None

    def __init__(self):
        super().__init__('publish_joint_states')

        # Get serial port and baudrate from parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # Initialize the serial port
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.get_logger().info('Serial port: {}'.format(serial_port))

        self.ser = serial.Serial(serial_port, baudrate, timeout=5)

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def encode(self, name: list[str], position: list[float]) -> bytearray:
        frame = bytearray()

        # Start bit
        frame.append(FRAME_START_BIT)

        # Create payload which will have name len, name, position
        payload = bytearray()
        for i, name in enumerate(name):
            payload.extend(struct.pack('>B', len(name)))
            payload.extend(name.encode('utf-8'))
            payload.extend(struct.pack('>f', float(position[i])))

        # Add payload to frame (length + payload)
        frame.extend(struct.pack('>H', len(payload)))
        frame.extend(payload)

        # Calculate CRC32 of payload
        crc = zlib.crc32(payload)
        frame.extend(struct.pack('>I', crc))

        # End bit
        frame.append(FRAME_END_BIT)

        return frame
    
    def decode(self, frame: bytearray) -> tuple[list[str], list[float]]:
        if frame[0] != FRAME_START_BIT:
            self.get_logger().info('Start bit not found')
            return None, None
        
        if frame[-1] != FRAME_END_BIT:
            self.get_logger().info('End bit not found')
            return None, None

        # Get payload length
        payload_len = struct.unpack('>H', frame[1:3])[0]

        # Get payload
        payload = frame[3:3+payload_len]
        
        # Get CRC32
        crc = struct.unpack('>I', frame[-5:-1])[0]

        # Check CRC32
        if crc != zlib.crc32(payload):
            self.get_logger().info('CRC32 check failed')
            return None, None

        # Get name len, name, position
        name = []
        position = []
        i = 0
        while i < len(payload):
            name_len = payload[i]
            i += 1
            name.append(payload[i:i+name_len].decode('utf-8'))
            i += name_len
            position.append(struct.unpack('>f', payload[i:i+4])[0])
            i += 4

        return name, position

    def joint_state_callback(self, msg : JointState):
        # Send the message only if it is different from the previous one
        if self.old_message == msg.position:
            return
        self.old_message = msg.position

        frame = self.encode(msg.name, msg.position)
        #self.get_logger().info("Encoded frame: {}".format(frame))
        #self.get_logger().info("Encoded frame length: {}".format(len(frame)))

        # Send the frame to serial port
        self.ser.write(frame)

        name, position = self.decode(frame)
        self.get_logger().info('Decoded name: {}'.format(name))
        self.get_logger().info('Decoded position: {}'.format(position))

def main(args=None):
    rclpy.init(args=args)

    publish_joint_states = PublishJointStates()

    executor = MultiThreadedExecutor()
    executor.add_node(publish_joint_states)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        publish_joint_states.destroy_node()
        rclpy.shutdown()