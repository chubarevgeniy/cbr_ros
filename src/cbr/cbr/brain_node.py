from enum import IntEnum
import traceback
from cbr.controller import ControllerState
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool
from rclpy.time import Time
import copy
import rclpy

class StateMode(IntEnum):
    """State Modes."""
    PASSIVE = 0,
    HOMING = 1,
    AI = 2,
    STATIC = 3,

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.controller_state = ControllerState()
        self.prev_controller_state = ControllerState()
        
        self.mode = StateMode.PASSIVE
        self.homing_start_time = None
        
        self.declare_parameter('joints', ['left_hip_joint', 'left_knee_joint', 'right_knee_joint', 'right_hip_joint'])
        self.declare_parameter('homing_time',10.0)
        self._load_params()
        
        self.timer = self.create_timer(1.0/50.0, self.timer_callback)
        
        self.joint_states = {}
        self.static_state = {}
        self.base_state = {}
        
        self.homing_pubs = {}
        self.publishers_ = {}
        self.subs_ = {}
        self.state_pubs = {}

        for joint in self.joints:
            # Publisher for Homing Trigger
            self.homing_pubs[joint] = self.create_publisher(Bool, f'home/{joint}', 10)

            # Publisher for Position Commands
            self.publishers_[joint] = self.create_publisher(JointState, f'commands/{joint}', 10)

            # Publisher for Axis State
            self.state_pubs[joint] = self.create_publisher(Bool, f'set_axis_state/{joint}', 10)
            
            # Subscriber for axis states (position etc)
            self.subs_[joint] = self.create_subscription(JointState, f'state/{joint}', self.axis_subscriber, 1)
            
        self.subs_['tilt'] = self.create_subscription(JointState, f'state/tilt', self.axis_subscriber, 1)
        self.subs_['height'] = self.create_subscription(JointState, f'state/height', self.axis_subscriber, 1)
        self.subs_['position'] = self.create_subscription(JointState, f'state/position', self.axis_subscriber, 1)
            
    def _load_params(self):
        self.joints = self.get_parameter('joints').value
        self.homing_time = self.get_parameter('homing_time').value
        
    def axis_subscriber(self, msg: JointState):
        if not(msg.name[0] in self.joints):
            if msg.name[0] in ['tilt','height','position']:
                self.base_state[msg.name[0]] = {'position':msg.position[0], 'velocity': msg.velocity[0], 'msg_time':Time.from_msg(msg.header.stamp)}
            return
        self.joint_states[msg.name[0]] = {'position':msg.position[0], 'velocity': msg.velocity[0], 'msg_time':Time.from_msg(msg.header.stamp)}
        
    def joy_callback(self, msg):
        self.controller_state.get_from_joy(msg)

        # Button logic (Edge detection)
        # Burger button: Home robot
        if self.controller_state.burger and not self.prev_controller_state.burger:
            self.get_logger().info(f"Start Homing")
            self.home_all_joints()
            self.mode = StateMode.HOMING
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.homing_start_time = current_time

        # A button: Active
        if self.controller_state.a and not self.prev_controller_state.a:
            self.get_logger().info(f"Set Active State")
            self.set_all_states(True)
            for name in self.joints:
                self.static_state[name] = self.joint_states[name]
            self.mode = StateMode.STATIC

        # X button: Passive mode
        if self.controller_state.x:
            self.get_logger().info(f"Set Passive State")
            self.set_all_states(False)
            self.mode = StateMode.PASSIVE

        self.prev_controller_state.get_from_joy(msg)
        
    def home_all_joints(self):
        for joint in self.joints:
            self.send_homing_goal(joint)
            
    def send_homing_goal(self, joint_name):
        if joint_name not in self.homing_pubs:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        msg = Bool()
        msg.data = True
        self.homing_pubs[joint_name].publish(msg)
        self.get_logger().info(f'Sent homing trigger for {joint_name}')
        
    def send_position_command(self, joint_name, position):
        if joint_name not in self.publishers_:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint_name]
        msg.position = [float(position)]

        self.publishers_[joint_name].publish(msg)
        
    def send_state_command(self, joint_name, enable):
        if joint_name not in self.state_pubs:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        msg = Bool()
        msg.data = enable
        self.state_pubs[joint_name].publish(msg)
        self.get_logger().info(f'Sent state command {enable} for {joint_name}')
        
    def set_all_states(self, enable):
        for joint in self.joints:
            self.send_state_command(joint, enable)
            
    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.mode == StateMode.HOMING and not(self.homing_start_time is None) and current_time-self.homing_start_time>self.homing_time:
            self.mode = StateMode.PASSIVE
        if self.mode == StateMode.STATIC:
            for joint in self.joints:
                self.send_position_command(joint,self.static_state[joint]['position'])
        if self.mode == StateMode.AI:
            # take all states and publish commands
            pass
        
def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception in spin: {e}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()