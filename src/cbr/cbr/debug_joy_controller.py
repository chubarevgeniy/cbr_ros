import traceback

from cbr.cbr.controller import ControllerState
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool

class DebugJoyController(Node):
    def __init__(self):
        super().__init__('debug_joy_controller')

        self.declare_parameter('joints', ['left_hip_joint', 'left_knee_joint', 'right_knee_joint', 'right_hip_joint'])
        self.joints = self.get_parameter('joints').value
        self.smoothness = 0.9

        self.homing_pubs = {}
        self.publishers_ = {}
        self.state_pubs = {}

        for joint in self.joints:
            # Publisher for Homing Trigger
            self.homing_pubs[joint] = self.create_publisher(Bool, f'home/{joint}', 10)

            # Publisher for Position Commands
            self.publishers_[joint] = self.create_publisher(JointState, f'commands/{joint}', 10)

            # Publisher for Axis State
            self.state_pubs[joint] = self.create_publisher(Bool, f'set_axis_state/{joint}', 10)

        self.get_logger().info(f'Debug Joy Controller initialized for joints: {self.joints}')

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(1.0/50.0, self.timer_callback)

        self.controller_state = ControllerState()
        self.prev_controller_state = ControllerState()
        self.joint_positions = {joint: 0.0 for joint in self.joints}

    def joy_callback(self, msg):
        self.controller_state.get_from_joy(msg)

        # Button logic (Edge detection)
        # A button: Enable
        if self.controller_state.a and not self.prev_controller_state.a:
            self.set_all_states(True)

        # B button: Disable
        if self.controller_state.b and not self.prev_controller_state.b:
            self.set_all_states(False)

        # X button: Home
        if self.controller_state.x and not self.prev_controller_state.x:
            self.home_all()

        self.prev_controller_state.get_from_joy(msg)

    def timer_callback(self):
        # Update positions smoothly
        target_map = {
            'left_hip_joint': self.controller_state.l_v,
            'left_knee_joint': self.controller_state.l_h,
            'right_hip_joint': self.controller_state.r_v,
            'right_knee_joint': self.controller_state.r_h
        }

        for joint in self.joints:
            if joint in target_map:
                target = target_map[joint]
                self.joint_positions[joint] = self.joint_positions[joint] * self.smoothness + (1-self.smoothness) * target
                self.send_position_command(joint, self.joint_positions[joint])

    def set_all_states(self, enable):
        for joint in self.joints:
            self.send_state_command(joint, enable)

    def home_all(self):
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
        # self.get_logger().info(f'Published position {position} for {joint_name}')

    def send_state_command(self, joint_name, enable):
        if joint_name not in self.state_pubs:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        msg = Bool()
        msg.data = enable
        self.state_pubs[joint_name].publish(msg)
        self.get_logger().info(f'Sent state command {enable} for {joint_name}')

def main(args=None):
    rclpy.init(args=args)
    node = DebugJoyController()

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
