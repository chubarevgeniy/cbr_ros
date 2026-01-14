import traceback

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Joy
from cbr_interfaces.action import Homing
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatus

class ControllerState:
    def __init__(self):
        self.get_from_joy(Joy(axes=[0 for i in range(8)], buttons=[0 for i in range(16)]))
    
    def get_from_joy(self, msg: Joy):
        self.r_v = msg.axes[3]
        self.r_h = msg.axes[2]
        self.l_v = msg.axes[1]
        self.l_h = msg.axes[0]
        self.left_right = msg.axes[6]
        self.up_down = msg.axes[7]
        self.l_trig = msg.axes[5]
        self.r_trig = msg.axes[4]
        self.x = msg.buttons[3]
        self.y = msg.buttons[4]
        self.a = msg.buttons[0]
        self.b = msg.buttons[1]
        self.burger = msg.buttons[11]
        self.xbox = msg.buttons[12]
        self.squares = msg.buttons[10]
        self.l_joy_click = msg.buttons[13]
        self.r_joy_click = msg.buttons[14]
        self.l_button = msg.buttons[6]
        self.r_button = msg.buttons[7]


class DebugJoyController(Node):
    def __init__(self):
        super().__init__('debug_joy_controller')

        self.declare_parameter('joints', ['left_hip_joint', 'left_knee_joint', 'right_knee_joint', 'right_hip_joint'])
        self.joints = self.get_parameter('joints').value
        self.smoothness = 0.9

        self.action_clients = {}
        self.publishers_ = {}
        self.service_clients = {}

        for joint in self.joints:
            # Action Client for Homing
            self.action_clients[joint] = ActionClient(self, Homing, f'home/{joint}')

            # Publisher for Position Commands
            self.publishers_[joint] = self.create_publisher(JointState, f'commands/{joint}', 10)

            # Service Client for Axis State
            self.service_clients[joint] = self.create_client(SetBool, f'set_axis_state/{joint}')

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
        if joint_name not in self.action_clients:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        client = self.action_clients[joint_name]

        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f'Action server not available for {joint_name}')
            return

        goal_msg = Homing.Goal()
        self.get_logger().info(f'Sending homing goal for {joint_name}...')

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, joint_name))

    def goal_response_callback(self, future, joint_name):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed for {joint_name}: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().info(f'Homing goal rejected for {joint_name}')
            return

        self.get_logger().info(f'Homing goal accepted for {joint_name}')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(f, joint_name))

    def get_result_callback(self, future, joint_name):
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            status = wrapped_result.status
        except Exception as e:
            self.get_logger().error(f'Failed to get result for {joint_name}: {e}')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            if hasattr(result, 'success') and result.success:
                self.get_logger().info(f'Homing succeeded for {joint_name}')
            else:
                self.get_logger().info(f'Homing finished, but success flag is False or missing. Result: {result}')
        else:
            self.get_logger().info(f'Homing action failed with status: {status}')

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
        if joint_name not in self.service_clients:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        client = self.service_clients[joint_name]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Service not available for {joint_name}')
            return

        req = SetBool.Request()
        req.data = enable
        future = client.call_async(req)
        future.add_done_callback(lambda f: self.state_response_callback(f, joint_name))

    def state_response_callback(self, future, joint_name):
        try:
            response = future.result()
            self.get_logger().info(f'State change for {joint_name}: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed for {joint_name}: {e}')

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
