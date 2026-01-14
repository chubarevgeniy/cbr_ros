import threading
import sys
import traceback

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from cbr_interfaces.action import Homing
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatus


class CLIcontroller(Node):
    def __init__(self):
        super().__init__('cli_controller')

        self.declare_parameter('joints', ['left_hip_joint', 'left_knee_joint', 'right_knee_joint', 'right_hip_joint'])
        self.joints = self.get_parameter('joints').value

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

        self.get_logger().info(f'State Manager initialized for joints: {self.joints}')
        self.print_help()

    def print_help(self):
        print("\nCommands:")
        print("  home <joint_name>      - Trigger homing")
        print("  pos <joint_name> <val> - Send position command")
        print("  state <joint_name> <on|off> - Set axis state")
        print("  list                   - List managed joints")
        print("  help                   - Show this help")
        print("  quit                   - Exit")
        print("> ", end="", flush=True)

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
        self.get_logger().info(f'Published position {position} for {joint_name}')

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

def input_loop(node):
    try:
        while rclpy.ok():
            line = sys.stdin.readline()
            if not line:
                print("Input stream closed (EOF). Exiting.")
                break
            line = line.strip()
            if not line:
                print("> ", end="", flush=True)
                continue

            parts = line.split()
            cmd = parts[0]

            if cmd == 'quit':
                break
            elif cmd == 'list':
                print(f"Joints: {node.joints}")
            elif cmd == 'help':
                node.print_help()
                continue
            elif cmd == 'home':
                if len(parts) < 2:
                    print("Usage: home <joint_name>")
                else:
                    node.send_homing_goal(parts[1])
            elif cmd == 'pos':
                if len(parts) < 3:
                    print("Usage: pos <joint_name> <position>")
                else:
                    try:
                        node.send_position_command(parts[1], float(parts[2]))
                    except ValueError:
                        print("Invalid position value")
            elif cmd == 'state':
                if len(parts) < 3:
                    print("Usage: state <joint_name> <on|off>")
                else:
                    state_val = parts[2].lower()
                    enable = state_val in ['on', 'true', '1']
                    node.send_state_command(parts[1], enable)
            else:
                print("Unknown command")

            print("> ", end="", flush=True)
    except Exception as e:
        print(f"Error in input loop: {e}")
    finally:
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CLIcontroller()

    # Start input thread
    input_thread = threading.Thread(target=input_loop, args=(node,), daemon=True)
    input_thread.start()

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
