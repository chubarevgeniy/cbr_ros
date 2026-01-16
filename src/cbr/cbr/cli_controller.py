import threading
import sys
import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class CLIcontroller(Node):
    def __init__(self):
        super().__init__('cli_controller')

        self.declare_parameter('joints', ['left_hip_joint', 'left_knee_joint', 'right_knee_joint', 'right_hip_joint'])
        self.joints = self.get_parameter('joints').value

        self.homing_pubs = {}
        self.publishers_ = {}
        self.state_pubs = {}

        for joint in self.joints:
            # Publisher for Homing
            self.homing_pubs[joint] = self.create_publisher(Bool, f'home/{joint}', 10)

            # Publisher for Position Commands
            self.publishers_[joint] = self.create_publisher(JointState, f'commands/{joint}', 10)

            # Publisher for Axis State
            self.state_pubs[joint] = self.create_publisher(Bool, f'set_axis_state/{joint}', 10)

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
        if joint_name not in self.homing_pubs:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        msg = Bool()
        msg.data = True
        self.homing_pubs[joint_name].publish(msg)
        self.get_logger().info(f'Published homing trigger for {joint_name}')

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
        if joint_name not in self.state_pubs:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return

        msg = Bool()
        msg.data = enable
        self.state_pubs[joint_name].publish(msg)
        self.get_logger().info(f'Published state command {enable} for {joint_name}')

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
