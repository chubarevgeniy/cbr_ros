
import struct
from enum import IntEnum
from cbr_interfaces.action import Homing
import asyncio
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer
from can_msgs.msg import Frame
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from rclpy.executors import MultiThreadedExecutor

class ODriveCommand(IntEnum):
    """ODrive CAN Command IDs."""
    SET_AXIS_STATE = 0x07
    GET_ENCODER_ESTIMATES = 0x09
    SET_CONTROLLER_MODES = 0x0B
    SET_INPUT_POS = 0x0C
    SET_INPUT_VEL = 0x0D
    SET_INPUT_TORQUE = 0x0E
    SET_LIMITS = 0x0F

class ODriveAxisState(IntEnum):
    """ODrive Axis States."""
    IDLE = 1
    CLOSED_LOOP_CONTROL = 8

class ODriveControlMode(IntEnum):
    """ODrive Control Modes."""
    TORQUE_CONTROL = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3
class ODriveInputMode(IntEnum):
    """ODrive Input Modes."""
    PASSTHROUGH = 1

class CBROdriveCANBridge(Node):
    def __init__(self):
        super().__init__('cbr_odrive_can_bridge')
        self.cb_group = ReentrantCallbackGroup()
        
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('can_id', 0)
        self.declare_parameter('homing_speed_threshold', 0.5)
        self.declare_parameter('homing_speed', 2.0)
        # time to assume that joint hit stopper (seconds)
        self.declare_parameter('homing_zero_speed_confirmation_time',2)
        self.declare_parameter('homing_torque_limit', 0.1)
        self.declare_parameter('homing_direction', 1)
        self.declare_parameter('joint_name', 'joint0')
        self.declare_parameter('homing_current_limit', 5.0)
        self.declare_parameter('current_limit', 15.0)
        self.declare_parameter('default_vel_limit', 20.0)
        self.declare_parameter('reduction', 1.0)
        
        # Load initial parameters
        self._load_params()

        self.is_homing = False
        self.is_homed = False
        self.homing_start_time = None
        self.homing_offset = 0.0
        
        self.joint_state_publisher = self.create_publisher(JointState, f'state/{self.joint_name}', 10)
        self.joint_command_subscriber = self.create_subscription(
            JointState, f'commands/{self.joint_name}', self.joint_command_callback, 10, callback_group=self.cb_group
        )
        self.can_publisher = self.create_publisher(Frame, f'can/tx', 10)
        self.can_subscriber = self.create_subscription(
            Frame, f'can/rx', self.can_message_callback, 10, callback_group=self.cb_group
        )
        
        self.future_homing = None
        self.homing_server = ActionServer(self, Homing, f'home/{self.joint_name}', self.home_service_callback, callback_group=self.cb_group)

        self.get_logger().info('ODrive CAN Bridge has been started.')

    def _load_params(self):
        """Load parameters from the parameter server."""
        self.can_bitrate = self.get_parameter('can_bitrate').value
        self.can_id = self.get_parameter('can_id').value
        self.homing_speed_threshold = self.get_parameter('homing_speed_threshold').value
        self.homing_speed = self.get_parameter('homing_speed').value
        self.homing_zero_speed_confirmation_time = self.get_parameter('homing_zero_speed_confirmation_time').value
        self.joint_name = self.get_parameter('joint_name').value
        self.homing_direction = self.get_parameter('homing_direction').value
        self.homing_torque_limit = self.get_parameter('homing_torque_limit').value
        self.homing_current_limit = self.get_parameter('homing_current_limit').value
        self.current_limit = self.get_parameter('current_limit').value
        self.default_vel_limit = self.get_parameter('default_vel_limit').value
        self.reduction = self.get_parameter('reduction').value
        self.get_logger().info("Parameters loaded/updated.")

    async def home_service_callback(self, goal_handle):
        """Service to start the homing procedure for a specific axis."""
        
        if not self.is_homing and self.future_homing is None:
            loop = asyncio.get_running_loop()
            self.is_homing = True
            self.future_homing = loop.create_future()

            self._publish_axis_state(ODriveAxisState.CLOSED_LOOP_CONTROL)
            self._publish_controller_modes(ODriveControlMode.TORQUE_CONTROL, ODriveInputMode.PASSTHROUGH)
            self._publish_limits(self.homing_speed, self.homing_current_limit)
            self._publish_input_torque(self.homing_direction * self.homing_torque_limit)

            try:
                await asyncio.wait_for(self.future_homing, timeout=20)
                self.future_homing = None
                self.is_homing = False
            
                # Homing succeeded, reset to position control
                self._reset_odrive_state()

                return Homing.Result(success=True)
            except asyncio.TimeoutError:
                self.is_homing = False
                # Homing timed out, reset and go to idle
                self._reset_odrive_state(set_idle=True)
                self.future_homing = None
                return Homing.Result(success=False)
        else:
            self.is_homing = False
            # Homing action called while already homing, reset and go to idle
            self._reset_odrive_state(set_idle=True)
            return Homing.Result(success=False)
        
    def can_message_callback(self, msg):
        """Callback for incoming CAN messages from the ODrive."""
        # Check if the message is for this node's ODrive axis
        if msg.id >> 5 != self.can_id:
            return

        cmd_id = msg.id & 0x1F  # Mask out the command ID

        if cmd_id == ODriveCommand.GET_ENCODER_ESTIMATES:
            pos_estimate, vel_estimate = struct.unpack('<ff', msg.data[:8])

            # Only publish joint state if homed
            if self.is_homed:
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = [self.joint_name]
                joint_state_msg.position = [((pos_estimate - self.homing_offset) / self.reduction) * -self.homing_direction]
                joint_state_msg.velocity = [(vel_estimate / self.reduction) * -self.homing_direction]
                self.joint_state_publisher.publish(joint_state_msg)

            # Homing logic
            if self.is_homing and self.future_homing:
                if abs(vel_estimate) <= self.homing_speed_threshold:
                    current_time = self.get_clock().now().seconds_nanoseconds()[0]
                    if self.homing_start_time is None:
                        self.homing_start_time = current_time
                        self.get_logger().info("Homing: Low velocity detected, starting confirmation timer.")
                    elif current_time - self.homing_start_time >= self.homing_zero_speed_confirmation_time:
                        self.get_logger().info("Homing: Stop confirmed.")
                        self.homing_offset = pos_estimate
                        self.is_homed = True
                        self.homing_start_time = 0
                        if not self.future_homing.done():
                            self.future_homing.set_result(True)
                else:
                    self.homing_start_time = None # Reset timer if speed increases

    def joint_command_callback(self, msg: JointState):
        if not self.is_homed:
            return
        if self.is_homing:
            return
        if msg.name != [self.joint_name]:
            return

        if msg.position:
            # Convert joint position command to motor position command
            # motor_pos = joint_pos * reduction
            motor_position_command = (msg.position[0] * -self.homing_direction * self.reduction) + self.homing_offset
            self._publish_input_pos(motor_position_command)
        
    def _publish_axis_state(self, state):
        """Publish a CAN message to set the axis state."""
        msg = get_can_message(self.can_id, ODriveCommand.SET_AXIS_STATE, struct.pack("<I", state))
        self.can_publisher.publish(msg)

    def _publish_controller_modes(self, control_mode, input_mode):
        """Publish a CAN message to set controller modes."""
        msg = get_can_message(self.can_id, ODriveCommand.SET_CONTROLLER_MODES, struct.pack("<ii", control_mode, input_mode))
        self.can_publisher.publish(msg)

    def _publish_limits(self, vel_limit, current_limit):
        """Publish a CAN message to set velocity and current limits."""
        msg = get_can_message(self.can_id, ODriveCommand.SET_LIMITS, struct.pack("<ff", vel_limit, current_limit))
        self.can_publisher.publish(msg)

    def _publish_input_torque(self, torque):
        """Publish a CAN message to set the input torque."""
        msg = get_can_message(self.can_id, ODriveCommand.SET_INPUT_TORQUE, struct.pack("<f", torque))
        self.can_publisher.publish(msg)

    def _publish_input_pos(self, pos, vel_ff=0.0, torque_ff=0.0):
        """Publish a CAN message to set the input position."""
        msg = get_can_message(self.can_id, ODriveCommand.SET_INPUT_POS, struct.pack('<fhh', pos, int(vel_ff * 1000), int(torque_ff * 1000)))
        self.can_publisher.publish(msg)

    def _reset_odrive_state(self, set_idle=False):
        """Reset ODrive to a known state after an operation."""
        self._publish_input_torque(0.0)
        self._publish_controller_modes(ODriveControlMode.POSITION_CONTROL, ODriveInputMode.PASSTHROUGH)
        self._publish_limits(self.default_vel_limit, self.current_limit)
        if set_idle:
            self._publish_axis_state(ODriveAxisState.IDLE)

def get_can_message(id, cmd_id, data=b''):
    can_id = id << 5 | cmd_id
    return Frame(id=can_id, dlc=len(data), data=list(data), is_extended=False)


def main(args=None):
    rclpy.init(args=args)
    node = CBROdriveCANBridge()

    # Use MultiThreadedExecutor to allow callbacks to run in parallel
    # This is crucial for the Action Server to wait while the CAN subscriber updates
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()