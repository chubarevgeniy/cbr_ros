
import struct
from enum import IntEnum
from cbr_interfaces.action import Homing
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer
from can_msgs.msg import Frame
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from rclpy.task import Future
from rclpy.time import Time
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

class ODriveCommand(IntEnum):
    """ODrive CAN Command IDs."""
    HEARTBEAT = 0x01
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
        self.can_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.declare_parameter('can_bitrate', 250000)
        self.declare_parameter('can_id', 0)
        self.declare_parameter('homing_speed_threshold', 1.0)
        self.declare_parameter('homing_speed', 2.0)
        # time to assume that joint hit stopper (seconds)
        self.declare_parameter('homing_zero_speed_confirmation_time', 1.0)
        self.declare_parameter('homing_torque_limit', 0.025)
        self.declare_parameter('homing_direction', 1)
        self.declare_parameter('joint_name', 'joint0')
        self.declare_parameter('homing_current_limit', 5.0)
        self.declare_parameter('current_limit', 15.0)
        self.declare_parameter('default_vel_limit', 20.0)
        self.declare_parameter('reduction', 12.0)
        
        # Load initial parameters
        self._load_params()

        self.is_homing = False
        self.is_homed = False
        self.homing_start_time = None
        self.homing_finish_time = None
        self.homing_offset = 0.0
        
        self.joint_state_publisher = self.create_publisher(JointState, f'state/{self.joint_name}', 10)
        self.joint_command_subscriber = self.create_subscription(
            JointState, f'commands/{self.joint_name}', self.joint_command_callback, 10, callback_group=self.cb_group
        )
        self.can_publisher = self.create_publisher(Frame, f'can/tx', 10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.can_subscriber = self.create_subscription(
            Frame, f'can/rx', self.can_message_callback, qos_profile, callback_group=self.can_cb_group
        )
        
        self.future_homing = None
        self.homing_server = ActionServer(self, Homing, f'home/{self.joint_name}', self.home_service_callback, callback_group=self.cb_group)
        self.axis_state_server = self.create_service(SetBool, f'set_axis_state/{self.joint_name}', self.set_axis_state_callback, callback_group=self.cb_group)

        # Timer to poll for feedback (RTR) in case cyclic messages are not configured on ODrive
        self.feedback_timer = self.create_timer(0.005, self._feedback_timer_callback, callback_group=self.cb_group)

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
        self.get_logger().info(f"Parameters loaded: Joint={self.joint_name}, CAN ID={self.can_id}, Bitrate={self.can_bitrate}")

    async def home_service_callback(self, goal_handle):
        """Service to start the homing procedure for a specific axis."""
        
        if not self.is_homing and self.future_homing is None:
            self.is_homing = True
            self.future_homing = Future()
            self.homing_start_time = None
            self.homing_finish_time = None
            self.is_homed = False

            self._publish_controller_modes(ODriveControlMode.TORQUE_CONTROL, ODriveInputMode.PASSTHROUGH)
            self._publish_limits(self.default_vel_limit, self.homing_current_limit)
            self._publish_axis_state(ODriveAxisState.CLOSED_LOOP_CONTROL)
            self._publish_input_torque(self.homing_direction * self.homing_torque_limit)

            timer = self.create_timer(20.0, self._homing_timeout_callback, callback_group=self.cb_group)

            try:
                success = await self.future_homing
            finally:
                timer.cancel()
                self.destroy_timer(timer)

            self.future_homing = None
            self.is_homing = False

            if success:
                # Homing succeeded
                goal_handle.succeed()
                return Homing.Result(success=True)
            else:
                # Homing timed out, reset and go to idle
                self._reset_odrive_state(set_idle=True)
                goal_handle.abort()
                return Homing.Result(success=False)
        else:
            self.is_homing = False
            # Homing action called while already homing, reset and go to idle
            self._reset_odrive_state(set_idle=True)
            goal_handle.abort()
            return Homing.Result(success=False)

    def _homing_timeout_callback(self):
        if self.future_homing and not self.future_homing.done():
            self.get_logger().warn("Homing timed out!")
            self.future_homing.set_result(False)

    def set_axis_state_callback(self, request, response):
        """Service callback to set the axis state (Enable/Disable)."""
        if request.data:
            # Enable: Set to Closed Loop Control
            self._reset_odrive_state(set_idle=False) # Ensure known control mode (Position)
            self._publish_axis_state(ODriveAxisState.CLOSED_LOOP_CONTROL)
            response.message = f"Axis {self.joint_name} set to CLOSED_LOOP_CONTROL"
        else:
            # Disable: Set to Idle
            self._reset_odrive_state(set_idle=True)
            response.message = f"Axis {self.joint_name} set to IDLE"
        
        response.success = True
        return response
        
    def _feedback_timer_callback(self):
        """Periodically request encoder estimates via RTR."""
        msg = get_can_message(self.can_id, ODriveCommand.GET_ENCODER_ESTIMATES, is_rtr=True)
        self.can_publisher.publish(msg)

    def can_message_callback(self, msg):
        """Callback for incoming CAN messages from the ODrive."""
        # Check if the message is for this node's ODrive axis
        if msg.id >> 5 != self.can_id:
            return

        cmd_id = msg.id & 0x1F  # Mask out the command ID

        if cmd_id == ODriveCommand.GET_ENCODER_ESTIMATES:
            pos_estimate, vel_estimate = struct.unpack('<ff', msg.data[:8])
            self.get_logger().info(f"Encoder: Pos={pos_estimate:.3f}, Vel={vel_estimate:.3f}",throttle_duration_sec=0.5)

            # Only publish joint state if homed
            if self.is_homed:
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = [self.joint_name]
                joint_state_msg.position = [((pos_estimate - self.homing_offset) / self.reduction) * -self.homing_direction]
                joint_state_msg.velocity = [(vel_estimate / self.reduction) * -self.homing_direction]
                self.joint_state_publisher.publish(joint_state_msg)

            # Homing logic
            if self.is_homing:
                future_homing = self.future_homing
                if self.is_homing and future_homing and not future_homing.done():
                    self.get_logger().info(f"Homing: vel_estimate={vel_estimate}; pos_estimate={pos_estimate}",throttle_duration_sec=0.5)
                    if abs(vel_estimate) <= self.homing_speed_threshold:
                        current_time = self.get_clock().now().nanoseconds / 1e9
                        if self.homing_start_time is None:
                            self.homing_start_time = current_time
                            self.get_logger().info(f"Homing: Low velocity detected, starting confirmation timer: vel_estimate={vel_estimate}; pos_estimate={pos_estimate}.",throttle_duration_sec=0.5)
                        elif current_time - self.homing_start_time >= self.homing_zero_speed_confirmation_time:
                            self.get_logger().info(f"Homing: Stop confirmed. pos_estimate={pos_estimate}")
                            self.homing_offset = pos_estimate
                            self.homing_finish_time = self.get_clock().now()
                            self.is_homed = True
                            self.homing_start_time = None
                            # Homing complete, switch to IDLE. User must explicitly enable axis later.
                            self._reset_odrive_state(set_idle=False)
                            future_homing.set_result(True)
                    else:
                        self.homing_start_time = None # Reset timer if speed increases

        elif cmd_id == ODriveCommand.HEARTBEAT:
            axis_error, axis_state = struct.unpack('<II', msg.data[:8])
            self.get_logger().info(f"Heartbeat: Error={axis_error}, State={axis_state}")
            if axis_error != 0:
                self.get_logger().error(f"ODrive Axis Error: {axis_error} | State: {axis_state}")

    def joint_command_callback(self, msg: JointState):
        if not self.is_homed:
            return
        if self.is_homing:
            return
        if msg.name != [self.joint_name]:
            return

        if self.homing_finish_time:
            msg_time = Time.from_msg(msg.header.stamp)
            if msg_time < self.homing_finish_time:
                self.get_logger().info(f"Ignored old command: {msg.position} (Time: {msg_time.nanoseconds} < {self.homing_finish_time.nanoseconds})")
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
        if set_idle:
            self._publish_axis_state(ODriveAxisState.IDLE)
            return

        self.get_logger().info(f"Resetting state. Homing offset: {self.homing_offset:.3f}")
        self._publish_input_pos(self.homing_offset)
        self._publish_limits(self.default_vel_limit, self.current_limit)
        self._publish_controller_modes(ODriveControlMode.POSITION_CONTROL, ODriveInputMode.PASSTHROUGH)

def get_can_message(id, cmd_id, data=b'', is_rtr=False):
    can_id = id << 5 | cmd_id
    frame_data = list(data)
    # Pad data to 8 bytes with zeros to ensure fixed-size array compatibility
    frame_data.extend([0] * (8 - len(frame_data)))
    return Frame(id=can_id, dlc=len(data) if not is_rtr else 8, data=frame_data, is_extended=False, is_rtr=is_rtr)


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