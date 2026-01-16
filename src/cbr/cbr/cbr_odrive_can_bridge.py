import struct
from enum import IntEnum

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# Direct CAN Imports
import can

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
        
        # --- Parameters ---
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 250000)
        self.declare_parameter('can_id', 0)
        
        self.declare_parameter('homing_speed_threshold', 1.0)
        self.declare_parameter('homing_speed', 2.0)
        self.declare_parameter('homing_zero_speed_confirmation_time', 1.0)
        self.declare_parameter('homing_timeout', 10.0)
        self.declare_parameter('homing_torque_limit', 0.025)
        self.declare_parameter('homing_direction', 1)
        self.declare_parameter('joint_name', 'joint0')
        self.declare_parameter('homing_current_limit', 5.0)
        self.declare_parameter('current_limit', 15.0)
        self.declare_parameter('default_vel_limit', 20.0)
        self.declare_parameter('reduction', 12.0)
        
        self._load_params()

        # --- State Variables ---
        self.is_homing = False
        self.is_homed = False
        self.homing_start_time = None
        self.homing_init_time = None
        self.homing_finish_time = None
        self.homing_offset = 0.0
        
        self.joint_state_publisher = self.create_publisher(JointState, f'state/{self.joint_name}', 10)
        self.joint_command_subscriber = self.create_subscription(
            JointState, f'commands/{self.joint_name}', self.joint_command_callback, 10
        )
        
        self.homing_sub = self.create_subscription(Bool, f'home/{self.joint_name}', self.homing_callback, 10)
        self.axis_state_sub = self.create_subscription(Bool, f'set_axis_state/{self.joint_name}', self.axis_state_callback, 10)

        # --- CAN Bus Initialization (Direct) ---
        try:
            # 1. Hardware Filter: Only receive messages for THIS specific ODrive Axis ID
            # This is critical for running multiple nodes on the same bus efficiently.
            # Mask 0x7E0 (11111100000 in binary) checks the top 6 bits (Node ID). 
            # 0x1F allows all Command IDs for that Node ID to pass.
            can_filter = [{"can_id": self.can_id << 5, "can_mask": 0x7E0, "extended": False}]
            
            self.bus = can.interface.Bus(
                channel=self.can_interface, 
                bustype='socketcan', 
                bitrate=self.can_bitrate,
                can_filters=can_filter 
            )
            
            self.get_logger().info(f'ODrive Bridge started on {self.can_interface} for Axis {self.can_id} with hardware filtering.')
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CAN: {e}")
            raise e

        # Timer to poll CAN messages (Single Threaded approach)
        self.can_timer = self.create_timer(0.001, self.can_timer_callback)

        # Timer to poll for feedback (Replaces the loop in the original architecture)
        self.feedback_timer = self.create_timer(0.01, self._feedback_timer_callback)

    def _load_params(self):
        self.can_interface = self.get_parameter('can_interface').value
        self.can_bitrate = self.get_parameter('can_bitrate').value
        self.can_id = self.get_parameter('can_id').value
        self.homing_speed_threshold = self.get_parameter('homing_speed_threshold').value
        self.homing_speed = self.get_parameter('homing_speed').value
        self.homing_zero_speed_confirmation_time = self.get_parameter('homing_zero_speed_confirmation_time').value
        self.homing_timeout = self.get_parameter('homing_timeout').value
        self.joint_name = self.get_parameter('joint_name').value
        self.homing_direction = self.get_parameter('homing_direction').value
        self.homing_torque_limit = self.get_parameter('homing_torque_limit').value
        self.homing_current_limit = self.get_parameter('homing_current_limit').value
        self.current_limit = self.get_parameter('current_limit').value
        self.default_vel_limit = self.get_parameter('default_vel_limit').value
        self.reduction = self.get_parameter('reduction').value

    # --------------------------------------------------------------------------
    # CAN Handling (Replaces Subscriber)
    # --------------------------------------------------------------------------

    def can_timer_callback(self):
        # Process multiple messages to prevent buffer buildup
        for _ in range(50):
            msg = self.bus.recv(timeout=0)
            if msg is None:
                break
            self._process_can_message(msg)

    def _process_can_message(self, msg: can.Message):
        if msg.arbitration_id >> 5 != self.can_id:
            return

        cmd_id = msg.arbitration_id & 0x1F

        if cmd_id == ODriveCommand.GET_ENCODER_ESTIMATES:
            pos_estimate, vel_estimate = struct.unpack('<ff', msg.data[:8])

            # --- 1. Publish Joint State ---
            if self.is_homed:
                msg_to_publish = JointState()
                msg_to_publish.header.stamp = self.get_clock().now().to_msg()
                msg_to_publish.name = [self.joint_name]
                msg_to_publish.position = [((pos_estimate - self.homing_offset) / self.reduction) * -self.homing_direction]
                msg_to_publish.velocity = [(vel_estimate / self.reduction) * -self.homing_direction]
                self.joint_state_publisher.publish(msg_to_publish)

            # --- 2. Homing Logic ---
            if self.is_homing:
                current_time = self.get_clock().now().nanoseconds / 1e9

                if self.homing_init_time and (current_time - self.homing_init_time > self.homing_timeout):
                    self.get_logger().warn(f"Homing timed out for {self.joint_name}")
                    self.is_homing = False
                    self._reset_odrive_state(set_idle=True)
                    return

                if abs(vel_estimate) <= self.homing_speed_threshold:
                    if self.homing_start_time is None:
                        self.homing_start_time = current_time
                        self.get_logger().info(f"Homing: Stop detected. Vel={vel_estimate:.3f}")
                    
                    elif current_time - self.homing_start_time >= self.homing_zero_speed_confirmation_time:
                        self.get_logger().info(f"Homing: Confirmed. Pos={pos_estimate:.3f}")
                        
                        self.homing_offset = pos_estimate
                        self.homing_finish_time = self.get_clock().now()
                        self.is_homed = True
                        self.is_homing = False
                        self.homing_start_time = None
                        
                        # Note: _reset_odrive_state sends CAN
                        self._reset_odrive_state(set_idle=False)
                else:
                    self.homing_start_time = None

        elif cmd_id == ODriveCommand.HEARTBEAT:
            axis_error, axis_state = struct.unpack('<II', msg.data[:8])
            if axis_error != 0:
                self.get_logger().error(f"ODrive Axis Error: {axis_error} | State: {axis_state}")

    def _feedback_timer_callback(self):
        """Triggers the RTR request for encoder estimates."""
        # Using send_message helper directly
        self._send_command(ODriveCommand.GET_ENCODER_ESTIMATES, is_rtr=True)

    # --------------------------------------------------------------------------
    # Topic Callbacks (Replaces Service & Action)
    # --------------------------------------------------------------------------

    def homing_callback(self, msg: Bool):
        if msg.data:
            if not self.is_homing:
                self.get_logger().info(f"Starting homing for {self.joint_name}")
                self.is_homing = True
                self.homing_start_time = None
                self.homing_init_time = self.get_clock().now().nanoseconds / 1e9
                self.homing_finish_time = None
                self.is_homed = False

                self._publish_controller_modes(ODriveControlMode.TORQUE_CONTROL, ODriveInputMode.PASSTHROUGH)
                self._publish_limits(self.default_vel_limit, self.homing_current_limit)
                self._publish_axis_state(ODriveAxisState.CLOSED_LOOP_CONTROL)
                self._publish_input_torque(self.homing_direction * self.homing_torque_limit)
        else:
            # Cancel homing
            self.is_homing = False
            self._reset_odrive_state(set_idle=True)

    def axis_state_callback(self, msg: Bool):
        if msg.data:
            self._reset_odrive_state(set_idle=False)
            self._publish_axis_state(ODriveAxisState.CLOSED_LOOP_CONTROL)
            self.get_logger().info(f"Axis {self.joint_name} set to CLOSED_LOOP_CONTROL")
        else:
            self._reset_odrive_state(set_idle=True)
            self.get_logger().info(f"Axis {self.joint_name} set to IDLE")

    def joint_command_callback(self, msg: JointState):
        # Variables to store command to send outside lock
        command_position = None

        if not self.is_homed or self.is_homing:
            return
        if msg.name != [self.joint_name]:
            return

        if self.homing_finish_time:
            msg_time = Time.from_msg(msg.header.stamp)
            if msg_time < self.homing_finish_time:
                return

        if msg.position:
            # Calculate the command value
            if msg.position[0] > 0:
                command_position = (msg.position[0] * -self.homing_direction * self.reduction) + self.homing_offset
            else:
                command_position = self.homing_offset

        if command_position is not None:
            self._publish_input_pos(command_position)

    # --------------------------------------------------------------------------
    # CAN Sending Helpers (Adjusted for python-can)
    # --------------------------------------------------------------------------

    def _send_command(self, cmd_id, data=b'', is_rtr=False):
        """Helper to send CAN messages directly."""
        arb_id = (self.can_id << 5) | cmd_id
        
        # ODrive expects 8 bytes padded with 0
        data_list = list(data)
        data_list.extend([0] * (8 - len(data_list)))
        final_data = bytes(data_list)

        msg = can.Message(
            arbitration_id=arb_id, 
            data=final_data, 
            is_extended_id=False, 
            is_remote_frame=is_rtr
        )
        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().warn(f"CAN Send Error: {e}")

    def _publish_axis_state(self, state):
        self._send_command(ODriveCommand.SET_AXIS_STATE, struct.pack("<I", state))

    def _publish_controller_modes(self, control_mode, input_mode):
        self._send_command(ODriveCommand.SET_CONTROLLER_MODES, struct.pack("<ii", control_mode, input_mode))

    def _publish_limits(self, vel_limit, current_limit):
        self._send_command(ODriveCommand.SET_LIMITS, struct.pack("<ff", vel_limit, current_limit))

    def _publish_input_torque(self, torque):
        self._send_command(ODriveCommand.SET_INPUT_TORQUE, struct.pack("<f", torque))

    def _publish_input_pos(self, pos, vel_ff=0.0, torque_ff=0.0):
        self._send_command(ODriveCommand.SET_INPUT_POS, struct.pack('<fhh', pos, int(vel_ff * 1000), int(torque_ff * 1000)))

    def _reset_odrive_state(self, set_idle=False):
        if set_idle:
            self._publish_axis_state(ODriveAxisState.IDLE)
            return
        
        self.get_logger().info(f"Resetting state. Homing offset: {self.homing_offset:.3f}")
        self._publish_input_pos(self.homing_offset)
        self._publish_limits(self.default_vel_limit, self.current_limit)
        self._publish_controller_modes(ODriveControlMode.POSITION_CONTROL, ODriveInputMode.PASSTHROUGH)

    def destroy_node(self):
        # Clean shutdown for Notifier and Bus
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CBROdriveCANBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()