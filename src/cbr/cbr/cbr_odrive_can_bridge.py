import struct
import threading
from enum import IntEnum

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from cbr_interfaces.action import Homing

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
        self.homing_finish_time = None
        self.homing_offset = 0.0
        self.future_homing = None
        
        # Thread lock to safely share data between CAN thread and ROS callbacks
        self.state_lock = threading.RLock() 

        # --- ROS Communications ---
        self.cb_group = ReentrantCallbackGroup()
        
        self.joint_state_publisher = self.create_publisher(JointState, f'state/{self.joint_name}', 10)
        self.joint_command_subscriber = self.create_subscription(
            JointState, f'commands/{self.joint_name}', self.joint_command_callback, 10, callback_group=self.cb_group
        )
        
        self.homing_server = ActionServer(self, Homing, f'home/{self.joint_name}', self.home_service_callback, callback_group=self.cb_group)
        self.axis_state_server = self.create_service(SetBool, f'set_axis_state/{self.joint_name}', self.set_axis_state_callback, callback_group=self.cb_group)

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
            
            # 2. Notifier: Spawns a background thread to read CAN messages
            self.can_notifier = can.Notifier(self.bus, [self._on_can_message])
            self.get_logger().info(f'ODrive Bridge started on {self.can_interface} for Axis {self.can_id} with hardware filtering.')
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CAN: {e}")
            raise e

        # Timer to poll for feedback (Replaces the loop in the original architecture)
        self.feedback_timer = self.create_timer(0.003, self._feedback_timer_callback, callback_group=self.cb_group)

    def _load_params(self):
        self.can_interface = self.get_parameter('can_interface').value
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

    # --------------------------------------------------------------------------
    # CAN Handling (Replaces Subscriber)
    # --------------------------------------------------------------------------

    def _on_can_message(self, msg: can.Message):
        """
        Callback triggered by python-can Notifier background thread.
        Contains the exact logic from the original `can_message_callback`.
        """
        # Note: Hardware filtering already ensures msg.arbitration_id matches our axis,
        # but the check below is kept for safety.
        if msg.arbitration_id >> 5 != self.can_id:
            return

        cmd_id = msg.arbitration_id & 0x1F

        if cmd_id == ODriveCommand.GET_ENCODER_ESTIMATES:
            pos_estimate, vel_estimate = struct.unpack('<ff', msg.data[:8])
            
            # Log with throttle to avoid console spam
            # (Note: rclpy logging isn't strictly thread-safe but usually works)
            # self.get_logger().info(f"Encoder: Pos={pos_estimate:.3f}, Vel={vel_estimate:.3f}", throttle_duration_sec=0.5)

            with self.state_lock:
                # 1. Publish Joint State
                if self.is_homed:
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = [self.joint_name]
                    joint_state_msg.position = [((pos_estimate - self.homing_offset) / self.reduction) * -self.homing_direction]
                    joint_state_msg.velocity = [(vel_estimate / self.reduction) * -self.homing_direction]
                    self.joint_state_publisher.publish(joint_state_msg)

                # 2. Homing Logic (Preserved exactly)
                if self.is_homing:
                    future_homing = self.future_homing
                    # Check if future exists and is not done
                    if self.is_homing and future_homing and not future_homing.done():
                        # self.get_logger().info(f"Homing... Vel: {vel_estimate}", throttle_duration_sec=0.5)
                        
                        if abs(vel_estimate) <= self.homing_speed_threshold:
                            current_time = self.get_clock().now().nanoseconds / 1e9
                            
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
                                
                                # Switch state
                                self._reset_odrive_state(set_idle=False)
                                
                                # Set Future Result
                                try:
                                    future_homing.set_result(True)
                                except Exception as e:
                                    self.get_logger().warn(f"Failed to set future result: {e}")
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
    # Service & Action Callbacks (Preserved)
    # --------------------------------------------------------------------------

    async def home_service_callback(self, goal_handle):
        with self.state_lock:
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
            else:
                self.is_homing = False
                self._reset_odrive_state(set_idle=True)
                goal_handle.abort()
                return Homing.Result(success=False)

        # Wait for the future (managed by CAN thread) or timeout
        timer = self.create_timer(20.0, self._homing_timeout_callback, callback_group=self.cb_group)
        
        try:
            success = await self.future_homing
        except Exception:
            success = False
        finally:
            timer.cancel()
            self.destroy_timer(timer)

        with self.state_lock:
            self.future_homing = None
            self.is_homing = False

            if success:
                goal_handle.succeed()
                return Homing.Result(success=True)
            else:
                self._reset_odrive_state(set_idle=True)
                goal_handle.abort()
                return Homing.Result(success=False)

    def _homing_timeout_callback(self):
        with self.state_lock:
            if self.future_homing and not self.future_homing.done():
                self.get_logger().warn("Homing timed out!")
                self.future_homing.set_result(False)

    def set_axis_state_callback(self, request, response):
        with self.state_lock:
            if request.data:
                self._reset_odrive_state(set_idle=False)
                self._publish_axis_state(ODriveAxisState.CLOSED_LOOP_CONTROL)
                response.message = f"Axis {self.joint_name} set to CLOSED_LOOP_CONTROL"
            else:
                self._reset_odrive_state(set_idle=True)
                response.message = f"Axis {self.joint_name} set to IDLE"
            response.success = True
        return response

    def joint_command_callback(self, msg: JointState):
        with self.state_lock:
            if not self.is_homed or self.is_homing:
                return
            if msg.name != [self.joint_name]:
                return

            if self.homing_finish_time:
                msg_time = Time.from_msg(msg.header.stamp)
                if msg_time < self.homing_finish_time:
                    return

            if msg.position:
                motor_position_command = (msg.position[0] * -self.homing_direction * self.reduction) + self.homing_offset
                self._publish_input_pos(motor_position_command)

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
        if hasattr(self, 'can_notifier'):
            self.can_notifier.stop()
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CBROdriveCANBridge()
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