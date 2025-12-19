from can_msgs.msg import Frame
import can
from rclpy.node import Node
import rclpy

class CANBridge(Node):
    def __init__(self):
        super().__init__('can_bridge')
        
        self.declare_parameter('can_bitrate', 500000)
        
        self._load_params()
        self.bustype = 'socketcan'
        self.channel = 'can0'
        self.bus = can.interface.Bus(bustype=self.bustype, channel=self.channel, bitrate=self.can_bitrate)
        
        self.publisher = self.create_publisher(Frame, f'can/rx', 10)
        self.subscriber = self.create_subscription(
            Frame, f'can/tx', self.subscriber_callback, 10
        )
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.timer_callback)
      
    def _load_params(self):
        self.can_bitrate = self.get_parameter('can_bitrate').value
          
    def subscriber_callback(self, msg: Frame):
        message = can.Message(
            arbitration_id=msg.id,
            data=bytes(msg.data),
            is_extended_id= msg.is_extended,
        )
        self.bus.send(message)
        
    def timer_callback(self):
        """Polls the CAN bus for new messages and publishes them to can/rx."""
        while True:
            # Use timeout=0 for non-blocking check
            can_msg = self.bus.recv(timeout=0)
            if can_msg is None:
                break  # No more messages currently in the buffer
            
            # Convert python-can message to ROS can_msgs/Frame
            ros_msg = Frame()
            ros_msg.id = can_msg.arbitration_id
            ros_msg.is_extended = can_msg.is_extended_id
            ros_msg.is_rtr = can_msg.is_remote_frame
            ros_msg.is_error = can_msg.is_error_frame
            ros_msg.dlc = can_msg.dlc
            ros_msg.data = list(can_msg.data)
            
            self.publisher.publish(ros_msg)
            
def main(args=None):
    rclpy.init(args=args)

    can_bridge = CANBridge()

    try:
        rclpy.spin(can_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        can_bridge.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()