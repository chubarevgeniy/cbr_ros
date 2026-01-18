import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import deque
import serial
from std_msgs.msg import Bool
from serial.tools import list_ports

class BaseBridge(Node):
    def __init__(self):
        super().__init__('base_bridge')
        
        self.declare_parameter('baudrate',115200)
        self.declare_parameter('port','/dev/ttyUSB0')
        # zero values used to determine that position should homed to using offset
        self.declare_parameter('position_zero',0)
        self.declare_parameter('height_zero',0)
        self.declare_parameter('tilt_zero',0)
        self.declare_parameter('position_dir',1)
        self.declare_parameter('height_dir',1)
        self.declare_parameter('tilt_dir',1)
        self.position_offset = 0
        self.height_offset = 0
        self.tilt_offset = 0
        
        self._load_params()
        
        self.position_pub = self.create_publisher(JointState,'state/position',1)
        self.height_pub = self.create_publisher(JointState,'state/height',1)
        self.tilt_pub = self.create_publisher(JointState,'state/tilt',1)
        
        self.home_sub = self.create_subscription(Bool, 'home_base', self.home_callback, 10)
        self.is_homing = False
        
        self.buffer = b''
        
        self.readings_buffer = deque(maxlen=50)
        
        ports = list_ports.comports()
        for i, port in enumerate(ports): self.get_logger().info(f"  [{i}]: {port.device} - {port.description}")
        
        
        self.serial = serial.Serial(self.port,self.baudrate, timeout=0.5)
        
        self.serial_timer = self.create_timer(0.001, self.serial_timer_callback)
        self.publish_timer = self.create_timer(0.01, self.publish_timer_callback)
        
    def _load_params(self):
        self.baudrate = self.get_parameter('baudrate').value
        self.port = self.get_parameter('port').value
        self.position_zero = self.get_parameter('position_zero').value
        self.height_zero = self.get_parameter('height_zero').value
        self.tilt_zero = self.get_parameter('tilt_zero').value
        self.position_dir = self.get_parameter('position_dir').value
        self.height_dir = self.get_parameter('height_dir').value
        self.tilt_dir = self.get_parameter('tilt_dir').value

    def home_callback(self, msg):
        if msg.data:
            self.is_homing = True
            self.get_logger().info("Homing requested")

    def serial_timer_callback(self):
        if self.serial.in_waiting > 0:
            self.buffer += self.serial.read(self.serial.in_waiting)

        if b'\n' in self.buffer:
            lines = self.buffer.split(b'\n')
            self.buffer = lines[-1]
            
            for line in lines[:-1]:
                try:
                    line_str = line.decode('utf-8').strip()
                    if not line_str:
                        continue
                    data = [int(x) for x in line_str.split()]
                    if len(data) >= 4:
                        now = self.get_clock().now().to_msg()
                        self.readings_buffer.append([now,self.position_dir*data[1]/4096,self.height_dir*data[2]/4096,self.tilt_dir*data[3]/4096])
                except ValueError:
                    pass

    def publish_timer_callback(self):
        if len(self.readings_buffer) < 2:
            return

        # Convert buffer to list of (t, pos, height, tilt)
        samples = []
        for item in self.readings_buffer:
            t = item[0].sec + item[0].nanosec * 1e-9
            samples.append((t, item[1], item[2], item[3]))

        # Filter samples to be within 20ms of the latest sample
        latest_t = samples[-1][0]
        window_start = latest_t - 0.02
        valid_samples = [s for s in samples if s[0] >= window_start]

        if len(valid_samples) < 2:
            return

        # Split into two halves to calculate averages
        mid = len(valid_samples) // 2
        old_half = valid_samples[:mid]
        new_half = valid_samples[mid:]

        def get_avg(data):
            return [sum(col) / len(col) for col in zip(*data)]

        avg_old = get_avg(old_half)
        avg_new = get_avg(new_half)

        dt = avg_new[0] - avg_old[0]
        if dt <= 1e-9:
            return

        vel_pos = (avg_new[1] - avg_old[1]) / dt
        vel_height = (avg_new[2] - avg_old[2]) / dt
        vel_tilt = (avg_new[3] - avg_old[3]) / dt

        latest = self.readings_buffer[-1]

        if self.is_homing:
            self.position_offset = self.position_zero - latest[1]
            self.height_offset = self.height_zero - latest[2]
            self.tilt_offset = self.tilt_zero - latest[3]
            self.is_homing = False
            self.get_logger().info(f"Homing done. Offsets: {self.position_offset}, {self.height_offset}, {self.tilt_offset}")

        self._publish_joint_state(self.position_pub, 'position', latest[1] + self.position_offset, vel_pos, latest[0])
        self._publish_joint_state(self.height_pub, 'height', latest[2] + self.height_offset, vel_height, latest[0])
        self._publish_joint_state(self.tilt_pub, 'tilt', latest[3] + self.tilt_offset, vel_tilt, latest[0])

    def _publish_joint_state(self, pub, name, pos, vel, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [name]
        msg.position = [float(pos)]
        msg.velocity = [float(vel)]
        pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = BaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()