from sensor_msgs.msg import Joy

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
