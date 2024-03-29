import rclpy
import time
import datetime
from rclpy.node import Node
import luma.core.interface.serial as lmcore
from PIL import ImageFont
from luma.oled.device import ssd1306, ssd1309, ssd1325, ssd1331, sh1106, sh1107, ws0010
from luma.core.render import canvas
from px4_msgs.msg import VehicleStatus 

class Px4Subscriber(Node):
    def __init__(self):
        super().__init__('px4_subscriber')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.listener_callback,
            qos_profile=qos_policy)
   
        self.subscription  # prevent unused variable warning
        self.device_handle = None
        self.pub_message = None
        
        # Pass in oled i2c device info
        self.declare_parameter("i2c_port", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("i2c_address", rclpy.Parameter.Type.INTEGER)

        # Pass in the font size and font file path as params
        # TODO: $USER variable in font path
        self.declare_parameter("font_path", rclpy.Parameter.Type.STRING)
        self.declare_parameter("font_size", 18)

    def listener_callback(self, msg):
        self.get_logger().info('Vehicle Arming Status Ready: "%s"' % msg.pre_flight_checks_pass)
        self.pub_message = msg.pre_flight_checks_pass
        self.write_oled()
    
    def oled_init(self):
        port_id = self.get_parameter("i2c_port").value
        i2c_addr = self.get_parameter("i2c_address").value
        # Open the OLED device at i2c port and address 
        lmcore.serial = lmcore.i2c(port=port_id, address=i2c_addr)
        lmcore.device = ssd1306(lmcore.serial)
        self.device_handle = lmcore.device

    def oled_draw(self, device, draw_text):
        with canvas(device) as draw:
            draw.rectangle(device.bounding_box, outline="white")
            # TODO: try except on font path (if optional)
            font_fp = self.get_parameter("font_path").get_parameter_value().string_value
            font_sz = self.get_parameter("font_size").value
            freepix = ImageFont.truetype(font_fp, font_sz)
            draw.text((3, 3), draw_text, fill="white", font=freepix)

    def write_oled(self):
        self.oled_init()
        if str(self.pub_message) == "False":

            self.oled_draw(self.device_handle, "Arming FAIL")
        
        elif str(self.pub_message) == "True":
            self.oled_draw(self.device_handle, "Arming READY")


def main(args=None):

    rclpy.init(args=args)

    px4_subscriber = Px4Subscriber()
    rclpy.spin(px4_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    px4_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
