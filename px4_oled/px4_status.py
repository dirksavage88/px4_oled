import rclpy
import time
import datetime
from rclpy.node import Node
import luma.core.interface.serial as lmcore
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
    def listener_callback(self, msg):
        self.get_logger().info('Preflight Checks Passed: "%s"' % msg.pre_flight_checks_pass)
        self.pub_message = msg.pre_flight_checks_pass
        self.write_oled()
        #battery percent = floor(remaining)
        #see sys_histogram as example

    def oled_init(self):
        lmcore.serial = lmcore.i2c(port=5, address=0x3c)
        lmcore.device = ssd1306(lmcore.serial)
        self.device_handle = lmcore.device

    def oled_draw(self, device, draw_text):
        with canvas(device) as draw:
            draw.rectangle(device.bounding_box, outline="white")
            draw.text((5, 30), draw_text, fill="white")

    def write_oled(self):
        self.oled_init()
        self.oled_draw(self.device_handle, "Arming ready: {}".format(self.pub_message))


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
