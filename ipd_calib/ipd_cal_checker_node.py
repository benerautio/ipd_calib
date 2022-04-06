import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
import os

from os.path import exists

#eventually need to make this some sort of var?
#or pass in some other way?
CAL_FILE = 'cam_cal.txt'
SAVE_PATH = '/home'
USER = os.getlogin()
CAL_PATH = os.path.join(SAVE_PATH, USER,CAL_FILE)

class CheckCalNode(Node):

    def __init__(self):
        super().__init__('ipd_cal_checker')
        self.publisher_ = self.create_publisher(Bool, 'check_cal_exists', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.cal_exists = False

    def timer_callback(self):
        msg = Bool()
        self.cal_exists = exists(CAL_PATH)
        msg.data = self.cal_exists
        self.publisher_.publish(msg)
        self.get_logger().info('Checking for calibration file result:: "%s"' % msg.data)
        self.i += 1
        # self.destroy_node()
        # rclpy.shutdown()
    
    def get_cal_status(self):
        return self.cal_exists


def main(args=None):
    rclpy.init(args=args)

    ipd_check_cal = CheckCalNode()
    counter = 0
    while (ipd_check_cal.get_cal_status() == False):
        print(ipd_check_cal.get_cal_status())
        rclpy.spin_once(ipd_check_cal)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ipd_check_cal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()