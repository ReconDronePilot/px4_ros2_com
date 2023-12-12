import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math
import time 

B_CROIX=0
B_ROND=1
B_TRIANGLE=2
B_CARRE=3
B_L1=4
B_R1=5
B_L2=6
B_R2=7
B_SAHRE=8
B_OPT=9
B_PS=10
J_LH=0
J_LV=1
J_L2=2
J_RH=3
J_RV=4
J_R2=5
J_BH=6
J_BV=7

class Ps4_control(Node):
    def __init__(self):
        super().__init__('ps4_control')
        self.subscriber = self.create_subscription(Joy,"joy", self.cmd_vel_callback,1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)


    def cmd_vel_callback(self, msg):
        self.get_logger().info("Controle manette")
        speed=10
        #PUBLISH SUR cmd_VEL
        cmdvel=Twist()
        cmdvel.linear.x=msg.axes[J_LV]*speed #J_LV
        cmdvel.angular.z=msg.axes[J_LH]*speed #J_LH

        self.pub_cmd_vel.publish(cmdvel)


def main(args=None):
    rclpy.init(args=args)
    node=Ps4_control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()