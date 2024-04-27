import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=10
        )
        timer_period = 0.5
        self.timer = self.create_timer(
            timer_period_sec=timer_period,
            callback=self.timer_callback
        )
        self.count = 0 
    
    def timer_callback(self):
        msg = Twist()
        if self.count < 10:
            msg.linear.x = 1.0
            msg.angular.z = 1.0
        elif self.count > 10    :
            msg.linear.x = 2.0
            msg.angular.z = 2.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f"Publicando velocidades para a tartaruga: angular {msg.angular.z} e linear {msg.linear.x}")
        self.count += 1  #

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
