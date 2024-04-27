import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription 
from turtlesim.msg import Pose

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtlecontroller')
        self.subscription = self.create_subscription(  
            msg_type=Pose,
            topic='/turtle1/pose',
            callback=self.pose_callback,
            qos_profile=10
        )
    
    def pose_callback(self, msg):
        self.get_logger().info(f"A tartaruga está em x={msg.x}, y={msg.y} e theta={msg.theta}")

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
