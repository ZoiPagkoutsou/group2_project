import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

def choose_direction(front_distance, left_distance, right_distance):

    safe_distance = 0.5

    if front_distance > safe_distance:
        decision = "FORWARD"
    elif left_distance > safe_distance:
        decision = "LEFT"
    elif right_distance > safe_distance:
        decision = "RIGHT"
    else:
        decision = "TURN_AROUND"

    return decision
    
class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/robot_8/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):

        front_index = round((0 - msg.angle_min) / msg.angle_increment)

        front_distance = msg.ranges[front_index]

        print("Front distance:", front_distance)

def main(args=None):
    rclpy.init(args=args)

    node = NavigationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()