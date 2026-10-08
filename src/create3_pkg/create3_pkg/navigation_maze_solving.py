import math # ROS uses angles in radians 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
    
class NavigationNode(Node):
# Create the navigation node as a ROS 2 Node

    def __init__(self):
        super().__init__('navigation_node')

        # Pledge algorithm variables
        self.mode = "GO_STRAIGHT"
        self.turn_count = 0
        self.preferred_heading = None
        self.previous_heading = None
        
        # Create a subscription to the LiDAR scan topic. The subscription will call the scan_callback function whenever a new LaserScan message is received.
        self.subscription = self.create_subscription(
            LaserScan,              # type of message to subscribe to
            '/robot_8/scan',        # topic name to subscribe to
            self.scan_callback,     # callback function to handle incoming messages
            10
        )

    def scan_callback(self, msg): 
        # This function runs automatically whenever a new LiDAR scan is received.
        # msg contains the LaserScan data, including angles and distance measurements. here "msg" is the data from the LiDAR scan

        # angle_min is the first angle fromLiDAR measurement 
        # angle_increment is how much the angle changes fromo one measurement to the next

        front_index = round((0 - msg.angle_min) / msg.angle_increment)
         # front = 0
        left_index = round((math.pi / 2 - msg.angle_min) / msg.angle_increment)
         # left = +90 = +pi/2 radians
        right_index = round((-math.pi / 2 - msg.angle_min) / msg.angle_increment)
        # right = -90 = -pi/2 radians

        # find the array index for the directions
        # formula: (target angle - starting angle) / angle step

        front_distance = msg.ranges[front_index]
        left_distance = msg.ranges[left_index]
        right_distance = msg.ranges[right_index]

        # Pledge: switch to wall-following when the front is blocked
        if self.mode == "GO_STRAIGHT":
            if front_distance < 0.5:
                self.mode = "FOLLOW_WALL"
                print("Pledge: Wall detected! Starting wall following.")
            else:
                decision = "FORWARD"
                print("Pledge decision:", decision)

        # Decide what to do while following the right wall
        if self.mode == "FOLLOW_WALL":

            if front_distance < 0.5:
                decision = "TURN_LEFT"

            elif right_distance > 0.5:
                decision = "TURN_RIGHT"

            else:
                decision = "FORWARD"

            print("Pledge decision:", decision)

        # print("Front distance:", front_distance)
        # print("Left distance:", left_distance)
        # print("Right distance:", right_distance)

def main(args=None):
    rclpy.init(args=args)

    node = NavigationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()