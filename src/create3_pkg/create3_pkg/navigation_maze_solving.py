import math # ROS uses angles in radians 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
    
class NavigationNode(Node):
# Create the navigation node as a ROS 2 Node

    def __init__(self):
        super().__init__('navigation_node')

        # Pledge algorithm variables
        self.mode = "GO_STRAIGHT"
        self.total_rotation = 0.0
        self.preferred_heading = None
        self.previous_heading = None
        self.current_heading = None
        # Remember whether the robot has started turning around a wall
        self.wall_follow_started = False
        
        # # Create a subscription to the LiDAR scan topic. 
        # # The subscription will call the scan_callback function whenever a new LaserScan message is received.
        # self.subscription = self.create_subscription(
        #     LaserScan,              # type of message to subscribe to
        #     '/robot_8/scan',        # topic name to subscribe to
        #     self.scan_callback,     # callback function to handle incoming messages
        #     10
        # )

        # Subscribe to Amad's processed wall distances

        self.amad_subscription = self.create_subscription(
            Float32MultiArray,
            '/robot_8/wall_follower/distances',
            self.amad_wall_callback,
            10
        )

    def update_heading(self, current_heading):

        self.current_heading = current_heading

        # Update robot heading and calculate its rotation
        # this where i add Zoe's code to update the heading of the robot based on its current heading and the preferred heading.

        # Save the first heading measurement
        if self.previous_heading is None:
            self.previous_heading = current_heading
            self.preferred_heading = current_heading
            # Count rotation only when following a wall
            if self.mode == "FOLLOW_WALL":
                self.total_rotation += angle_change

            if abs(self.total_rotation) > 0.1: # Confirm that the robot has started turning
                self.wall_follow_started = True

            return
        
        # Calculate how much the robot has rotated
        angle_change = current_heading - self.previous_heading

        # Correct angle wraparound
        if angle_change > math.pi:
            angle_change = angle_change - 2 * math.pi

        elif angle_change < -math.pi:
            angle_change = angle_change + 2 * math.pi

        # Remember the latest heading for the next measurement
        self.previous_heading = current_heading

    def pledge_decision(self, front_distance, left_distance, right_distance): 

        # Pledge algorithm:
            # Move in a preferred direction until blocked.
            # Follow a wall while tracking the total rotation.
            # Leave the wall when facing the preferred direction
            # and the accumulated rotation returns to zero.
        # This function implements the Pledge algorithm to decide the robot's next move based on the distances to obstacles in front, left, and right
        
        decision = "STOP" # default decision if no conditions are met

        if self.mode == "GO_STRAIGHT":
            if front_distance < 0.5:
                self.mode = "FOLLOW_WALL"
                self.total_rotation = 0.0 # Start counting turns from zero for this wall
                self.wall_follow_started = False
                
                print("Pledge: Wall detected! Starting wall following.")
            else:
                decision = "FORWARD"

        if self.mode == "FOLLOW_WALL":

                # Check whether the robot can leave the wall

            if self.wall_follow_started and self.current_heading is not None:

                # Difference between current and preferred direction
                heading_error = (self.current_heading - self.preferred_heading + math.pi) % (2 * math.pi) - math.pi

                if (
                    abs(self.total_rotation) < 0.1
                    and abs(heading_error) < 0.1
                    and front_distance > 0.5
                ):
                    self.mode = "GO_STRAIGHT"
                    decision = "FORWARD"
                    return decision
        
            if front_distance < 0.5:

                if left_distance > 0.5:
                    decision = "TURN_LEFT"

                elif right_distance > 0.5:
                    decision = "TURN_RIGHT"
                else:
                    decision = "TURN_AROUND"

            elif right_distance > 0.5:
                decision = "TURN_RIGHT"
            else:
                decision = "FORWARD"

        return decision

    # Receive processed LiDAR distances from Amad

    def amad_wall_callback(self, msg):

        front_distance = msg.data[0]
        left_distance = msg.data[1]
        right_distance = msg.data[2]

        decision = self.pledge_decision(
            front_distance,
            left_distance,
            right_distance
        )

        print("Pledge decision:", decision)

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

        # Send LiDAR distances to my Pledge algorithm
        decision = self.pledge_decision(front_distance, left_distance, right_distance)
        print ("Navigation decision:", decision)


def main(args=None):
    rclpy.init(args=args)

    node = NavigationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()