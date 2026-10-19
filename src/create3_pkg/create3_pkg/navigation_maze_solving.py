import math # ROS uses angles in radians 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, String
    
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

        # Subscribe to Amad's processed wall distances

        self.amad_subscription = self.create_subscription(
            Float32MultiArray,
            '/robot_8/wall_follower/distances',
            self.amad_wall_callback,
            10
        )

        # Subscribe to Zoe's heading publisher

        self.heading_subscription = self.create_subscription(
            Float64,
            '/robot_8/heading',
            self.heading_callback,
            10
        )

        # Send navigation decisions to Aida's movement controller

        self.movement_publisher = self.create_publisher(
            String,
            '/movement_command',
            10
        )

        # Receive movement status from Aida

        self.movement_subscription = self.create_subscription(
            String,
            '/movement_status',
            self.movement_status_callback,
            10
        )

        self.movement_busy = False
        self.navigation_paused = False
        # Remember the last movement command
        self.last_command = None

    def heading_callback(self, msg):
        self.update_heading(msg.data)

    def update_heading(self, current_heading):
        self.current_heading = current_heading

        if self.previous_heading is None:
            self.previous_heading = current_heading
            self.preferred_heading = current_heading
            return

        angle_change = current_heading - self.previous_heading

        # Correct angle wraparound
        angle_change = (angle_change + math.pi) % (2 * math.pi) - math.pi

        self.previous_heading = current_heading

        if self.mode == "FOLLOW_WALL":
            self.total_rotation += angle_change

            if abs(self.total_rotation) > 0.1:
                self.wall_follow_started = True

    def movement_status_callback(self, msg):

        status = msg.data

        if status == "BUSY":
            self.movement_busy = True

        elif status == "DONE":
            self.movement_busy = False

        elif status in ["COLLISION", "SAFETY_BLOCKED",
                        "SAFETY_STOPPED", "ERROR", "STOPPED"]:

            self.movement_busy = False
            self.navigation_paused = True

            self.get_logger().warning(
                f"Navigation paused: {status}"
            )

        self.get_logger().info(f"Movement status: {status}")

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
        
            # Right-hand wall-following rule

            if right_distance > 0.5:
                decision = "TURN_RIGHT"

            elif front_distance > 0.5:
                decision = "FORWARD"

            elif left_distance > 0.5:
                decision = "TURN_LEFT"

            else:
                decision = "TURN_AROUND"

        return decision

    # Receive processed LiDAR distances from Amad

    def amad_wall_callback(self, msg):

        # Wait until the previous movement finishes
        if self.movement_busy or self.navigation_paused:
            return

        # Wait for Zoe's heading information
        if self.current_heading is None:
            return

        # Check Amad's distance data
        if len(msg.data) < 3:
            self.get_logger().warning("Missing distance data")
            return

        front_distance = msg.data[0]
        left_distance = msg.data[1]
        right_distance = msg.data[2]

        decision = self.pledge_decision(
            front_distance,
            left_distance,
            right_distance
        )        

        # Send the Pledge decision to Aida
        command_msg = String()
        command_msg.data = decision

        self.last_command = decision
        self.movement_busy = True
        self.movement_publisher.publish(command_msg)

        self.get_logger().info(f"Pledge command sent: {decision}")


def main(args=None):
    rclpy.init(args=args)

    node = NavigationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()