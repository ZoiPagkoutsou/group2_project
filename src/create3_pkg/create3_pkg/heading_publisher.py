#!/usr/bin/env python3
# A simple ROS2 Publisher


import rclpy # ROS Client Library for Python
import math  # Math library of Python
from rclpy.node import Node #import the Node class 

from geometry_msgs.msg import Point #Import Point message type
from nav_msgs.msg import Odometry # import Odometry message type 
from std_msgs.msg import Float64 # Float message type 
from rclpy.qos import qos_profile_sensor_data # Quality of Service Settings 

# Construct the HeadingPublisher class
class HeadingPublisher(Node): 

    def __init__(self):   # Using the init method which will be called when a new object is created
        super().__init__("heading_publisher") 

        # Starting Refrence Values
        self.yaw_ref = None
        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.xref = 0.0; self.yref = 0.0; self.theta_zref = 0.0 
        self.yaw = 0.0
        self.displacement = 0.0
        self.accumulated_yaw = 0.0
        self.relative_x = 0.0
        self.relative_y = 0.0
    
        self.heading_publisher = self.create_publisher(
            msg_type = Float64,
            topic = "/robot_8/heading",
            qos_profile=10,
        ) 

        # Position publisher
        self.position_publisher = self.create_publisher(
            Point,
            "/robot_8/position",
            10
        )
        
        self.accumulated_publisher = self.create_publisher(
            msg_type=Float64,
            topic='/robot_8/accumulated_heading',
            qos_profile=10
        )
        
        self.odom_subscription = self.create_subscription(
            msg_type = Odometry,
            topic = "/robot_8/odom",
            callback = self.odom_callback,
            qos_profile = qos_profile_sensor_data
        )

        publish_rate = 1 # Hz
        self.timer = self.create_timer(
            timer_period_sec = 1/publish_rate, 
            callback=self.timer_callback
        ) 

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )

    # Robots current position (odom frame)
    def odom_callback(self, msg_data: Odometry):

        # Robot's position
        pose = msg_data.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y 

        # Robot's orientation quaternion
        q = pose.orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 -2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4) # in radians 
        self.yaw = yaw_z
        
        if self.yaw_ref is None:
            self.xref = self.x 
            self.yref = self.y
            self.theta_zref = self.yaw 
            self.yaw_ref = yaw_z 
            
        else:
            
            difference = yaw_z - self.yaw_ref
            
            difference = math.atan2(
            math.sin(difference), 
            math.cos(difference)
        )
        
            self.accumulated_yaw += difference 
            self.yaw_ref = yaw_z

        dx = self.x - self.xref
        dy = self.y - self.yref

        self.relative_x = (math.cos(self.theta_zref) * dx + math.sin(self.theta_zref) * dy)
        self.relative_y = (-math.sin(self.theta_zref) * dx + math.cos(self.theta_zref) * dy)
        self.displacement = math.hypot(self.relative_x, self.relative_y)

    def timer_callback(self): 
        heading_msg = Float64()
        heading_msg.data = self.yaw
        self.heading_publisher.publish(heading_msg)

        accumulated_msg = Float64()
        accumulated_msg.data = self.accumulated_yaw 
        self.accumulated_publisher.publish(accumulated_msg)

        position_msg = Point()
        position_msg.x = self.relative_x
        position_msg.y = self.relative_y
        self.position_publisher.publish(position_msg)

        self.get_logger().info(
        f"Publishing heading: {heading_msg.data:.3f} radians, "
        f"accumulated heading: {accumulated_msg.data:.3f} radians, "
        f"x: {self.relative_x:.3f} m, "
        f"y: {self.relative_y:.3f} m "
    )
        
def main(args=None): 
    rclpy.init(args=args)
    heading_publisher = HeadingPublisher()
    try:
        rclpy.spin(heading_publisher)
    except KeyboardInterrupt: 
        pass
    finally: 
        heading_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
