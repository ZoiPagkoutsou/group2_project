

from irobot_create_msgs.msg import IrIntensityVector
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ir_subscriber(Node):

    def __init__(self):
        super().__init__("ir_subscriber")
        
        # Subscribe to the ir_intensity topic, which has a message with type IrIntensityVector
        self.irSubscriber = self.create_subscription(IrIntensityVector,"/robot_8/ir_intensity",self.ir_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # Remember to change /robot_1 by to the namespace of your robot!

    def ir_callback(self,msg):
        print('Message type is:',type(msg))
        print('\n Header data is:', msg.header)
        print('\n The readings data is:',msg.readings)
        newlist = [readings.value for readings in msg.readings]
        print('\n The values are: ', newlist)
   
def main():
    rclpy.init()
    subcriberNode = ir_subscriber()
    rclpy.spin_once(subcriberNode)

if __name__ == '__main__':
    main()