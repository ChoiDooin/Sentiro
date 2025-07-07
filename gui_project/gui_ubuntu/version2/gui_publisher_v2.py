# gui_publisher_v2.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

rclpy.init()

class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('gui_publisher_v2')
        self.publisher = self.create_publisher(String, 'gui/ui_event', 10)

    def send_index(self, index: int):
        msg = String()
        msg.data = json.dumps({"index": index})
        self.publisher.publish(msg)

ros_node_v2 = ROS2Publisher()

def send_to_ros2(index: int):
    ros_node_v2.send_index(index)

