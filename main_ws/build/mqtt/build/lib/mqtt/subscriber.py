import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, Int64
import paho.mqtt.client as mqtt
import struct
import json
from geometry_msgs.msg import PointStamped

class MQTTToROSNode(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros_node')
        self.position_ = self.create_publisher(Point, 'user_position', 10)
        self.gui_ = self.create_publisher(Int64, 'web_data', 10)
        self.fire_ = self.create_publisher(PointStamped, 'clicked_point', 10)
        self.subscribe = self.create_subscription(Float64MultiArray, 'gui_odom', self.listener_callback, 10)
        self.subscribe

        # MQTT 클라이언트 설정
        self.global_client = mqtt.Client()
        self.global_client.on_connect = self.global_on_connect
        self.global_client.on_message = self.global_on_message
        self.global_client.connect("192.168.1.122", 1883, 60)
        self.global_client.loop_start()  # 비동기 처리
        
        self.local_client = mqtt.Client()
        self.local_client.on_connect = self.local_on_connect
        self.local_client.on_message = self.local_on_message
        self.local_client.connect("192.168.1.113", 1883, 60)
        self.local_client.loop_start()  # 비동기 처리

    def listener_callback(self, msg):
        self.get_logger().info(f'📥 수신: "{msg.data}"')
        x = msg.data[0]
        y = msg.data[1]
        payload = {"x" : x, "y" : y}
        self.local_client.publish("robot/main_server", json.dumps(payload))

    def global_on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ MQTT 연결됨") 
        client.subscribe("fire_coords")

    def global_on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            msg = PointStamped()
            msg.header.frame_id = "map"  # 예시 프레임
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = data["x"]
            msg.point.y = data["y"]
            msg.point.z = 0.0
            self.fire_.publish(msg)
            self.get_logger().info(f"🔔 퍼블리시: x={msg.point.x}, y={msg.point.y}")
        except Exception as e:
            self.get_logger().error(f"❌ 파싱 에러: {e}")
            
    def local_on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ MQTT 연결됨") 
        client.subscribe("robot/main_motor")
        client.subscribe("user/position")

    def local_on_message(self, client, userdata, msg):
        try:
            if msg.topic == "robot/main_motor":
                data = json.loads(msg.payload.decode())
                int_data = data["index"]
                msg_ros = Int64()
                msg_ros.data = int(int_data)
                self.gui_.publish(msg_ros)
                self.get_logger().info(f"📥 수신됨 → 목적지 결정됨")
            elif msg.topic == "user/position":
                x, y, z = struct.unpack('ddd', msg.payload)
                point = Point()
                point.x = x
                point.y = y
                point.z = z
                self.position_.publish(point)
                self.get_logger().info(f"📥 수신됨 → center_x:{x:.2f}, center_y:{y:.2f}, depth:{z:.2f}")
        except Exception as e:
            self.get_logger().error(f"❌ 파싱 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MQTTToROSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
