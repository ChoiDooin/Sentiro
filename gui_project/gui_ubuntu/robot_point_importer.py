# # 지도의 실제 월드 좌표 범위 (예: self.walkway.bounds)와
# # GUI 뷰의 픽셀 크기를 미리 계산해 둡니다.
# minx, miny, maxx, maxy = self.walkway.bounds
# gui_width = self.view.viewport().width()
# gui_height = self.view.viewport().height()

# def scale_world_to_gui(world_x: float, world_y: float) -> tuple[float, float]:
#     """
#     실세계(world) 좌표를 GUI 좌표로 변환합니다.
#     """
#     # 미터 단위 → 픽셀 단위 변환 비율
#     scale_x = gui_width  / (maxx - minx)
#     scale_y = gui_height / (maxy - miny)

#     # world 좌표를 0~1 범위로 정규화한 뒤 픽셀로 스케일
#     gui_x = (world_x - minx) * scale_x
#     # y축은 위쪽이 감소이므로 반전 처리
#     gui_y = gui_height - (world_y - miny) * scale_y

#     return gui_x, gui_y

# # --- ROS 콜백 예시 (나중에 사용) ---
# # from geometry_msgs.msg import PoseStamped
# #
# # def ros_callback(msg: PoseStamped):
# #     wx = msg.pose.position.x
# #     wy = msg.pose.position.y
# #     # 스케일링 적용
# #     gx, gy = scale_world_to_gui(wx, wy)
# #     # GUI 위젯에 로봇 아이콘 갱신 호출
# #     w.update_robot_position(gx, gy)
# #
# # node.create_subscription(PoseStamped, '/robot_pose', ros_callback, 10)
# robot_point_importer.py

# —————— 외부(ROS 혹은 테스트)에서 들어오는 원시 월드 좌표(m 단위) ——————


import paho.mqtt.client as mqtt
import json

from PIL import Image

# SLAM 기준 로봇 위치 (미터 단위)
dummy_x = 0
dummy_y = 0

# PGM 메타 정보
origin_x, origin_y = -5.26, -11
resolution = 0.2
image_height = 217  # 현재 PGM 기준

# SLAM → 픽셀 좌표 변환 함수
def slam_to_pixel(slam_x, slam_y, origin_x, origin_y, resolution, image_height):
    px = int((slam_x - origin_x) / resolution)
    py = int((slam_y - origin_y) / resolution) - image_height
    return px, py

# 변환된 픽셀 좌표
start_x, start_y = slam_to_pixel(dummy_x, dummy_y, origin_x, origin_y, resolution, image_height)


def on_connect(client, userdata, flags, rc):
    print("MQTT 연결됨")
    client.subscribe("robot/main_server")

def on_message(client, userdata, msg):
    global dummy_x, dummy_y
    data = json.loads(msg.payload.decode())
    dummy_x = data["x"]
    dummy_y = data["y"]
    
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.1.113", 1883, 60)
client.loop_start()


#TEST_MODE = True  # 테스트용일 때 반드시 True

#dummy_raw_x = 0.0
#dummy_raw_y = 0.0

#if not TEST_MODE:
#    import paho.mqtt.client as mqtt
#    import json

#    dummy_raw_x = 0.0
#    dummy_raw_y = 0.0

#    def on_connect(client, userdata, flags, rc):
#        print("MQTT 연결됨")
#        client.subscribe("robot/main_server")

#    def on_message(client, userdata, msg):
#        global dummy_raw_x, dummy_raw_y
#        data = json.loads(msg.payload.decode())
#        dummy_raw_x = data.get("x",0.0)
#        dummy_raw_y = data.get("y",0.0)
    
#    client = mqtt.Client()
#    client.on_connect = on_connect
#    client.on_message = on_message

#    client.connect("192.168.1.113", 1883, 60)
#    client.loop_start()
    
#else:
#    print("[TEST MODE] MQTT 연결 생략 — 위치 (0.0, 0.0) 사용")



