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
dummy_x = 3
dummy_y = 25
