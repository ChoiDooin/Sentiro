# First_page.py

import os
import json                                        # JSON 직렬화용
from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout, QSizePolicy
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPixmap
from constants_v2 import TOUCH_ON, GUIDE_COMPLETE
from gui_publisher_v2 import send_to_ros2


class FirstPage(QWidget):
    def __init__(self, stacked_widget, basepage):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.basepage = basepage                       # Basepage 인스턴스 저장
        self.initUI()

    def initUI(self):
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(0)
        self.setLayout(self.layout)

        # 광고 이미지 파일 목록
        self.images = ["ad1.jpg", "ad2.png", "ad3.png"]
        self.images = [img for img in self.images if os.path.exists(img)]
        self.current_index = 0

        # 이미지 출력용 라벨
        self.label = QLabel()
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label.setStyleSheet("background-color: #000;")
        self.label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # ▶ 이 한 줄 추가 ▶
        # 라벨이 마우스 이벤트를 받지 않고, 부모로 전달하도록 설정
        self.label.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)

        self.layout.addWidget(self.label)

        if self.images:
            self.timer = QTimer()
            self.timer.timeout.connect(self.nextImage)
            self.timer.start(3000)                      # 3초마다 이미지 변경
            self.showImage(self.current_index)
        else:
            self.label.setText("광고 이미지를 불러올 수 없습니다.")
            self.label.setStyleSheet("color: red; font-size: 18px;")

    def showImage(self, index):
        pixmap = QPixmap(self.images[index])
        if pixmap.isNull():
            self.label.setText(f"❌ 이미지 로드 실패: {self.images[index]}")
        else:
            scaled = pixmap.scaled(
                self.label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
            self.label.setPixmap(scaled)

    def nextImage(self):
        self.current_index = (self.current_index + 1) % len(self.images)
        self.showImage(self.current_index)

    def resizeEvent(self, event):
        if self.images:
            self.showImage(self.current_index)
        super().resizeEvent(event)

    def mousePressEvent(self, event):
        # 클릭 시 Basepage 화면으로 전환
        self.stacked_widget.setCurrentIndex(1)

        # 터치 여부 전송
        payload = {"index": TOUCH_ON}
        send_to_ros2(TOUCH_ON)
        
        # 부모 메서드 호출 (선택적)
        super().mousePressEvent(event)
