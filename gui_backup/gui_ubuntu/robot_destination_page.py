# robot_destination_page.py
import os
import glob
import geopandas as gpd
import json
import subprocess
from PyQt6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QGridLayout,
    QFrame, QPushButton, QSizePolicy
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from constants import TOUCH_ON, GUIDE_COMPLETE

from PyQt6.QtMultimedia import QMediaPlayer, QAudioOutput
from PyQt6.QtCore import QUrl

BASE_SHOP_INDEX = 10  # 원하는 시작 index

class RobotDestinationPage(QWidget):
    """
    로봇 길안내 목적지 설정 페이지
    """
    def __init__(self, basepage):
        super().__init__()
        self.basepage = basepage
        # 매장 이름 목록 가져오기
        folder = basepage.geojson_folder
        files = glob.glob(os.path.join(folder, 'shops_*.geojson'))
        names = []
        for path in files:
            df = gpd.read_file(path)
            if 'name' in df.columns:
                names.extend(df['name'].tolist())
        self.shop_names = names
        self.init_ui()

    def init_ui(self):
        self.setStyleSheet("background-color: white;")

        # 전체를 감싸는 외부 수직 레이아웃
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(20, 20, 20, 20)
        outer_layout.setSpacing(0)

        # 수직 중앙 정렬을 위한 가운데 inner 레이아웃
        inner_layout = QVBoxLayout()
        inner_layout.setSpacing(10)
        inner_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # 제목
        title = QLabel("로봇 길안내 목적지 설정")
        title.setFont(QFont("Pretendard", 20, QFont.Weight.Bold)) 
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        inner_layout.addWidget(title)

        # 설명
        info = QLabel(
            "작은 로봇이 원하는 장소로 안내해드립니다.\n"
            "아래에서 목적지를 선택해주세요!"
        )
        info.setFont(QFont("Pretendard", 12))  
        info.setAlignment(Qt.AlignmentFlag.AlignCenter)
        info.setWordWrap(True)
        inner_layout.addWidget(info)

        inner_layout.addSpacing(10)

        # 버튼 그리드
        grid_widget = QWidget()
        grid = QGridLayout(grid_widget)
        grid.setSpacing(8)
        grid.setAlignment(Qt.AlignmentFlag.AlignCenter)

        per_row = 5
        for idx, name in enumerate(self.shop_names):
            r = idx // per_row
            c = idx % per_row
            fr = QFrame()
            fr.setFixedSize(150, 150)
            fr.setStyleSheet("background-color: #ece6cc; border: none;")
            vb = QVBoxLayout(fr)
            vb.setContentsMargins(0, 0, 0, 0)
            vb.setSpacing(5)
            vb.addStretch()

            lbl = QLabel(name)
            lbl.setFont(QFont("Pretendard", 11, QFont.Weight.Bold))
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            vb.addWidget(lbl)
            vb.addStretch()

            btn = QPushButton("목적지로 설정")
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            btn.clicked.connect(lambda _, n=name: self.set_destination(n))
            vb.addWidget(btn)

            grid.addWidget(fr, r, c)

        inner_layout.addWidget(grid_widget, alignment=Qt.AlignmentFlag.AlignCenter)

        # 중앙 정렬을 위해 상단·하단 stretch 배치
        outer_layout.addStretch(2)
        outer_layout.addLayout(inner_layout)
        outer_layout.addStretch(3)



    def set_destination(self, name):
        # 목적지 index 찾기
        dest_index = self.shop_names.index(name)

        # 레이아웃 교체 (기존 코드 유지)
        new_layout = QVBoxLayout()
        new_layout.setContentsMargins(0, 0, 0, 0)
        new_layout.setSpacing(0)
        QWidget().setLayout(self.layout())
        self.setLayout(new_layout)

        self.setStyleSheet("background-color: white;")
        wait_label = QLabel("잠시만 기다려주세요!")
        wait_label.setFont(QFont("Pretendard", 48, QFont.Weight.Bold))
        wait_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        new_layout.addStretch()
        new_layout.addWidget(wait_label, alignment=Qt.AlignmentFlag.AlignCenter)
        new_layout.addStretch()

        # index 번호만 전송
        payload = {"index": BASE_SHOP_INDEX + dest_index}
        self.basepage.mqtt_client.publish("robot/main_server", json.dumps(payload))
        
        # 오디오 파일 경로 지정
        wav_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "robot_guide.wav"))
        
        
        # 시스템 명령으로 재생 (paplay는 PulseAudio sink를 직접 사용)
        subprocess.Popen(["paplay", wav_path])
        print(f"[DEBUG] Playing audio via paplay: {wav_path}")

        # 10초 후 첫 화면 복귀
        QTimer.singleShot(10000, self.return_to_first_page)




    def return_to_first_page(self):
        if hasattr(self.basepage, 'stacked_widget'):
            # Basepage의 맵 초기화
            if hasattr(self.basepage, 'map_widget'):
                self.basepage.map_widget.start_pt = None
                self.basepage.map_widget.end_pt = None
                self.basepage.map_widget.clear_path()

            # 안내 완료 전송
            payload = {"index": GUIDE_COMPLETE}
            self.basepage.mqtt_client.publish("robot/main_motor", json.dumps(payload))

            # FirstPage로 이동
            self.basepage.stacked_widget.setCurrentIndex(0)

