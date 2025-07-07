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
from PyQt6.QtGui import QFont, QPalette, QColor, QPixmap
from constants import TOUCH_ON, GUIDE_COMPLETE

from PyQt6.QtMultimedia import QMediaPlayer, QAudioOutput
from PyQt6.QtCore import QUrl

BASE_SHOP_INDEX = 10  # 원하는 시작 index

class RobotDestinationPage(QWidget):
    """
    로봇 길안내 목적지 설정 페이지
    """
    def __init__(self, basepage, auto_name=None):
        super().__init__()
        self.basepage = basepage
        # 매장 이름 목록 가져오기
        folder = basepage.geojson_folder
        files = [os.path.join(folder, 'shops_pgm_1F.geojson')]
        names = []
        for path in files:
            df = gpd.read_file(path)
            if 'name' in df.columns:
                names.extend(df['name'].tolist())
        self.shop_names = names
        self.shop_coords = {
            "NICE": (-5.25, 3.923),
            "GARA": (-5.25, 3.923),
            "M&H": (-5.25, 3.923)
        }
        # 자동 목적지 설정 모드일 경우
        
        
        if auto_name is not None:
            QTimer.singleShot(0, lambda: self.set_destination(auto_name))
        else:
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

            btn = QPushButton(name)
            btn.setFixedSize(150, 150)
            btn.setFont(QFont("Pretendard", 11, QFont.Weight.Bold))
            btn.setStyleSheet("background-color: #ece6cc; border: none;")
            btn.clicked.connect(lambda _, n=name: self.set_destination(n))

            grid.addWidget(btn, r, c)

        inner_layout.addWidget(grid_widget, alignment=Qt.AlignmentFlag.AlignCenter)


        # 중앙 정렬을 위해 상단·하단 stretch 배치
        outer_layout.addStretch(2)
        outer_layout.addLayout(inner_layout)
        outer_layout.addStretch(3)



    def set_destination(self, name):
        if name not in self.shop_coords:
            print(f"❌ '{name}'에 대한 좌표 정보가 없습니다.")
            return

        x, y = self.shop_coords[name]
        # 목적지 index 찾기
        dest_index = self.shop_names.index(name)

        # ✅ 모든 기존 자식 위젯 제거
        for child in self.findChildren(QWidget):
            child.setParent(None)

        # ✅ 배경색 흰색 적용
        self.setStyleSheet("background-color: white;")

        # ✅ 새 이미지 QLabel 생성
        image_label = QLabel(self)
        image_path = os.path.expanduser("~/gui_project/gui_ubuntu/images/wait.png")
        pixmap = QPixmap(image_path)

        if not pixmap or pixmap.isNull():
            print(f"❌ 이미지 로드 실패: {image_path}")
            return

        # 화면 크기에 맞게 자동 스케일
        pixmap = pixmap.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        image_label.setPixmap(pixmap)
        image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        image_label.setGeometry(0, 0, self.width(), self.height())
        image_label.show()

        # ✅ 안내 시작 예약
        QTimer.singleShot(500, lambda: self._start_guidance(x, y))

    def _start_guidance(self, x, y):
        # 안내 완료 전송
        payload = {"index":GUIDE_COMPLETE}
        self.basepage.publish_local("robot/main_motor", json.dumps(payload))
        #self.basepage.publish_global("robot/main_motor", json.dumps(payload))
        print(f"payload : {payload}")
            
        #self.basepage.publish_global("sub4/mode_coords", json.dumps(payload),qos=1,retain=True)
        
        # 오디오 파일 경로 지정
        wav_path = os.path.expanduser("~/gui_project/gui_ubuntu/robot_guide.wav")

        # 시스템 명령으로 재생 (paplay는 PulseAudio sink를 직접 사용)
        subprocess.Popen(["paplay", wav_path])
        print(f"[DEBUG] Playing audio via paplay: {wav_path}")

        # 60초 후 첫 화면 복귀
        QTimer.singleShot(60000, self.return_to_first_page)




    def return_to_first_page(self):
        if hasattr(self.basepage, 'stacked_widget'):
            # Basepage의 맵 초기화
            if hasattr(self.basepage, 'map_widget'):
                self.basepage.map_widget.start_pt = None
                self.basepage.map_widget.end_pt = None
                self.basepage.map_widget.clear_path()

            #self.basepage.publish_local("user/result", 0)

            # FirstPage로 이동
            self.basepage.stacked_widget.setCurrentIndex(0)

