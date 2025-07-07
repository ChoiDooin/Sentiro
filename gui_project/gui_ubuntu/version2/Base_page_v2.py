# Base_page_v2.py

import sys
import os
from datetime import datetime

# MQTT용 패키지 import
import paho.mqtt.client as mqtt
import json

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QSizePolicy, QButtonGroup, QStackedLayout 
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont

from gui_map_viewer_start_point_import_v2 import MapWidget
from shop_search_page_v2 import ShopSearchPage
from robot_destination_page_v2 import RobotDestinationPage
from constants_v2 import TOUCH_ON, GUIDE_COMPLETE


class Basepage(QWidget):
    def __init__(self, stacked_widget=None):
        super().__init__()
        self.floor_label = None
        self.stacked_widget = stacked_widget

        # ── 여기에 MQTT client 초기화 코드 추가 ──
        self.mqtt_client = mqtt.Client()  
        # 브로커 IP와 포트는 환경에 맞게 수정하세요
        #self.mqtt_client.connect("192.168.179.50", 1883, 60)  #경한 선배 ip
        self.mqtt_client.connect_async("192.168.219.116", 1883, 60)
        self.mqtt_client.loop_start()

        # geojson 및 이미지 폴더 경로 저장
        base = os.path.dirname(os.path.abspath(__file__))
        self.geojson_folder = os.path.join(base, "geojson")
        self.images_folder = os.path.join(base, "gui_images")

        # 초기 선택 층
        self.selected_floor = "1F"

        self.initUI()


    def initUI(self):
        # 전체 배경 흰색
        self.setStyleSheet("background-color: white;")
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)

        # 메인 레이아웃: 좌(콘텐츠) + 우(메뉴)
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        center_layout = QHBoxLayout()
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(0)

        # 왼쪽 콘텐츠 영역 준비
        self.left_widget = QWidget()
        self.left_layout = QVBoxLayout()
        self.left_layout.setContentsMargins(20, 20, 0, 0)
        self.left_layout.setSpacing(10)
        self.left_widget.setLayout(self.left_layout)
        
        title = QLabel("층별 안내")
        title.setFont(QFont("Pretendard", 24, QFont.Weight.Bold))
        self.left_layout.addWidget(title)

        self.floor_label = QLabel(self.selected_floor)
        self.floor_label.setFont(QFont("Pretendard", 36))
        self.left_layout.addWidget(self.floor_label)
        
        self.map_container = QWidget()
        self.map_container.setLayout(QStackedLayout())
        self.left_layout.addWidget(self.map_container)

        # 최초 실행 시 “층별안내” 화면 표시
        self.showFloorContent()

        center_layout.addWidget(self.left_widget, 85)

        # 오른쪽 메뉴 패널
        menu_panel = self.createRightMenu()
        menu_panel.setSizePolicy(
            QSizePolicy.Policy.Minimum,
            QSizePolicy.Policy.Expanding
        )
        center_layout.addWidget(menu_panel, 15)

        main_layout.addLayout(center_layout)
        self.setLayout(main_layout)


    def getFloorButtonStyle(self, is_checked):
        # 층 버튼 스타일
        return f"""
            QPushButton {{
                background-color: {'#555555' if is_checked else '#dddddd'};
                color: {'white' if is_checked else 'black'};
                font-weight: bold;
                border-radius: 8px;
                padding: 6px 12px;
            }}
        """


    def changeFloor(self, floor, button):
        """우측 층 버튼 클릭 시 호출"""
        self.selected_floor = floor
        # 버튼 스타일 업데이트
        for btn in self.floor_button_group.buttons():
            chk = (btn == button)
            btn.setChecked(chk)
            btn.setStyleSheet(self.getFloorButtonStyle(chk))
        # 좌측에 맵만 바뀌게
        self.showFloorContent()


    def clearLeft(self):
        """좌측 레이아웃의 모든 위젯 제거"""
        for i in reversed(range(self.left_layout.count())):
            w = self.left_layout.itemAt(i).widget()
            if w:
                self.left_layout.removeWidget(w)
                w.setParent(None)


    def showFloorContent(self):
        """‘층별안내’ 화면 구성 (맵 + 층라벨)"""
        
        # 제목·레이블만 지우고, map_container는 그대로 사용
        # clearLeft() 대신 맵 스택만 초기화
        # self.clearLeft()
        
        self.floor_label.setText(self.selected_floor)

        stack = self.map_container.layout()
        while stack.count():
            w = stack.widget(0)
            stack.removeWidget(w)
            w.setParent(None)        

        # MapWidget 삽입 (지도)
        mw = MapWidget(self.geojson_folder, self.images_folder, floor_code=self.selected_floor)
        mw.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        mw.view.scale(1.3, 1.3)
        self.map_widget = mw
        
        
        stack.addWidget(mw)
        stack.setCurrentWidget(mw)
        # map_widget 속성에 저장하여 shop_search_page 등에서 참조 가능
        #self.map_widget = map_w
        #map_w.setSizePolicy(
            #QSizePolicy.Policy.Expanding,
            #QSizePolicy.Policy.Expanding
        #)
        #map_w.view.scale(1.3, 1.3)
        #self.left_layout.addWidget(map_w)


    def showShopContent(self):
        """‘매장검색’ 화면 구성 (검색창 + 매장 카드)"""
        self.clearLeft()

        # 제목
        title = QLabel("매장 검색")
        title.setFont(QFont("Pretendard", 24, QFont.Weight.Bold))
        self.left_layout.addWidget(title)

        # ShopSearchPage 위젯 삽입
        shop_page = ShopSearchPage(self)
        self.left_layout.addWidget(shop_page)


    def showFloorGuide(self):
        """스택 위젯을 FloorGuidePage로 전환"""
        if self.stacked_widget:
            self.stacked_widget.setCurrentIndex(2)  # Floor_guide_page 인덱스


    def createRightMenu(self):
        panel = QWidget()
        panel.setStyleSheet("background-color: #ece6cc;")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        # 1) 층 선택 버튼
        row = QWidget()
        hbox = QHBoxLayout(row)
        hbox.setContentsMargins(10, 10, 10, 0)
        hbox.setSpacing(10)
        hbox.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.floor_button_group = QButtonGroup(self)
        for fl in ["1F","2F"]:
            btn = QPushButton(fl)
            btn.setCheckable(True)
            btn.setChecked(fl == self.selected_floor)
            btn.setStyleSheet(self.getFloorButtonStyle(btn.isChecked()))
            btn.clicked.connect(lambda _, f=fl, b=btn: self.changeFloor(f, b))
            self.floor_button_group.addButton(btn)
            hbox.addWidget(btn)
        layout.addWidget(row)

        # 2) 로고·날짜·시간·날씨
        header = QWidget()
        vhead = QVBoxLayout(header)
        vhead.setAlignment(Qt.AlignmentFlag.AlignCenter)

        mall = QLabel("HY\nMALL")
        mall.setFont(QFont("Pretendard", 36, QFont.Weight.Bold))
        mall.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vhead.addWidget(mall)

        self.date_label = QLabel()
        self.date_label.setFont(QFont("Pretendard", 14))
        self.date_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vhead.addWidget(self.date_label)

        self.time_label = QLabel()
        self.time_label.setFont(QFont("Pretendard", 16))
        self.time_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vhead.addWidget(self.time_label)

        weather = QLabel("🌤️ 19°C")
        weather.setFont(QFont("Pretendard", 14))
        weather.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vhead.addWidget(weather)

        layout.addWidget(header, 4)

        # 3) 시계 타이머
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateTime)
        self.timer.start(1000)
        self.updateTime()

        # 4) 메뉴 버튼 헬퍼
        def createMenuButton(txt, handler):
            b = QPushButton(txt)
            b.setStyleSheet("""
                QPushButton {
                    background-color: #f5fdc;
                    font-size: 18px;
                    border: none;
                    border-radius: 8px;
                    margin: 5px;
                }
                QPushButton:hover { background-color: #ffffff; }
            """)
            b.clicked.connect(handler)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            return b

        # 5) 메뉴 버튼들
        layout.addWidget(createMenuButton("층별안내", self.showFloorGuide), 2)
        layout.addWidget(createMenuButton("매장검색", self.showShopContent), 2)
        layout.addWidget(createMenuButton("로봇 길안내", self.showRobotDestination), 2)

        # 6) 언어 선택
        lang_w = QWidget()
        vlang = QVBoxLayout(lang_w)
        vlang.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vlang.addWidget(QLabel("Language", alignment=Qt.AlignmentFlag.AlignCenter))

        hlang = QHBoxLayout()
        ko = QPushButton("가"); en = QPushButton("A")
        for btn in (ko, en):
            btn.setCheckable(True)
            btn.setFixedSize(25, 25)
            btn.setStyleSheet("""
                QPushButton { border: 1px solid #aaa; background-color: #f5fdc; }
                QPushButton:checked { background-color: #ffffff; }
            """)
            hlang.addWidget(btn)
        bg = QButtonGroup(self)
        bg.setExclusive(True)
        bg.addButton(ko); bg.addButton(en)
        ko.setChecked(True)
        vlang.addLayout(hlang)
        layout.addWidget(lang_w)

        # 7) Home 버튼
        layout.addStretch()
        refresh_btn = QPushButton("Home")
        refresh_btn.setStyleSheet("background-color:#f5fdc; border:none; padding:6px 12px; border-radius:8px;")
        refresh_btn.clicked.connect(self.refresh_to_base)
        layout.addWidget(refresh_btn, alignment=Qt.AlignmentFlag.AlignRight)

        return panel
    
    def reset_to_initial_view(self):
        """Base_page 초기 화면(층별안내 지도)으로 완전히 되돌림"""
        self.clearLeft()

        # 제목
        title = QLabel("층별 안내")
        title.setFont(QFont("Pretendard", 24, QFont.Weight.Bold))
        self.left_layout.addWidget(title)

        # 층 라벨
        self.floor_label = QLabel(self.selected_floor)
        self.floor_label.setFont(QFont("Pretendard", 36))
        self.left_layout.addWidget(self.floor_label)

        # 맵 컨테이너
        self.map_container = QWidget()
        self.map_container.setLayout(QStackedLayout())
        self.left_layout.addWidget(self.map_container)

        # 지도 출력
        self.showFloorContent()

    
    def refresh_to_base(self):
        """Home 버튼 클릭 시 Base_page로 전환"""
        if self.stacked_widget:
            self.stacked_widget.setCurrentIndex(1)  # index 1 = Base_page
        self.reset_to_initial_view()


    def updateTime(self):
        """날짜·시간 갱신"""
        now = datetime.now()
        self.date_label.setText(now.strftime("%Y-%m-%d (%a)"))
        self.time_label.setText(now.strftime("%p %I:%M"))


    def showRobotDestination(self):
        self.clearLeft()
        self.left_layout.addWidget(RobotDestinationPage(self))


    def showEvent(self, event):
        super().showEvent(event)
        # 페이지가 보일 때마다 층별안내 화면으로 초기화
        self.showFloorContent()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = Basepage()
    w.setWindowFlags(Qt.WindowType.FramelessWindowHint)
    w.showFullScreen()
    sys.exit(app.exec())
