# Floor_guide_page.py

import os
from PyQt6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QGridLayout,
    QSizePolicy, QFrame, QPushButton, QHBoxLayout,
    QStackedLayout
)
from PyQt6.QtGui import QFont, QMouseEvent, QPalette, QColor
from PyQt6.QtCore import Qt, pyqtSignal
from gui_map_viewer_start_point_import import MapWidget  # 지도 위젯 임포트

class FloorItem(QFrame):
    """사용자 정의 위젯: 층 안내 아이템"""
    clicked = pyqtSignal(str)

    def __init__(self, floor_label: str, desc: str, code: str):
        super().__init__()
        self.code = code
        # 메뉴 버튼과 동일한 배경색
        self.setStyleSheet("QFrame { background-color: #f5f5dc; }")
        self.setAutoFillBackground(True)

        # 수평 레이아웃: 층 번호 + 설명
        layout = QHBoxLayout(self)
        layout.setContentsMargins(15, 10, 15, 10)
        layout.setSpacing(10)

        lbl_floor = QLabel(floor_label)
        lbl_floor.setFont(QFont("Pretendard", 25, QFont.Weight.Bold))
        lbl_floor.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_floor.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        lbl_desc = QLabel(desc)
        lbl_desc.setFont(QFont("Pretendard", 18))
        lbl_desc.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_desc.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        layout.addWidget(lbl_floor)
        layout.addWidget(lbl_desc)

        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self.setMaximumWidth(600)

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self.code)
        super().mousePressEvent(event)


class FloorGuidePage(QWidget):
    """층별 안내: 목록 클릭 시 상세(텍스트+지도)만 보이도록 분리"""
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.initUI()

    def initUI(self):
        # 스택 레이아웃: 0=목록, 1=상세
        self.stack = QStackedLayout(self)
        # 1) 목록 페이지 생성
        self.buildListPage()
        # 2) 상세 페이지 생성
        self.buildDetailPage()
        # 초기 화면: 목록
        self.stack.setCurrentWidget(self.list_page)

    def buildListPage(self):
        self.list_page = QWidget()
        # 배경색 설정
        pal = self.list_page.palette()
        pal.setColor(QPalette.ColorRole.Window, QColor('#f5f5dc'))
        self.list_page.setPalette(pal)
        self.list_page.setAutoFillBackground(True)

        main_layout = QVBoxLayout(self.list_page)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 제목 위젯
        title_w = QWidget()
        title_w.setAutoFillBackground(True)
        tp = title_w.palette()
        tp.setColor(QPalette.ColorRole.Window, QColor('#f5f5dc'))
        title_w.setPalette(tp)
        tl = QVBoxLayout(title_w)
        tl.setContentsMargins(20, 20, 0, 0)
        tl.setSpacing(10)
        lbl = QLabel("층별 안내")
        lbl.setFont(QFont("Pretendard", 24, QFont.Weight.Bold))
        tl.addWidget(lbl)
        main_layout.addWidget(title_w)

        # 층 리스트 정보
        self.floors_info = [
            ("1층", "패션,잡화", "1F"),
            ("2층", "여성 패션", "2F")
        ]
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(0)
        total = len(self.floors_info)
        for idx, (fl, ds, cd) in enumerate(self.floors_info):
            row = total - 1 - idx
            item = FloorItem(fl, ds, cd)
            item.clicked.connect(self.goToFloor)
            grid.addWidget(item, row, 0)

        container = QWidget()
        container.setAutoFillBackground(True)
        cp = container.palette()
        cp.setColor(QPalette.ColorRole.Window, QColor('#f5f5dc'))
        container.setPalette(cp)
        container.setLayout(grid)
        container.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        container.setMaximumWidth(640)
        main_layout.addWidget(container, alignment=Qt.AlignmentFlag.AlignHCenter)

        # 뒤로가기 버튼(Base_page)
        back_btn = QPushButton("뒤로가기")
        back_btn.setStyleSheet("""
            QPushButton {
                background-color: #f5f5dc;
                color: #222;
                font-size: 18px;
                border: none;
                padding: 8px 16px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #ffffff;
            }
        """)
        back_btn.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        back_btn.clicked.connect(self.goBack)
        main_layout.addWidget(back_btn, alignment=Qt.AlignmentFlag.AlignRight)

        self.stack.addWidget(self.list_page)

    def buildDetailPage(self):
        self.detail_page = QWidget()
        # 배경색 설정
        pal = self.detail_page.palette()
        pal.setColor(QPalette.ColorRole.Window, QColor('#f5f5dc'))
        self.detail_page.setPalette(pal)
        self.detail_page.setAutoFillBackground(True)

        # 상세 레이아웃
        self.detail_layout = QVBoxLayout(self.detail_page)
        self.detail_layout.setContentsMargins(20, 20, 20, 20)
        self.detail_layout.setSpacing(10)

        self.stack.addWidget(self.detail_page)

    def goToFloor(self, floor_code: str):
        # 1) 기존 상세 내용 
        for i in reversed(range(self.detail_layout.count())):
            w = self.detail_layout.itemAt(i).widget()
            if w:
                w.setParent(None)

        # 2) 클릭된 층의 라벨/설명 찾기
        floor_label, desc = next(
            ((fl, ds) for fl, ds, cd in self.floors_info if cd == floor_code),
            (None, None)
        )

        # 3) 층 설명 텍스트 추가
        if floor_label and desc:
            txt = QLabel(f"{floor_label} : {desc}")
            txt.setFont(QFont("Pretendard", 18))
            txt.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.detail_layout.addWidget(txt)

        # 4) Base_page
        if self.stacked_widget:
            base_page = self.stacked_widget.widget(1)
            base_page.selected_floor = floor_code
            base_page.showFloorContent()
            self.stacked_widget.setCurrentIndex(1)

        # 6) 목록으로 돌아가기 버튼
        back_list = QPushButton("목록으로")
        back_list.setStyleSheet("""
            QPushButton {
                background-color: #f5f5dc;
                color: #222;
                font-size: 18px;
                border: none;
                padding: 8px 16px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #ffffff;
            }
        """)
        back_list.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        back_list.clicked.connect(self.goBack)
        self.detail_layout.addWidget(back_list, alignment=Qt.AlignmentFlag.AlignRight)

        # …(스타일 및 시그널 설정)…
        back_list.clicked.connect(lambda: self.stack.setCurrentWidget(self.list_page))
        self.detail_layout.addWidget(back_list, alignment=Qt.AlignmentFlag.AlignRight)

        # 7) 상세 페이지로 전환
        self.stack.setCurrentWidget(self.detail_page)

    def goBack(self):
        """목록 페이지의 ‘뒤로가기’ 클릭 시 Base_page(헤더+메뉴) 표시"""
        if self.stacked_widget:
            self.stacked_widget.setCurrentIndex(1)
