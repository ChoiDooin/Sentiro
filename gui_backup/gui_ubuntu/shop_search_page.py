# shop_search_page.py
import os
import glob
import geopandas as gpd

import sys
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLineEdit,
    QPushButton, QLabel, QStackedWidget, QFrame,
    QApplication, QGridLayout, QSizePolicy, QScrollArea, QMessageBox
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

from jamo import h2j, j2hcj
from hangul_utils import join_jamos
from robot_destination_page import RobotDestinationPage  # 로봇 목적지 페이지 임포트

class OnScreenKeyboardWidget(QWidget):
    """
    화면 내 가상 키보드
    """
    def __init__(self, target: QLineEdit, parent=None, basepage=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        self.target = target
        self.basepage = basepage
        self.is_korean = True
        self.is_shift = False
        self.composed = ''
        self.jamo_buffer = []
        self.vowel_compound_map = {
            ('ㅗ','ㅏ'):'ㅘ', ('ㅗ','ㅐ'):'ㅙ', ('ㅗ','ㅣ'):'ㅚ',
            ('ㅜ','ㅓ'):'ㅝ', ('ㅜ','ㅔ'):'ㅞ', ('ㅜ','ㅣ'):'ㅟ',
            ('ㅡ','ㅣ'):'ㅢ'
        }
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5,5,5,5)
        layout.setSpacing(5)

        # 제어 버튼
        ctrl = QHBoxLayout()
        self.lang_btn = QPushButton("한/영"); self.lang_btn.clicked.connect(self.toggle_language)
        self.shift_btn = QPushButton("Shift"); self.shift_btn.setCheckable(True); self.shift_btn.clicked.connect(self.toggle_shift)
        home_btn = QPushButton("새로고침"); home_btn.clicked.connect(self.on_home)
        close_btn = QPushButton("닫기"); close_btn.clicked.connect(self.on_close)
        ctrl.addWidget(self.lang_btn); ctrl.addWidget(self.shift_btn); ctrl.addWidget(home_btn)
        ctrl.addStretch(); ctrl.addWidget(close_btn)
        layout.addLayout(ctrl)

        # 키 그리드
        self.grid = QGridLayout()
        self.grid.setSpacing(5)
        layout.addLayout(self.grid)
        self.load_keys()

    def load_keys(self):
        # 기존 버튼 제거
        while self.grid.count():
            w = self.grid.takeAt(0).widget()
            if w: w.deleteLater()
        # 키 배열
        num_row = ['1','2','3','4','5','6','7','8','9','0']
        base1 = ['ㅂ','ㅈ','ㄷ','ㄱ','ㅅ','ㅛ','ㅕ','ㅑ','ㅐ','ㅔ']
        base2 = ['ㅁ','ㄴ','ㅇ','ㄹ','ㅎ','ㅗ','ㅓ','ㅏ','ㅣ']
        base3 = ['ㅋ','ㅌ','ㅊ','ㅍ','ㅠ','ㅜ','ㅡ']
        shift1 = ['ㄲ','ㄸ','ㅃ','ㅆ','ㅉ','ㅘ','ㅙ','ㅚ','ㅝ','ㅞ']
        shift2 = ['ㅟ','ㅢ'] + base3
        controls = ['←','Space']
        rows = [num_row]
        if self.is_korean:
            rows += [shift1, shift2] if self.is_shift else [base1, base2]
            rows.append(base3)
        else:
            rows += [list('QWERTYUIOP'), list('ASDFGHJKL'), list('ZXCVBNM')]
        all_rows = rows + [controls]

        for r, row in enumerate(all_rows):
            for c, key in enumerate(row):
                btn = QPushButton(key)
                btn.setFont(QFont("Pretendard",14))
                btn.setFlat(False)
                btn.clicked.connect(lambda _, k=key: self.key_pressed(k))
                if key == 'Space':
                    btn.setFixedHeight(40)
                    self.grid.addWidget(btn, r, c, 1, 1)
                else:
                    btn.setFixedSize(50,50)
                    self.grid.addWidget(btn, r, c)

    def toggle_language(self):
        if self.jamo_buffer:
            self.composed += join_jamos(self.jamo_buffer)
            self.jamo_buffer.clear()
        self.is_korean = not self.is_korean
        self.is_shift = False; self.shift_btn.setChecked(False)
        self.load_keys(); self.target.setText(self.composed)

    def toggle_shift(self):
        self.is_shift = not self.is_shift; self.shift_btn.setChecked(self.is_shift)
        self.load_keys()

    def on_home(self):
        if self.jamo_buffer:
            self.composed += join_jamos(self.jamo_buffer)
            self.jamo_buffer.clear()
        if self.basepage:
            self.basepage.showFloorContent()
        self.hide()

    def on_close(self):
        p = self.basepage or self.parent()
        if hasattr(p, 'search_box'):
            p.search_box.clear()
            p.shop_names = list(p.shop_names_master)
            p.set_row_mode(2); p.build_pages(); p.update_nav_buttons()
        self.hide()

    def key_pressed(self, key: str):
        # 기본 키 처리 로직 유지
        if key == '←':
            if self.jamo_buffer:
                self.jamo_buffer.pop()
            else:
                self.composed = self.composed[:-1]
            text = self.composed + (join_jamos(self.jamo_buffer) if self.is_korean else ''.join(self.jamo_buffer))
            self.target.setText(text)
            return
        if key == 'Space':
            if self.jamo_buffer:
                self.composed += join_jamos(self.jamo_buffer)
                self.jamo_buffer.clear()
            self.composed += ' '
            self.target.setText(self.composed)
            return
        if key.isdigit() or (not self.is_korean and key.isalpha()):
            if self.jamo_buffer:
                self.composed += join_jamos(self.jamo_buffer)
                self.jamo_buffer.clear()
            self.composed += key
            self.target.setText(self.composed)
            return
        if self.is_korean:
            if self.jamo_buffer:
                prev = self.jamo_buffer[-1]
                if (prev, key) in self.vowel_compound_map:
                    self.jamo_buffer[-1] = self.vowel_compound_map[(prev, key)]
                    comp = join_jamos(self.jamo_buffer)
                    self.target.setText(self.composed + comp)
                    return
            self.jamo_buffer.append(key)
            comp = join_jamos(self.jamo_buffer)
            self.target.setText(self.composed + comp)

class ShopSearchPage(QWidget):
    """
    매장 검색 페이지
    """
    def __init__(self, basepage, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        self.basepage = basepage
        # GeoJSON에서 모든 층의 shop name 읽기
        folder = basepage.geojson_folder
        files = glob.glob(os.path.join(folder, 'shops_*.geojson'))
        names = []
        for f in files:
            df = gpd.read_file(f)
        
            if 'name' in df.columns:
                names.extend(df['name'].dropna().tolist())
        self.shop_names_master = sorted(set(names))
        self.shop_names = list(self.shop_names_master)
        self.rows = 2; self.current_page = 0
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10,10,10,10)
        layout.setSpacing(15)

        # 홈 버튼
        home_btn = QPushButton("새로고침")
        home_btn.setFixedSize(80,30)
        home_btn.setStyleSheet("background-color:#ece6cc; border-radius:8px;")
        home_btn.clicked.connect(lambda: self.basepage.showFloorContent())
        layout.addWidget(home_btn, alignment=Qt.AlignmentFlag.AlignRight)

        # 스크롤 영역
        self.stack = QStackedWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(self.stack)
        layout.addWidget(scroll)

        # 네비게이션
        nav = QHBoxLayout(); nav.setSpacing(20); nav.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.prev_btn = QPushButton("‹ PREV"); self.prev_btn.clicked.connect(self.show_prev_page)
        self.page_label = QLabel(); self.page_label.setFont(QFont("Pretendard",12))
        self.next_btn = QPushButton("NEXT ›"); self.next_btn.clicked.connect(self.show_next_page)
        for w in (self.prev_btn, self.page_label, self.next_btn): nav.addWidget(w)
        layout.addLayout(nav)

        # 검색창
        sl = QHBoxLayout()
        self.search_box = QLineEdit(); self.search_box.setPlaceholderText("매장 이름을 입력하세요")
        self.search_box.setFixedHeight(40); self.search_box.setFont(QFont("Pretendard",14))
        self.search_btn = QPushButton("검색"); self.search_btn.setFixedSize(100,40)
        self.search_btn.setStyleSheet("background-color:#ece6cc; border-radius:8px;")
        self.search_btn.clicked.connect(self.perform_search)
        for w in (self.search_box, self.search_btn): sl.addWidget(w)
        layout.addLayout(sl)

        # 키보드
        orig = self.search_box.mousePressEvent
        def custom_mouse_press(e): orig(e); self.keyboard.show(); self.keyboard.raise_()
        self.search_box.mousePressEvent = custom_mouse_press
        self.keyboard = OnScreenKeyboardWidget(self.search_box, parent=self, basepage=self.basepage)
        self.keyboard.hide()
        layout.addWidget(self.keyboard)

        self.build_pages()
        self.update_nav_buttons()
        self.setLayout(layout)

    def set_row_mode(self, rows):
        self.rows = rows; self.build_pages(); self.update_nav_buttons()

    def build_pages(self):
        # 초기화
        while self.stack.count(): w = self.stack.widget(0); self.stack.removeWidget(w); w.deleteLater()
        if not self.shop_names:
            msg = QWidget(); vb = QVBoxLayout(msg)
            lbl = QLabel("해당 장소가 존재하지 않습니다"); lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            vb.addWidget(lbl); self.stack.addWidget(msg); return

        per = self.rows * 5; total = len(self.shop_names)
        pages = (total + per - 1)//per; self.current_page = 0
        for p in range(pages):
            page = QWidget(); grid = QGridLayout(page); grid.setSpacing(10)
            start, end = p*per, min((p+1)*per, total)
            for idx in range(start, end):
                r, c = divmod(idx-start, 5)
                fr = QFrame(); fr.setFixedSize(150,150)
                fr.setStyleSheet("background-color: #ece6cc; border: none;")
                vb = QVBoxLayout(fr); vb.setContentsMargins(0,0,0,0); vb.setSpacing(5)
                vb.addStretch()
                lb = QLabel(self.shop_names[idx]); lb.setFont(QFont("Pretendard",11,QFont.Weight.Bold))
                lb.setAlignment(Qt.AlignmentFlag.AlignCenter)
                vb.addWidget(lb); vb.addStretch()
                hb = QHBoxLayout(); hb.setContentsMargins(0,0,0,0); hb.setSpacing(5)
                btn1 = QPushButton("길찾기"); btn1.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
                btn1.setFlat(False); btn1.clicked.connect(lambda _, n=self.shop_names[idx]: self.start_path(n))
                btn2 = QPushButton("로봇 길찾기"); btn2.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
                btn2.setFlat(False); btn2.clicked.connect(lambda _, n=self.shop_names[idx]: self.start_robot(n))
                hb.addWidget(btn1); hb.addWidget(btn2); vb.addLayout(hb)
                grid.addWidget(fr, r, c)
            self.stack.addWidget(page)
        self.stack.setCurrentIndex(0)

    def show_prev_page(self):
        if self.current_page>0: self.current_page-=1; self.stack.setCurrentIndex(self.current_page); self.update_nav_buttons()

    def show_next_page(self):
        if self.current_page<self.stack.count()-1: self.current_page+=1; self.stack.setCurrentIndex(self.current_page); self.update_nav_buttons()

    def update_nav_buttons(self):
        self.prev_btn.setEnabled(self.current_page>0)
        self.next_btn.setEnabled(self.current_page<self.stack.count()-1)
        self.page_label.setText(f"{self.current_page+1} / {self.stack.count()}")

    def perform_search(self):
        term = self.search_box.text().strip()
        self.shop_names = [s for s in self.shop_names_master if term in s]
        self.set_row_mode(1)

    def start_path(self, name):
        layer = getattr(self.basepage.map_widget, 'layers', {}).get('room_n_stair')
        if layer is not None:
            match = layer[layer['name']==name]
            if not match.empty:
                target = match.geometry.iloc[0].centroid
                self.basepage.showFloorContent()
                self.basepage.map_widget.end_pt = target
                self.basepage.map_widget.draw_path()
                return
        QMessageBox.warning(self, "오류", f"'{name}' 위치를 찾을 수 없습니다.")

    def start_robot(self, name):
        # 로봇 길찾기 클릭 시 바로 목적지 설정 페이지로 이동
        self.basepage.clearLeft()
        rdp = RobotDestinationPage(self.basepage)
        self.basepage.left_layout.addWidget(rdp)
        rdp.set_destination(name)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ShopSearchPage(None)
    w.show()
    sys.exit(app.exec())
