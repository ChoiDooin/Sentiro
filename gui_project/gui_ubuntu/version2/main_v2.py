# main_v2.py

import sys
import json
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt

QApplication.setAttribute(
    Qt.ApplicationAttribute.AA_SynthesizeMouseForUnhandledTouchEvents, True
)
QApplication.setAttribute(
    Qt.ApplicationAttribute.AA_SynthesizeTouchForUnhandledMouseEvents, True
)

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QStackedWidget
from PyQt6.QtCore import QObject, QEvent, QTimer
from First_page_v2 import FirstPage
from Base_page_v2 import Basepage
from Floor_guide_page_v2 import FloorGuidePage
from constants_v2 import GUIDE_COMPLETE
from gui_publisher_v2 import send_to_ros2


# ESC 누름 시 전체 앱 종료 처리용
class EscapeKeyFilter(QObject):
    def __init__(self, window):
        super().__init__()
        self.window = window

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.KeyPress and event.key() == Qt.Key.Key_Escape:
            self.window.close()
            return True
        return False

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("매장 검색 키오스크")
        self.setAttribute(Qt.WidgetAttribute.WA_AcceptTouchEvents, True)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        self.showFullScreen()

        # 스택 위젯 생성
        self.stacked_widget = QStackedWidget()

        # ─── 순서 중요 ───
        # 1) Basepage 생성 (mqtt_client 초기화 포함)
        self.base_page = Basepage(self.stacked_widget)
        # 2) FirstPage 생성( Basepage 참조 전달 )
        self.first_page = FirstPage(self.stacked_widget, self.base_page)
        # 3) FloorGuidePage 생성
        self.floor_guide_page = FloorGuidePage(self.stacked_widget)

        # 스택에 위젯 추가 (인덱스: 0=First, 1=Base, 2=FloorGuide)
        self.stacked_widget.addWidget(self.first_page)
        self.stacked_widget.addWidget(self.base_page)
        self.stacked_widget.addWidget(self.floor_guide_page)

        # 레이아웃 설정
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        layout.addWidget(self.stacked_widget)

        # 2분 무입력시 안내 완료 False 전송용 타이머
        self.no_input_timer = QTimer(self)
        self.no_input_timer.setInterval(10 * 1000)  # 2분
        self.no_input_timer.timeout.connect(self.no_input_timeout)
        self.no_input_timer.start()
        QApplication.instance().installEventFilter(self)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.close()
            
        self.no_input_timer.start()

    def no_input_timeout(self):
        payload = {"index": GUIDE_COMPLETE}
        send_to_ros2(GUIDE_COMPLETE)


        
    def eventFilter(self, obj, event):
        et = event.type()
        if et in (QEvent.Type.MouseButtonPress, QEvent.Type.KeyPress):
            # 타이머를 다시 10초(실사용 시 2분) 카운트다운
            self.no_input_timer.start()
        # 원래 이벤트 흐름 계속
        return super().eventFilter(obj, event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyApp()
    escape_filter = EscapeKeyFilter(window)
    app.installEventFilter(escape_filter)
    window.show()
    sys.exit(app.exec())
