# main.py

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
from First_page import FirstPage
from Base_page import Basepage
from Floor_guide_page import FloorGuidePage
from constants import GUIDE_COMPLETE


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
        self.base_page = Basepage(self.stacked_widget)
        self.first_page = FirstPage(self.stacked_widget, self.base_page)
        self.floor_guide_page = FloorGuidePage(self.stacked_widget)

        self.stacked_widget.addWidget(self.first_page)       # index 0
        self.stacked_widget.addWidget(self.base_page)        # index 1
        self.stacked_widget.addWidget(self.floor_guide_page) # index 2

        # 전체 레이아웃
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.stacked_widget)

        # 무입력 타이머 (예: 2분 → 여기선 테스트용 10초)
        self.no_input_timer = QTimer(self)
        self.no_input_timer.setInterval(10 * 1000)
        self.no_input_timer.timeout.connect(self.no_input_timeout)
        self.no_input_timer.start()

        QApplication.instance().installEventFilter(self)

    def reset_to_first_page(self):
        """처음 화면으로 돌아갈 때 호출"""
        self.no_input_timer.stop()
        self.stacked_widget.setCurrentIndex(0)
        self.first_page.restart_video()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.reset_to_first_page()
        self.no_input_timer.start()

    def no_input_timeout(self):
        current_index = self.stacked_widget.currentIndex()
        if current_index == 0:
            return
        # 안내 완료 index 전송
        payload = {"index": GUIDE_COMPLETE}
        self.base_page.mqtt_client.publish("robot/main_motor", json.dumps(payload))

        # 화면 초기화 및 동영상 재시작
        self.reset_to_first_page()

    def eventFilter(self, obj, event):
        et = event.type()
        if et in (QEvent.Type.MouseButtonPress, QEvent.Type.KeyPress):
            if self.stacked_widget.currentIndex() != 0:
                self.no_input_timer.start()
        # 원래 이벤트 흐름 계속
        return super().eventFilter(obj, event)
        
    def force_quit(self):
        # 동영상 정지
        self.first_page.media_player.stop()
        self.first_page.audio_output.setVolume(0)  # 오디오도 완전 끔

        # 윈도우 닫기 + 이벤트 루프 종료
        self.close()
        QApplication.quit()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyApp()
    escape_filter = EscapeKeyFilter(window)
    app.installEventFilter(escape_filter)
    window.show()
    sys.exit(app.exec())

