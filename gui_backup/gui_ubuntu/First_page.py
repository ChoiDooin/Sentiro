# First_page.py

import os
import json                                        # JSON 직렬화용
from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout, QSizePolicy
from PyQt6.QtCore import QTimer, Qt,QUrl
from PyQt6.QtGui import QPixmap
from PyQt6.QtMultimedia import QMediaPlayer, QAudioOutput
from PyQt6.QtMultimediaWidgets import QVideoWidget
from constants import TOUCH_ON, GUIDE_COMPLETE



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

        self.video_widget = QVideoWidget()
        self.layout.addWidget(self.video_widget)
        
        # 오디오 출력 설정
        self.audio_output = QAudioOutput()
        self.audio_output.setVolume(1.0)
        self.media_player = QMediaPlayer()
        self.media_player.setVideoOutput(self.video_widget)
        self.media_player.setAudioOutput(self.audio_output)
        
        # 영상 파일 경로 (예: "intro.mp4"는 same folder)
        video_path = os.path.abspath("intro.mp4")
        self.media_player.setSource(QUrl.fromLocalFile(video_path))
        
        # 자동 반복 재생
        self.media_player.mediaStatusChanged.connect(self.handle_media_status)
        self.media_player.play()


    def resizeEvent(self, event):
        self.video_widget.resize(self.size())
        super().resizeEvent(event)


    def mousePressEvent(self, event):
        # 동영상 정지 및 숨김
        self.media_player.stop()             # 비디오 + 오디오 모두 끔
        self.audio_output.setVolume(0)       # 오디오도 무음 처리
        self.video_widget.hide()

        # 화면 전환
        self.stacked_widget.setCurrentIndex(1)

        # 터치 여부 MQTT 전송
        payload = {"index": TOUCH_ON}
        self.basepage.mqtt_client.publish("robot/main_motor", json.dumps(payload))

        super().mousePressEvent(event)

        
    def handle_media_status(self, status):
        if status == QMediaPlayer.MediaStatus.EndOfMedia:
            self.media_player.setPosition(0)
        self.media_player.play()
        
    def restart_video(self):
        self.video_widget.show()
        self.audio_output.setVolume(100)  # 다시 음량 복구
        self.media_player.setPosition(0)
        self.media_player.play()



