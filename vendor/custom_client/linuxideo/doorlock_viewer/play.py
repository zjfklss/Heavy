import sys
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                                QLabel, QGraphicsDropShadowEffect)
from PySide6.QtCore import Qt, QPropertyAnimation, QEasingCurve, QTimer
from PySide6.QtGui import QFont, QColor

class WeatherCard(QWidget):
    def __init__(self, day, icon, temp_high, temp_low):
        super().__init__()
        self.setFixedSize(120, 160)
        self.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #667eea, stop:1 #764ba2);
                border-radius: 16px;
            }
        """)

        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(20)
        shadow.setColor(QColor(102, 126, 234, 100))
        shadow.setOffset(0, 8)
        self.setGraphicsEffect(shadow)

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        day_label = QLabel(day)
        day_label.setFont(QFont("Segoe UI", 11))
        day_label.setStyleSheet("color: rgba(255,255,255,0.8); background: transparent;")
        day_label.setAlignment(Qt.AlignCenter)

        icon_label = QLabel(icon)
        icon_label.setFont(QFont("Segoe UI Emoji", 36))
        icon_label.setStyleSheet("background: transparent;")
        icon_label.setAlignment(Qt.AlignCenter)

        temp_label = QLabel(f"{temp_high}°")
        temp_label.setFont(QFont("Segoe UI", 20, QFont.Bold))
        temp_label.setStyleSheet("color: white; background: transparent;")
        temp_label.setAlignment(Qt.AlignCenter)

        low_label = QLabel(f"{temp_low}°")
        low_label.setFont(QFont("Segoe UI", 12))
        low_label.setStyleSheet("color: rgba(255,255,255,0.6); background: transparent;")
        low_label.setAlignment(Qt.AlignCenter)

        layout.addWidget(day_label)
        layout.addWidget(icon_label)
        layout.addWidget(temp_label)
        layout.addWidget(low_label)
        self.setLayout(layout)

class WeatherApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("天气预报")
        self.setFixedSize(750, 400)
        self.setStyleSheet("background-color: #0f0c29;")

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(30, 30, 30, 30)

        # 标题区
        title = QLabel("北京 · 天气预报")
        title.setFont(QFont("Microsoft YaHei", 22, QFont.Bold))
        title.setStyleSheet("color: white;")

        subtitle = QLabel("今天感觉不错呢 ☀️")
        subtitle.setFont(QFont("Microsoft YaHei", 12))
        subtitle.setStyleSheet("color: rgba(255,255,255,0.6);")

        # 当前温度
        current = QLabel("26°C")
        current.setFont(QFont("Segoe UI", 48, QFont.Light))
        current.setStyleSheet("color: #667eea;")

        # 天气卡片
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(16)

        weather_data = [
            ("周一", "☀️", 28, 18), ("周二", "⛅", 25, 16),
            ("周三", "🌧️", 22, 14), ("周四", "⛈️", 20, 13),
            ("周五", "🌤️", 26, 17),
        ]

        self.cards = []
        for day, icon, high, low in weather_data:
            card = WeatherCard(day, icon, high, low)
            cards_layout.addWidget(card)
            self.cards.append(card)

        main_layout.addWidget(title)
        main_layout.addWidget(subtitle)
        main_layout.addWidget(current)
        main_layout.addStretch()
        main_layout.addLayout(cards_layout)
        self.setLayout(main_layout)

        # 入场动画
        QTimer.singleShot(100, self.animate_cards)

    def animate_cards(self):
        for i, card in enumerate(self.cards):
            anim = QPropertyAnimation(card, b"windowOpacity")
            anim.setDuration(500)
            anim.setStartValue(0)
            anim.setEndValue(1)
            anim.setEasingCurve(QEasingCurve.OutCubic)
            QTimer.singleShot(i * 100, anim.start)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = WeatherApp()
    window.show()
    sys.exit(app.exec())
