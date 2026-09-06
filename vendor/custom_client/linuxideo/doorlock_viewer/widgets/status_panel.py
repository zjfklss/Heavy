from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QLabel, QFormLayout,
    QPushButton, QFrame
)


class StatusPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(200)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        layout.addWidget(self._create_connection_group())
        layout.addWidget(self._create_stats_group())
        layout.addWidget(self._create_actions_group())
        layout.addStretch()

    def _create_connection_group(self):
        group = QGroupBox("Connection")
        layout = QFormLayout(group)

        self._status_indicator = QLabel("⚫ Disconnected")
        self._status_indicator.setStyleSheet("color: #888; font-weight: bold;")
        layout.addRow("", self._status_indicator)

        self._host_label = QLabel("--")
        layout.addRow("Host:", self._host_label)

        self._port_label = QLabel("--")
        layout.addRow("Port:", self._port_label)

        self._encode_label = QLabel("--")
        layout.addRow("Encode:", self._encode_label)

        return group

    def _create_stats_group(self):
        group = QGroupBox("Stream Stats")
        layout = QFormLayout(group)

        self._fps_label = QLabel("--")
        self._fps_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #4af;")
        layout.addRow("FPS:", self._fps_label)

        self._kbps_label = QLabel("--")
        layout.addRow("Bitrate:", self._kbps_label)

        self._pkts_label = QLabel("--")
        layout.addRow("Packets:", self._pkts_label)

        self._buf_label = QLabel("--")
        layout.addRow("Buffer:", self._buf_label)

        self._drops_label = QLabel("--")
        layout.addRow("Drops:", self._drops_label)

        self._err_label = QLabel("--")
        layout.addRow("Errors:", self._err_label)

        return group

    def _create_actions_group(self):
        group = QGroupBox("Actions")
        layout = QVBoxLayout(group)

        self._screenshot_btn = QPushButton("Screenshot (Ctrl+S)")
        layout.addWidget(self._screenshot_btn)

        self._settings_btn = QPushButton("Settings...")
        layout.addWidget(self._settings_btn)

        return group

    def update_stats(self, stats: dict):
        connected = stats.get("connected", False)
        if connected:
            self._status_indicator.setText("● Connected")
            self._status_indicator.setStyleSheet("color: #4f8; font-weight: bold;")
        else:
            self._status_indicator.setText("⚫ Disconnected")
            self._status_indicator.setStyleSheet("color: #888; font-weight: bold;")

        self._host_label.setText(stats.get("host", "--"))
        self._port_label.setText(str(stats.get("port", "--")))
        self._encode_label.setText(
            f"{stats.get('encode_w', '?')}x{stats.get('encode_h', '?')}"
        )

        fps = stats.get("fps", 0)
        if fps > 0:
            self._fps_label.setText(f"{fps:.1f}")
        else:
            self._fps_label.setText("--")
            self._fps_label.setStyleSheet(
                "font-size: 18px; font-weight: bold; color: #888;"
            )
        if fps >= 30:
            self._fps_label.setStyleSheet(
                "font-size: 18px; font-weight: bold; color: #4f8;"
            )
        elif fps >= 15:
            self._fps_label.setStyleSheet(
                "font-size: 18px; font-weight: bold; color: #fa4;"
            )
        elif fps > 0:
            self._fps_label.setStyleSheet(
                "font-size: 18px; font-weight: bold; color: #f44;"
            )

        self._kbps_label.setText(f"{stats.get('kbps', 0):.1f} kbps")
        self._pkts_label.setText(f"{stats.get('pkts_hz', 0):.0f} Hz")
        self._buf_label.setText(f"{stats.get('buf_bytes', 0)} B")

        drops = stats.get("drops", 0)
        errs = stats.get("dec_err", 0) + stats.get("sync_loss", 0)
        self._drops_label.setText(f"{drops}" if drops > 0 else "0")
        self._err_label.setText(f"{errs}" if errs > 0 else "0")
