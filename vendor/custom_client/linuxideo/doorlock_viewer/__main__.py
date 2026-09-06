import sys
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from PySide6.QtWidgets import QApplication
from doorlock_viewer.settings import Settings
from doorlock_viewer.main_window import MainWindow


def main():
    parser = argparse.ArgumentParser(description="Doorlock Viewer")
    parser.add_argument("--host", help="MQTT broker host")
    parser.add_argument("--port", type=int, help="MQTT broker port")
    parser.add_argument("--encode-width", type=int, help="Encoding width")
    parser.add_argument("--encode-height", type=int, help="Encoding height")
    parser.add_argument("--display-scale", type=int, help="Display scale")
    parser.add_argument("--demo", metavar="VIDEO",
                        help="Demo mode: simulate the full pipeline from a local video file")
    parser.add_argument("--demo-quality", type=int, default=3,
                        help="WebP quality for demo mode (1-100, default 3)")
    parser.add_argument("--demo-serial-hz", type=int, default=50,
                        help="Simulated serial send rate for demo (default 50)")
    parser.add_argument("--demo-output-fps", type=int, default=60,
                        help="Encoder target fps for demo (default 60)")
    args = parser.parse_args()

    settings = Settings()
    if args.host:
        settings.set("host", args.host)
    if args.port:
        settings.set("port", args.port)
    if args.encode_width:
        settings.set("encode_width", args.encode_width)
    if args.encode_height:
        settings.set("encode_height", args.encode_height)
    if args.display_scale:
        settings.set("display_scale", args.display_scale)

    app = QApplication(sys.argv)
    app.setApplicationName("Doorlock Viewer")
    app.setOrganizationName("jlauto-aim")

    app.setStyleSheet("""
        QMainWindow { background-color: #1a1a1a; }
        QGroupBox {
            color: #ccc;
            border: 1px solid #444;
            border-radius: 4px;
            margin-top: 8px;
            padding-top: 12px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 8px;
            padding: 0 4px;
        }
        QLabel { color: #ccc; }
        QMenuBar { background-color: #222; color: #ccc; }
        QMenuBar::item:selected { background-color: #444; }
        QMenu { background-color: #222; color: #ccc; }
        QMenu::item:selected { background-color: #444; }
        QStatusBar { background-color: #181818; color: #888; }
        QPushButton {
            background-color: #333;
            color: #ccc;
            border: 1px solid #555;
            border-radius: 3px;
            padding: 4px 12px;
        }
        QPushButton:hover { background-color: #444; }
        QPushButton:pressed { background-color: #555; }
        QLineEdit, QSpinBox {
            background-color: #2a2a2a;
            color: #ccc;
            border: 1px solid #555;
            border-radius: 3px;
            padding: 2px 4px;
        }
        QSlider::groove:horizontal {
            height: 6px;
            background: #333;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            width: 14px;
            height: 14px;
            background: #4af;
            border-radius: 7px;
        }
    """)

    receiver = None
    if args.demo:
        from doorlock_viewer.demo_receiver import DemoReceiver
        receiver = DemoReceiver(
            video_path=args.demo,
            encode_width=int(settings.get("encode_width")),
            encode_height=int(settings.get("encode_height")),
            webp_quality=args.demo_quality,
            serial_hz=args.demo_serial_hz,
            output_fps=args.demo_output_fps,
        )

    window = MainWindow(settings, receiver=receiver)
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
