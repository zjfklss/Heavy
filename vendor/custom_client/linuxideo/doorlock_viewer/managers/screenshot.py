import cv2
import numpy as np
from pathlib import Path
from datetime import datetime
from PySide6.QtCore import QObject, Signal


class ScreenshotManager(QObject):
    captured = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._save_dir = Path.home() / "Pictures" / "doorlock_viewer"
        self._save_dir.mkdir(parents=True, exist_ok=True)

    def set_save_dir(self, path: Path):
        self._save_dir = path
        self._save_dir.mkdir(parents=True, exist_ok=True)

    def capture(self, frame: np.ndarray):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"doorlock_{timestamp}.png"
        filepath = self._save_dir / filename
        cv2.imwrite(str(filepath), frame)
        self.captured.emit(str(filepath))
