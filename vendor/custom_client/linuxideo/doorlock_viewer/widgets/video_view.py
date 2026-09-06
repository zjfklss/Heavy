from PySide6.QtCore import Qt, QRectF, Signal
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QColor
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
import numpy as np


class VideoView(QGraphicsView):
    zoom_changed = Signal(float)
    mouse_pos_changed = Signal(int, int)
    pixel_picked = Signal(int, int)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._pixmap_item = QGraphicsPixmapItem()
        self._scene.addItem(self._pixmap_item)

        self._overlay_items = []

        self.setRenderHints(
            QPainter.Antialiasing | QPainter.SmoothPixmapTransform
        )
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setFrameStyle(0)
        self.setBackgroundBrush(QColor(22, 22, 22))

        self._base_scale = 3.0
        self._zoom_min = 1.0
        self._zoom_max = 8.0
        self._crosshair_pen = QPen(QColor(230, 190, 235, 200), 1)
        self._crosshair_circle_pen = QPen(QColor(170, 255, 170, 160), 1)

    def set_frame(self, bgr_array: np.ndarray):
        h, w, ch = bgr_array.shape
        bytes_per_line = ch * w
        qimg = QImage(bgr_array.data, w, h, bytes_per_line, QImage.Format_BGR888)
        self._pixmap_item.setPixmap(QPixmap.fromImage(qimg))

    def set_base_scale(self, scale: float):
        self._base_scale = scale
        self.resetTransform()
        self.scale(scale, scale)

    def set_zoom_limits(self, min_zoom: float, max_zoom: float):
        self._zoom_min = min_zoom
        self._zoom_max = max_zoom

    def current_zoom(self) -> float:
        return self.transform().m11()

    def reset_zoom(self):
        self.resetTransform()
        self.scale(self._base_scale, self._base_scale)
        self.zoom_changed.emit(self._base_scale)

    def zoom_to(self, scale: float):
        current = self.current_zoom()
        factor = scale / current
        self.scale(factor, factor)
        self.zoom_changed.emit(self.current_zoom())

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        current = self.current_zoom()
        new_scale = current * factor
        if self._zoom_min <= new_scale <= self._zoom_max:
            self.scale(factor, factor)
            self.zoom_changed.emit(new_scale)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        scene_pos = self.mapToScene(event.pos())
        self.mouse_pos_changed.emit(int(scene_pos.x()), int(scene_pos.y()))

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            scene_pos = self.mapToScene(event.pos())
            self.pixel_picked.emit(int(scene_pos.x()), int(scene_pos.y()))
        super().mousePressEvent(event)

    def drawForeground(self, painter: QPainter, rect: QRectF):
        super().drawForeground(painter, rect)
        view_rect = self.mapToScene(self.viewport().rect()).boundingRect()
        cx = view_rect.center().x()
        cy = view_rect.center().y()
        painter.setPen(self._crosshair_pen)
        painter.drawLine(0, cy, self._pixmap_item.pixmap().width(), cy)
        painter.drawLine(cx, 0, cx, self._pixmap_item.pixmap().height())
        painter.setPen(self._crosshair_circle_pen)
        painter.drawEllipse(view_rect.center(), 24, 24)
