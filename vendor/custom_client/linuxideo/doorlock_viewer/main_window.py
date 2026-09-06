import queue
import numpy as np
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QMenuBar, QMenu, QStatusBar, QLabel
)
from PySide6.QtGui import QAction, QActionGroup, QKeySequence, QShortcut

from .receiver import FrameReceiver
from .widgets.video_view import VideoView
from .widgets.status_panel import StatusPanel
from .settings import Settings, SettingsDialog
from .managers.screenshot import ScreenshotManager
from .managers.trail import TrailEffect
from .managers.super_resolution import SuperResolution


class MainWindow(QMainWindow):
    def __init__(self, settings: Settings, receiver=None):
        super().__init__()
        self._settings = settings
        self._params = settings.all_params()
        self._latest_frame = None
        self._frame_lock = False

        self.setWindowTitle("Doorlock Viewer")
        self.setMinimumSize(600, 400)

        # 搭建界面布局
        self._setup_central()   #搭建中央区
        self._setup_menu()      #搭建菜单栏目
        self._setup_statusbar() #搭建状态栏
        self._setup_shortcuts() #搭建快捷键

        if receiver is not None:
            self._receiver = receiver
            self.setWindowTitle("Doorlock Viewer [DEMO]")
        else:
            self._receiver = FrameReceiver()
            self._receiver.apply_params(
                self._params["host"],
                int(self._params["port"]),
                int(self._params["encode_width"]),
                int(self._params["encode_height"])
            )

        # 连接信号槽
        self._receiver.frame_ready.connect(self._on_frame)
        self._receiver.stats_updated.connect(self._on_stats)
        self._receiver.connected.connect(self._on_connected)
        self._receiver.disconnected.connect(self._on_disconnected) 
        self._receiver.start()

        self._screenshot_mgr = ScreenshotManager()
        self._screenshot_mgr.captured.connect(self._on_screenshot_saved)


        self._trail = TrailEffect()
        self._trail.enabled = self._settings.get_bool("trail_enabled")
        self._trail_action.setChecked(self._trail.enabled)

        display_scale = int(self._params.get("display_scale", 3))
        self._base_display_scale = float(display_scale)
        self._sr = SuperResolution(scale=display_scale)
        self._sr.set_mode(str(self._settings.get("sr_mode") or SuperResolution.MODE_OFF))
        self._sync_sr_menu()
        self._apply_view_scale()
        self._status_sr.setText(self._sr_status_text())
        self._video_view.set_zoom_limits(1.0, 12.0)

        geom = self._settings.get("geometry")
        if geom:
            self.restoreGeometry(geom)
        else:
            self.resize(860, 520)

    def closeEvent(self, event):
        self._settings.set("geometry", self.saveGeometry())
        self._receiver.stop()
        super().closeEvent(event)

    def _setup_menu(self):
        menubar = self.menuBar()

        # 文件菜单设置
        file_menu = menubar.addMenu("&File")
        self._screenshot_action = QAction("Screenshot", self)
        self._screenshot_action.setShortcut(QKeySequence("Ctrl+S"))
        self._screenshot_action.triggered.connect(self._do_screenshot)
        file_menu.addAction(self._screenshot_action)

        file_menu.addSeparator()

        exit_action = QAction("E&xit", self)
        exit_action.setShortcut(QKeySequence("Ctrl+Q"))
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        view_menu = menubar.addMenu("&View")

        self._zoom_in_action = QAction("Zoom In", self)
        self._zoom_in_action.setShortcut(QKeySequence("Ctrl+="))
        self._zoom_in_action.triggered.connect(
            lambda: self._video_view.zoom_to(min(
                self._video_view.current_zoom() * 1.5, 8.0
            ))
        )
        view_menu.addAction(self._zoom_in_action)

        self._zoom_out_action = QAction("Zoom Out", self)
        self._zoom_out_action.setShortcut(QKeySequence("Ctrl+-"))
        self._zoom_out_action.triggered.connect(
            lambda: self._video_view.zoom_to(max(
                self._video_view.current_zoom() / 1.5, 1.0
            ))
        )
        view_menu.addAction(self._zoom_out_action)

        self._zoom_reset_action = QAction("Reset Zoom", self)
        self._zoom_reset_action.setShortcut(QKeySequence("0"))
        self._zoom_reset_action.triggered.connect(self._video_view.reset_zoom)
        view_menu.addAction(self._zoom_reset_action)

        zoom_presets = view_menu.addMenu("Zoom Preset")
        for level in [1, 2, 3, 4, 6, 8]:
            act = QAction(f"{level}x", self)
            act.triggered.connect(lambda checked, l=level: self._video_view.zoom_to(float(l)))
            act.setShortcut(QKeySequence(str(level)))
            zoom_presets.addAction(act)

        view_menu.addSeparator()

        self._trail_action = QAction("Motion Trail", self)
        self._trail_action.setCheckable(True)
        self._trail_action.setShortcut(QKeySequence("T"))
        self._trail_action.toggled.connect(self._on_trail_toggled)
        view_menu.addAction(self._trail_action)

        sr_menu = view_menu.addMenu("Super Resolution")
        self._sr_action_group = QActionGroup(self)
        self._sr_action_group.setExclusive(True)
        self._sr_actions = {}
        for mode in (SuperResolution.MODE_OFF, SuperResolution.MODE_LANCZOS,
                     SuperResolution.MODE_SHARPEN, SuperResolution.MODE_DNN):
            act = QAction(SuperResolution.MODE_LABELS[mode], self)
            act.setCheckable(True)
            act.triggered.connect(lambda checked, m=mode: self._on_sr_mode(m))
            self._sr_action_group.addAction(act)
            sr_menu.addAction(act)
            self._sr_actions[mode] = act

        settings_menu = menubar.addMenu("&Settings")
        settings_action = QAction("Preferences...", self)
        settings_action.setShortcut(QKeySequence("Ctrl+,"))
        settings_action.triggered.connect(self._open_settings)
        settings_menu.addAction(settings_action)

    def _setup_central(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self._video_view = VideoView()
        layout.addWidget(self._video_view, 1)

        self._status_panel = StatusPanel()
        self._status_panel._screenshot_btn.clicked.connect(self._do_screenshot)
        self._status_panel._settings_btn.clicked.connect(self._open_settings)
        layout.addWidget(self._status_panel)

    def _setup_statusbar(self):
        self._statusbar = QStatusBar()
        self.setStatusBar(self._statusbar)

        self._status_fps = QLabel("FPS: --")
        self._status_fps.setMinimumWidth(80)
        self._statusbar.addPermanentWidget(self._status_fps)

        self._status_kbps = QLabel("-- kbps")
        self._status_kbps.setMinimumWidth(80)
        self._statusbar.addPermanentWidget(self._status_kbps)

        self._status_zoom = QLabel("3x")
        self._status_zoom.setMinimumWidth(50)
        self._statusbar.addPermanentWidget(self._status_zoom)

        self._video_view.zoom_changed.connect(
            lambda z: self._status_zoom.setText(f"{z:.1f}x")
        )

        self._status_sr = QLabel("SR: Off")
        self._status_sr.setMinimumWidth(110)
        self._statusbar.addPermanentWidget(self._status_sr)

        self._status_encode = QLabel("--")
        self._status_encode.setMinimumWidth(100)
        self._statusbar.addPermanentWidget(self._status_encode)

    def _setup_shortcuts(self):
        QShortcut(QKeySequence("Escape"), self, activated=self.close)

    def _on_frame(self, img: np.ndarray):
        img = self._trail.process(img)
        img = self._sr.process(img)
        self._latest_frame = img
        self._video_view.set_frame(img)

    def _apply_view_scale(self):
        # process() may already upscale by sr.output_scale(); divide the view's
        # base zoom so the on-screen size stays constant across SR modes.
        view_scale = self._base_display_scale / self._sr.output_scale()
        self._video_view.set_base_scale(view_scale)

    def _sync_sr_menu(self):
        """超分辨率模式菜单状态同步"""
        act = self._sr_actions.get(self._sr.mode)
        if act is not None:
            act.setChecked(True)

    def _on_sr_mode(self, mode: str):
        self._sr.set_mode(mode)
        self._settings.set("sr_mode", mode)
        self._apply_view_scale()
        label = SuperResolution.MODE_LABELS.get(mode, mode)
        if mode == SuperResolution.MODE_DNN and not self._sr.dnn_available():
            err = self._sr.last_error() or "model unavailable"
            self._statusbar.showMessage(
                f"Neural SR unavailable ({err}) — using Lanczos fallback", 6000)
        else:
            self._statusbar.showMessage(f"Super Resolution → {label}", 2500)
        self._status_sr.setText(self._sr_status_text())

    def _sr_status_text(self):
        short = {
            SuperResolution.MODE_OFF: "SR: Off",
            SuperResolution.MODE_LANCZOS: f"SR: Lanczos x{self._sr.scale}",
            SuperResolution.MODE_SHARPEN: f"SR: Sharpen x{self._sr.scale}",
            SuperResolution.MODE_DNN: f"SR: Neural x{self._sr.scale}",
        }
        return short.get(self._sr.mode, "SR: Off")

    def _on_trail_toggled(self, checked: bool):
        self._trail.enabled = checked
        if not checked:
            self._trail.reset()
        self._settings.set("trail_enabled", checked)
        self._statusbar.showMessage(
            f"Motion trail {'ON' if checked else 'OFF'}", 2000)

    def _on_stats(self, stats: dict):
        fps = stats.get("fps", 0)
        kbps = stats.get("kbps", 0)
        self._status_fps.setText(f"FPS: {fps:.1f}" if fps > 0 else "FPS: --")
        self._status_kbps.setText(f"{kbps:.1f} kbps")
        self._status_encode.setText(
            f"{stats.get('encode_w', '?')}x{stats.get('encode_h', '?')}"
        )
        self._status_panel.update_stats(stats)

    def _on_connected(self):
        self._statusbar.showMessage("Connected", 3000)

    def _on_disconnected(self):
        self._statusbar.showMessage("Disconnected", 3000)
        self._status_panel.update_stats({"connected": False})

    def _do_screenshot(self):
        if self._latest_frame is not None:
            self._screenshot_mgr.capture(self._latest_frame)

    def _on_screenshot_saved(self, path: str):
        self._statusbar.showMessage(f"Screenshot saved: {path}", 5000)

    def _open_settings(self):
        dlg = SettingsDialog(self._settings, self._params, self)
        if dlg.exec() == SettingsDialog.Accepted:
            new_params = dlg.current_params()
            self._apply_settings(new_params)

    def _apply_settings(self, params: dict):
        self._params = params
        self._receiver.apply_params(
            params["host"],
            int(params["port"]),
            int(params["encode_width"]),
            int(params["encode_height"]),
        )
        display_scale = int(params.get("display_scale", 3))
        self._base_display_scale = float(display_scale)
        self._sr.set_scale(display_scale)
        self._apply_view_scale()
        self._status_sr.setText(self._sr_status_text())
        self._status_encode.setText(
            f"{params.get('encode_width', 0)}x{params.get('encode_height', 0)}"
        )
