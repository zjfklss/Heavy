from PySide6.QtCore import QSettings
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout, QLineEdit, QSpinBox,
    QSlider, QDialogButtonBox, QLabel, QHBoxLayout, QGroupBox
)
from PySide6.QtCore import Qt


SETTINGS_ORG = "DoorlockViewer"
SETTINGS_APP = "App"


class Settings:
    def __init__(self):
        self._qs = QSettings(SETTINGS_ORG, SETTINGS_APP)
        self.defaults = {
            "host": "192.168.12.1",
            "port": 3333,
            "encode_width": 120,
            "encode_height": 120,
            "display_scale": 3,
            "trail_enabled": False,
            "sr_mode": "off",
            "geometry": None,
        }

    def get(self, key):
        return self._qs.value(key, self.defaults.get(key))

    def get_bool(self, key):
        val = self._qs.value(key, self.defaults.get(key))
        if isinstance(val, str):
            return val.lower() in ("true", "1", "yes")
        return bool(val)

    def set(self, key, value):
        self._qs.setValue(key, value)

    def all_params(self):
        return {k: self.get(k) for k in self.defaults}


class SettingsDialog(QDialog):
    def __init__(self, settings: Settings, params: dict, parent=None):
        super().__init__(parent)
        self._settings = settings
        self._params = dict(params)
        self.setWindowTitle("Settings")
        self.setMinimumWidth(360)

        layout = QVBoxLayout(self)

        layout.addWidget(self._create_connection_group())
        layout.addWidget(self._create_encode_group())
        layout.addWidget(self._create_display_group())

        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel | QDialogButtonBox.Apply
        )
        button_box.accepted.connect(self._on_ok)
        button_box.rejected.connect(self.reject)
        button_box.button(QDialogButtonBox.Apply).clicked.connect(self._on_apply)
        layout.addWidget(button_box)

        self._load_values()

    def _create_connection_group(self):
        group = QGroupBox("MQTT Connection")
        layout = QFormLayout(group)

        self._host_edit = QLineEdit()
        layout.addRow("Host:", self._host_edit)

        self._port_spin = QSpinBox()
        self._port_spin.setRange(1, 65535)
        layout.addRow("Port:", self._port_spin)

        return group

    def _create_encode_group(self):
        group = QGroupBox("Encoding (must match sender)")
        layout = QFormLayout(group)

        self._encode_w_spin = QSpinBox()
        self._encode_w_spin.setRange(16, 640)
        layout.addRow("Encode Width:", self._encode_w_spin)

        self._encode_h_spin = QSpinBox()
        self._encode_h_spin.setRange(16, 640)
        layout.addRow("Encode Height:", self._encode_h_spin)

        return group

    def _create_display_group(self):
        group = QGroupBox("Display")
        layout = QFormLayout(group)

        self._scale_slider = QSlider(Qt.Horizontal)
        self._scale_slider.setRange(1, 8)
        self._scale_slider.setTickPosition(QSlider.TicksBelow)
        self._scale_slider.setTickInterval(1)
        self._scale_label = QLabel()
        self._scale_slider.valueChanged.connect(
            lambda v: self._scale_label.setText(f"{v}x")
        )
        row = QHBoxLayout()
        row.addWidget(self._scale_slider)
        row.addWidget(self._scale_label)
        layout.addRow("Display Scale:", row)

        return group

    def _load_values(self):
        self._host_edit.setText(str(self._params.get("host", "192.168.12.1")))
        self._port_spin.setValue(int(self._params.get("port", 3333)))
        self._encode_w_spin.setValue(int(self._params.get("encode_width", 120)))
        self._encode_h_spin.setValue(int(self._params.get("encode_height", 120)))
        self._scale_slider.setValue(int(self._params.get("display_scale", 3)))

    def _collect_values(self):
        return {
            "host": self._host_edit.text().strip(),
            "port": self._port_spin.value(),
            "encode_width": self._encode_w_spin.value(),
            "encode_height": self._encode_h_spin.value(),
            "display_scale": self._scale_slider.value(),
        }

    def _on_apply(self):
        vals = self._collect_values()
        for k, v in vals.items():
            self._settings.set(k, v)
        self._params = vals

    def _on_ok(self):
        self._on_apply()
        self.accept()

    def current_params(self):
        return dict(self._params)
