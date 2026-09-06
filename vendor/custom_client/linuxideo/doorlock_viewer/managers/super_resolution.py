import os
import numpy as np
import cv2


class SuperResolution:
    """Receiver-side display enhancement / upscaling.

    Four selectable modes (A/B/C/D), display-only, zero bandwidth cost:
      A "off"      - no processing; the view scales the small frame on the GPU.
      B "lanczos"  - CPU Lanczos4 upscale (sharper than GPU bilinear).
      C "sharpen"  - Lanczos4 upscale + unsharp-mask sharpening.
      D "dnn"      - OpenCV DNN super-resolution (FSRCNN), real detail synthesis.

    When a mode other than "off" is active, process() returns a frame already
    upscaled by `scale`x, so the view's base zoom must be divided by `scale`
    to keep the on-screen size constant (handled by the caller).
    """

    MODE_OFF = "off"
    MODE_LANCZOS = "lanczos"
    MODE_SHARPEN = "sharpen"
    MODE_DNN = "dnn"

    MODE_LABELS = {
        MODE_OFF: "A: Off (GPU scale)",
        MODE_LANCZOS: "B: Lanczos",
        MODE_SHARPEN: "C: Lanczos + Sharpen",
        MODE_DNN: "D: Neural (FSRCNN)",
    }

    def __init__(self, scale=3, model_dir=None):
        self.mode = self.MODE_OFF
        self.scale = int(scale)
        self.sharpen_amount = 0.6
        self._model_dir = model_dir or os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "models")

        self._dnn = None
        self._dnn_scale = None
        self._dnn_error = None

    def output_scale(self):
        """Resolution multiplier process() applies to the frame."""
        return 1 if self.mode == self.MODE_OFF else self.scale

    def set_scale(self, scale):
        self.scale = int(scale)
        if self.mode == self.MODE_DNN:
            self._ensure_dnn()

    def set_mode(self, mode):
        if mode not in self.MODE_LABELS:
            return
        self.mode = mode
        if mode == self.MODE_DNN:
            self._ensure_dnn()

    def dnn_available(self):
        return self._dnn is not None and self._dnn_scale == self.scale

    def last_error(self):
        return self._dnn_error

    def _model_path(self):
        return os.path.join(self._model_dir, f"FSRCNN_x{self.scale}.pb")

    def _ensure_dnn(self):
        if self._dnn is not None and self._dnn_scale == self.scale:
            return
        self._dnn = None
        self._dnn_scale = None
        self._dnn_error = None

        if not hasattr(cv2, "dnn_superres"):
            self._dnn_error = "OpenCV built without dnn_superres"
            return

        path = self._model_path()
        if not os.path.isfile(path):
            self._dnn_error = f"model not found: {path}"
            return

        try:
            sr = cv2.dnn_superres.DnnSuperResImpl_create()
            sr.readModel(path)
            sr.setModel("fsrcnn", self.scale)
            self._dnn = sr
            self._dnn_scale = self.scale
        except Exception as e:
            self._dnn_error = f"DNN load failed: {e}"
            self._dnn = None

    def _lanczos(self, frame):
        h, w = frame.shape[:2]
        return cv2.resize(
            frame, (w * self.scale, h * self.scale),
            interpolation=cv2.INTER_LANCZOS4)

    def _sharpen(self, frame):
        blur = cv2.GaussianBlur(frame, (0, 0), 1.0)
        return cv2.addWeighted(
            frame, 1.0 + self.sharpen_amount, blur, -self.sharpen_amount, 0)

    def process(self, frame: np.ndarray) -> np.ndarray:
        if frame is None or self.mode == self.MODE_OFF:
            return frame

        if self.mode == self.MODE_LANCZOS:
            return self._lanczos(frame)

        if self.mode == self.MODE_SHARPEN:
            return self._sharpen(self._lanczos(frame))

        if self.mode == self.MODE_DNN:
            if self.dnn_available():
                try:
                    return self._dnn.upsample(frame)
                except Exception as e:
                    self._dnn_error = f"DNN upsample failed: {e}"
            # Fallback to Lanczos so the display never breaks
            return self._lanczos(frame)

        return frame
