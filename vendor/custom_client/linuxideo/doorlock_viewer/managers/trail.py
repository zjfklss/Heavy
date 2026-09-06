import numpy as np
import cv2


class TrailEffect:
    """Pure receiver-side visual motion trail.

    Detects motion between consecutive decoded frames and keeps a fading
    "ghost" of moving regions on top of the live frame. This is a display-only
    effect: it does not affect bandwidth or the transmitted stream, and can be
    toggled on/off freely.

    Algorithm:
      - Maintain a float accumulator the same size as the frame.
      - Each frame: detect motion (abs-diff vs previous frame, thresholded).
      - Globally decay the accumulator toward black (trails fade over time).
      - Where motion occurs, deposit current pixels into the accumulator using
        a max operation, so a bright moving object leaves a lingering afterimage
        that is NOT wiped when the object moves away.
      - Output = max(live frame, accumulator * strength) so trails appear as
        fading ghosts behind moving objects.
    """

    def __init__(self, motion_threshold=18, fade=0.85, strength=0.6,
                 dilate_px=2):
        self.enabled = False
        self.motion_threshold = motion_threshold
        self.fade = fade            # 0..1, higher = longer trail
        self.strength = strength    # 0..1, trail overlay opacity
        self.dilate_px = dilate_px

        self._prev_gray = None
        self._accum = None          # float32 BGR accumulator
        self._dilate_kernel = None

    def reset(self):
        self._prev_gray = None
        self._accum = None

    def set_params(self, motion_threshold=None, fade=None, strength=None):
        if motion_threshold is not None:
            self.motion_threshold = int(motion_threshold)
        if fade is not None:
            self.fade = float(fade)
        if strength is not None:
            self.strength = float(strength)

    def process(self, frame: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return frame

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_f = frame.astype(np.float32)

        if self._prev_gray is None or self._accum is None \
                or self._accum.shape != frame_f.shape:
            self._prev_gray = gray
            self._accum = np.zeros_like(frame_f)
            return frame

        diff = cv2.absdiff(gray, self._prev_gray)
        _, motion_mask = cv2.threshold(
            diff, self.motion_threshold, 255, cv2.THRESH_BINARY)

        if self.dilate_px > 0:
            if self._dilate_kernel is None:
                k = 2 * self.dilate_px + 1
                self._dilate_kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (k, k))
            motion_mask = cv2.dilate(motion_mask, self._dilate_kernel)

        # Globally decay the accumulator (afterimages fade over time)
        self._accum *= self.fade

        # Where motion occurs, deposit current pixels with a max op so the
        # afterimage lingers even after the object moves on.
        mask = motion_mask.astype(bool)
        if mask.any():
            self._accum[mask] = np.maximum(self._accum[mask], frame_f[mask])

        # Overlay the fading trail on top of the live frame
        out = np.maximum(frame_f, self._accum * self.strength)
        out = np.clip(out, 0, 255).astype(np.uint8)

        self._prev_gray = gray
        return out
