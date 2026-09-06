import os
from pathlib import Path


_FONT_DIR_CANDIDATES = (
    Path("/usr/share/fonts/truetype/dejavu"),
    Path("/usr/share/fonts/truetype/noto"),
    Path("/usr/share/fonts/opentype/noto"),
)


def ensure_cv2_qt_fontdir(cv2_module):
    current = os.environ.get("QT_QPA_FONTDIR")
    if current and os.path.isdir(current):
        return current

    candidates = []
    cv2_file = getattr(cv2_module, "__file__", None)
    if cv2_file:
        candidates.append(Path(cv2_file).resolve().parent / "qt" / "fonts")
    candidates.extend(_FONT_DIR_CANDIDATES)

    for candidate in candidates:
        if candidate.is_dir():
            os.environ["QT_QPA_FONTDIR"] = str(candidate)
            return str(candidate)

    os.environ.pop("QT_QPA_FONTDIR", None)
    return None
