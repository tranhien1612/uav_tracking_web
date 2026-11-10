import sys
import os

if __name__ == "__main__":
    # Auto-detect and re-exec in workspace .venv if invoked with system python3
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    VENV_PYTHON = os.path.join(BASE_DIR, ".venv", "bin", "python")
    if os.path.exists(VENV_PYTHON) and sys.executable != VENV_PYTHON:
        os.execv(VENV_PYTHON, [VENV_PYTHON] + sys.argv)

# Suppress verbose C-level OpenCV / FFmpeg stderr warnings (e.g. when RTSP stream is offline)
os.environ.setdefault("OPENCV_LOG_LEVEL", "OFF")
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")

import cv2
cv2.setNumThreads(1)

import uvicorn
from app.core.config import settings

if __name__ == "__main__":
    print(f"Starting AERO-TRACK GCS on http://localhost:{settings.PORT}")
    uvicorn.run(
        "app.main:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=False,
        log_level="info"
    )


