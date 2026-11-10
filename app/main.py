import os

# Suppress verbose C-level OpenCV / FFmpeg stderr warnings
os.environ.setdefault("OPENCV_LOG_LEVEL", "OFF")
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")

import cv2
cv2.setNumThreads(1)

from contextlib import asynccontextmanager
from fastapi import FastAPI, Request, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
from app.api.endpoints import router as api_router, websocket_telemetry
from app.vision.video_manager import video_manager
from app.core.mavlink_controller import mavlink_ctrl
from app.core.gimbal_controller import gimbal_ctrl

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATES_DIR = os.path.join(BASE_DIR, "templates")
STATIC_DIR = os.path.join(BASE_DIR, "static")

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Start vision loop
    video_manager.start()

    yield
    # Shutdown: Stop vision & command UAV hover & center gimbal
    video_manager.stop()
    gimbal_ctrl.stop_tracking()
    gimbal_ctrl.center()
    gimbal_ctrl.disconnect()
    mavlink_ctrl.send_stop()
    mavlink_ctrl.disconnect()

app = FastAPI(
    title="AERO-TRACK GCS",
    description="UAV Visual Servoing & Tracking with ArduPilot MAVLink",
    version="1.0.0",
    lifespan=lifespan
)

# Mount static assets
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

# Jinja2 Templates
templates = Jinja2Templates(directory=TEMPLATES_DIR)

# Include API routes
app.include_router(api_router, prefix="/api")

# Mount WebSocket endpoint on both /ws/telemetry and /api/ws/telemetry
@app.websocket("/ws/telemetry")
async def ws_root_telemetry(websocket: WebSocket):
    await websocket_telemetry(websocket)

@app.get("/", response_class=HTMLResponse)
async def index_page(request: Request):
    """Serve main GCS control dashboard."""
    return templates.TemplateResponse(request=request, name="index.html")

