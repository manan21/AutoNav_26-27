import csv
import io
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, LaserScan, NavSatFix
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32
    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False

try:
    from cv_bridge import CvBridge
    _HAS_CV_BRIDGE = True
except ImportError:
    _HAS_CV_BRIDGE = False

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

import numpy as np
from PIL import Image as PILImage

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Polygon


# ── OSM tile helpers ──────────────────────────────────────────────────
_TILE_CACHE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'resource', 'tile_cache',
)
_GPS_VIEW_RADIUS_FT = 100  # feet on each side of current position
_GPS_VIEW_RADIUS_M = _GPS_VIEW_RADIUS_FT * 0.3048  # ≈ 30.48 m
_GPS_TILE_ZOOM = 19  # high zoom for ~100 ft view


def _latlon_to_tile(lat, lon, zoom):
    """Convert lat/lon to OSM tile (x, y)."""
    n = 2 ** zoom
    x = int((lon + 180.0) / 360.0 * n)
    lat_rad = math.radians(lat)
    y = int((1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n)
    return x, y


def _tile_bounds(tx, ty, zoom):
    """Return (lat_min, lat_max, lon_min, lon_max) for a tile."""
    n = 2 ** zoom
    lon_min = tx / n * 360.0 - 180.0
    lon_max = (tx + 1) / n * 360.0 - 180.0
    lat_max = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    lat_min = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (ty + 1) / n))))
    return lat_min, lat_max, lon_min, lon_max


def _fetch_tile(tx, ty, zoom):
    """Download one 256×256 hybrid satellite+road tile, with disk cache."""
    os.makedirs(_TILE_CACHE_DIR, exist_ok=True)
    cache_path = os.path.join(_TILE_CACHE_DIR, f'{zoom}_{tx}_{ty}.png')
    if os.path.exists(cache_path):
        return PILImage.open(cache_path).convert('RGB')
    # Google hybrid: satellite imagery + road/label overlay (lyrs=y)
    url = (f'https://mt1.google.com/vt/lyrs=y&x={tx}&y={ty}&z={zoom}')
    req = urllib.request.Request(url, headers={'User-Agent': 'AutoNavGUI/1.0'})
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = resp.read()
        with open(cache_path, 'wb') as f:
            f.write(data)
        return PILImage.open(io.BytesIO(data)).convert('RGB')
    except Exception:
        return None


def _fetch_map_for_gps(lats, lons, zoom=_GPS_TILE_ZOOM):
    """Fetch & stitch OSM tiles covering the given GPS points.

    Returns (numpy_rgb, (lon_min, lon_max, lat_min, lat_max)) or (None, None).
    """
    if not lats or not lons:
        return None, None
    lat_min_d, lat_max_d = min(lats), max(lats)
    lon_min_d, lon_max_d = min(lons), max(lons)
    # Add padding of ~200 ft in degrees
    pad_lat = _GPS_VIEW_RADIUS_M * 3 / 111320.0
    pad_lon = _GPS_VIEW_RADIUS_M * 3 / (111320.0 * math.cos(math.radians((lat_min_d + lat_max_d) / 2)))
    tx_min, ty_min = _latlon_to_tile(lat_max_d + pad_lat, lon_min_d - pad_lon, zoom)
    tx_max, ty_max = _latlon_to_tile(lat_min_d - pad_lat, lon_max_d + pad_lon, zoom)
    # Clamp tile range to avoid huge downloads
    if (tx_max - tx_min + 1) * (ty_max - ty_min + 1) > 100:
        return None, None

    rows = []
    for ty in range(ty_min, ty_max + 1):
        row_imgs = []
        for tx in range(tx_min, tx_max + 1):
            tile = _fetch_tile(tx, ty, zoom)
            if tile is None:
                return None, None
            row_imgs.append(np.array(tile))
        rows.append(np.concatenate(row_imgs, axis=1))
    img = np.concatenate(rows, axis=0)

    # Compute overall extent
    bnd_lat_min, _, bnd_lon_min, _ = _tile_bounds(tx_min, ty_max + 1, zoom)
    _, bnd_lat_max, _, bnd_lon_max = _tile_bounds(tx_max + 1, ty_min, zoom)
    # Fix: tile_bounds for ty_max+1 gives lat below last tile
    bnd_lat_min2, _, _, _ = _tile_bounds(tx_min, ty_max, zoom)
    _, bnd_lat_max2, _, _ = _tile_bounds(tx_min, ty_min, zoom)
    bnd_lon_min2, _, bnd_lon_min3, _ = _tile_bounds(tx_min, ty_min, zoom)
    _, _, _, bnd_lon_max3 = _tile_bounds(tx_max, ty_min, zoom)

    # Simpler: compute from tile coords directly
    n = 2 ** zoom
    bnd_lon_min = tx_min / n * 360.0 - 180.0
    bnd_lon_max = (tx_max + 1) / n * 360.0 - 180.0
    bnd_lat_max = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty_min / n))))
    bnd_lat_min = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (ty_max + 1) / n))))

    extent = (bnd_lon_min, bnd_lon_max, bnd_lat_min, bnd_lat_max)
    return img, extent

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QStackedWidget,
    QStyle,
    QStyleOptionSlider,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


def _make_dark_canvas(nrows=1, ncols=1, figsize=(3.6, 2.4)):
    """Create a small matplotlib figure + canvas with a dark theme."""
    fig = Figure(figsize=figsize, dpi=80, facecolor='#1e1e1e')
    fig.subplots_adjust(left=0.18, right=0.94, top=0.88, bottom=0.22)
    axes = fig.subplots(nrows, ncols)
    canvas = FigureCanvasQTAgg(fig)
    canvas.setStyleSheet("border: none;")
    return fig, axes, canvas


def _make_mini_canvas(figsize=(1.4, 1.0)):
    """Create a tiny single-axis canvas for individual power traces."""
    fig = Figure(figsize=figsize, dpi=80, facecolor='#1e1e1e')
    fig.subplots_adjust(left=0.22, right=0.94, top=0.78, bottom=0.28)
    ax = fig.add_subplot(111)
    canvas = FigureCanvasQTAgg(fig)
    canvas.setStyleSheet("border: none;")
    return fig, ax, canvas


def _style_ax(ax, title=None):
    """Apply dark styling to a matplotlib axes."""
    ax.set_facecolor('#111111')
    ax.tick_params(colors='#888', labelsize=7)
    for spine in ax.spines.values():
        spine.set_color('#444')
    ax.xaxis.label.set_color('#888')
    ax.yaxis.label.set_color('#888')
    if title:
        ax.set_title(title, fontsize=8, color='#ccc')


class HudWindow(QMainWindow):
    """1920x720 dark-themed HUD window for the AutoNav bar display."""

    def __init__(self, ros_node=None):
        super().__init__()
        self._ros_node = ros_node
        self.setWindowTitle('AutoNav HUD')
        self.resize(1920, 720)
        self.showFullScreen()

        # Dark palette
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(20, 20, 20))
        palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
        palette.setColor(QPalette.Base, QColor(30, 30, 30))
        palette.setColor(QPalette.AlternateBase, QColor(40, 40, 40))
        palette.setColor(QPalette.ToolTipBase, QColor(25, 25, 25))
        palette.setColor(QPalette.ToolTipText, QColor(220, 220, 220))
        palette.setColor(QPalette.Text, QColor(220, 220, 220))
        palette.setColor(QPalette.Button, QColor(40, 40, 40))
        palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
        palette.setColor(QPalette.BrightText, QColor(255, 50, 50))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
        self.setPalette(palette)

        # Playback state
        self._pb_rows = []
        self._pb_duration_ns = 0
        self._pb_timer = None
        self._pb_wall_start = 0.0
        self._pb_row_idx = 0
        self._pb_state = 'idle'
        self._pb_pause_elapsed_ns = 0
        self._pb_elapsed_ns = 0
        self.sensor_value_labels = {}

        # Rolling data buffers for plots
        self._power_buf = {'t': deque(), 'V': deque(), 'I': deque(), 'P': deque()}
        self._gps_buf = {'lat': [], 'lon': []}
        self._odom_buf = {'x': [], 'y': [], 'theta': []}
        self._odom_tri_patch = None
        self._plots_dirty = False

        # Live mode state
        self._live_active = False
        self._live_timer = None      # QTimer for 10 Hz refresh
        self._live_t0 = 0.0          # wall-clock start for power X axis
        _LIVE_GPS_MAXLEN = 100
        _LIVE_ODOM_MAXLEN = 100
        self._live_gps_maxlen = _LIVE_GPS_MAXLEN
        self._live_odom_maxlen = _LIVE_ODOM_MAXLEN

        # Container connection state
        self._container_connected = False
        self._container_name = 'koopa-kingdom'
        self._container_workdir = '/autonav/isaac_ros-dev'
        self._container_user = 'admin'

        # Video playback state (camera + lidar mp4s)
        self._camera_cap = None   # cv2.VideoCapture
        self._lidar_cap = None    # cv2.VideoCapture
        self._cam_im = None       # matplotlib imshow handle
        self._lidar_im = None     # matplotlib imshow handle
        self._video_fps = 30      # must match recorder's VIDEO_FPS

        # --- Central widget + top-level 3-column layout ---
        central = QWidget()
        self.setCentralWidget(central)
        top_layout = QHBoxLayout(central)
        top_layout.setContentsMargins(4, 4, 4, 4)
        top_layout.setSpacing(4)

        # Shared helpers
        section_title_font = QFont()
        section_title_font.setPointSize(14)
        section_title_font.setBold(True)

        button_style = (
            "QPushButton {"
            "  background-color: #2a2a2a; color: #dcdcdc;"
            "  border: 1px solid #555; border-radius: 4px;"
            "  padding: 10px; font-size: 13px;"
            "}"
            "QPushButton:hover { background-color: #3a3a3a; }"
            "QPushButton:pressed { background-color: #1a1a1a; }"
        )

        frame_style = (
            "QFrame#sensorCell {"
            "  border: 1px solid #444;"
            "  background-color: #1e1e1e;"
            "  border-radius: 3px;"
            "}"
        )

        val_label_style = (
            "border: none; color: #0f0; font-size: 11px;"
            " font-family: monospace;"
        )

        section_title_style = "color: #ffffff; border: none;"
        group_label_style = (
            "border: none; font-size: 9px; color: #666;"
            " font-weight: bold; text-transform: uppercase;"
        )

        # =====================================================================
        # SECTION 1: OPTIONS (left column) – uses QStackedWidget for sub-pages
        # =====================================================================
        options_frame = QFrame()
        options_frame.setStyleSheet("QFrame { border: 1px solid #444; }")
        options_outer = QVBoxLayout(options_frame)
        options_outer.setContentsMargins(10, 6, 10, 10)

        header_font = QFont()
        header_font.setPointSize(20)
        header_font.setBold(True)
        lbl_header = QLabel("AutoNav GUI")
        lbl_header.setFont(header_font)
        lbl_header.setAlignment(Qt.AlignCenter)
        lbl_header.setStyleSheet("color: #ffffff; border: none;")
        options_outer.addWidget(lbl_header)

        self._options_stack = QStackedWidget()
        options_outer.addWidget(self._options_stack, stretch=1)

        # --- Page 0: Main OPTIONS ---
        page_main = QWidget()
        options_layout = QVBoxLayout(page_main)
        options_layout.setContentsMargins(6, 0, 6, 0)

        lbl_options = QLabel("OPTIONS")
        lbl_options.setFont(section_title_font)
        lbl_options.setAlignment(Qt.AlignCenter)
        lbl_options.setStyleSheet(section_title_style)
        options_layout.addWidget(lbl_options)

        self._nav_buttons = []  # (QPushButton, base_label, base_style)

        disabled_btn_style = (
            "QPushButton {"
            "  background-color: #1a1a1a; color: #555;"
            "  border: 1px solid #333; border-radius: 4px;"
            "  padding: 10px; font-size: 13px;"
            "}"
        )
        self._disabled_btn_style = disabled_btn_style

        connect_style = (
            button_style.replace("#2a2a2a", "#1a2a3a")
                        .replace("#3a3a3a", "#2a3a4a")
                        .replace("#1a1a1a", "#0a1a2a")
        )
        self.btn_connect = QPushButton("Connect to Container")
        self.btn_connect.setStyleSheet(connect_style)
        self.btn_connect.setFocusPolicy(Qt.NoFocus)
        self.btn_connect.clicked.connect(self._on_connect_container)
        options_layout.addWidget(self.btn_connect)
        self._nav_buttons.append((self.btn_connect, "Connect to Container", connect_style))
        self._connect_style = connect_style

        # -- Container Dependent Actions --
        lbl_dep = QLabel("Container Dependent")
        lbl_dep.setAlignment(Qt.AlignCenter)
        lbl_dep.setStyleSheet(group_label_style)
        options_layout.addWidget(lbl_dep)

        self._container_dots = []

        def _make_container_btn_row(label, style, click_handler):
            row = QHBoxLayout()
            row.setSpacing(6)
            dot = QLabel()
            dot.setFixedSize(10, 10)
            dot.setStyleSheet(
                "background-color: #f44; border-radius: 5px; border: none;"
            )
            btn = QPushButton(label)
            btn.setStyleSheet(style)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.setEnabled(False)
            btn.clicked.connect(click_handler)
            row.addWidget(dot)
            row.addWidget(btn, stretch=1)
            self._container_dots.append(dot)
            return row, btn

        row_launch, self.btn_launch = _make_container_btn_row(
            "Launch/End Processes", disabled_btn_style, self._show_launch_page)
        options_layout.addLayout(row_launch)
        self._nav_buttons.append((self.btn_launch, "Launch/End Processes", button_style))

        row_live, self.btn_live = _make_container_btn_row(
            "Live Mode", disabled_btn_style, self._on_live_clicked)
        options_layout.addLayout(row_live)
        self._nav_buttons.append((self.btn_live, "Live Mode", button_style))

        row_test, self.btn_test = _make_container_btn_row(
            "Test Mode", disabled_btn_style, self._on_test_clicked)
        options_layout.addLayout(row_test)
        self._nav_buttons.append((self.btn_test, "Test Mode", button_style))

        # Map of container-gated buttons to their base labels
        self._container_buttons = {
            self.btn_launch: "Launch/End Processes",
            self.btn_live: "Live Mode",
            self.btn_test: "Test Mode",
        }

        # -- Container Independent Actions --
        lbl_indep = QLabel("Container Independent")
        lbl_indep.setAlignment(Qt.AlignCenter)
        lbl_indep.setStyleSheet(group_label_style)
        options_layout.addWidget(lbl_indep)

        self.btn_playback = QPushButton("Playback Mode")
        self.btn_playback.setStyleSheet(button_style)
        self.btn_playback.setFocusPolicy(Qt.NoFocus)
        self.btn_playback.clicked.connect(self._on_playback_clicked)
        options_layout.addWidget(self.btn_playback)
        self._nav_buttons.append((self.btn_playback, "Playback Mode", button_style))

        options_layout.addStretch()

        quit_style = (
            button_style.replace("#2a2a2a", "#4a1a1a")
                        .replace("#3a3a3a", "#6a2a2a")
                        .replace("#1a1a1a", "#300a0a")
        )
        btn_quit = QPushButton("Quit GUI")
        btn_quit.setStyleSheet(quit_style)
        btn_quit.setFocusPolicy(Qt.NoFocus)
        btn_quit.clicked.connect(QApplication.quit)
        options_layout.addWidget(btn_quit)
        self._nav_buttons.append((btn_quit, "Quit GUI", quit_style))

        self._options_stack.addWidget(page_main)  # index 0

        # --- Page 1: Launch/End Processes ---
        page_launch = QWidget()
        launch_layout = QVBoxLayout(page_launch)
        launch_layout.setContentsMargins(6, 0, 6, 0)

        lbl_launch = QLabel("LAUNCH / END PROCESSES")
        lbl_launch.setFont(section_title_font)
        lbl_launch.setAlignment(Qt.AlignCenter)
        lbl_launch.setStyleSheet(section_title_style)
        launch_layout.addWidget(lbl_launch)

        # Queue status label
        self._queue_label = QLabel("Queue: idle")
        self._queue_label.setStyleSheet(
            "border: none; color: #888; font-size: 10px;"
            " font-family: monospace;"
        )
        self._queue_label.setAlignment(Qt.AlignCenter)
        self._queue_label.setWordWrap(True)
        launch_layout.addWidget(self._queue_label)

        # Toggle style: green border when active
        toggle_on_style = (
            button_style.replace("border: 1px solid #555", "border: 1px solid #0f0")
                        .replace("color: #dcdcdc", "color: #0f0")
        )
        self._toggle_on_style = toggle_on_style

        # Device definitions: (button label, status_dot key(s), real command)
        self._launch_devices = [
            ("Pre-SLAM", ["Encoders", "Arduino", "Motor Ctrl", "CONTROL"],
             "ros2 launch bringup pre_slam.launch.py"),
            ("Camera", ["Camera"], "./config/run-zed.sh"),
            ("Lidar", ["Lidar"], "./config/run-lidar.sh"),
            ("SLAM", ["SLAM"], "ros2 launch slam slam.launch.py"),
            ("LINE DETECT", ["LINE DETECT"], "./config/run-lines.sh"),
            ("NAV2", ["NAV2"], "./config/run-nav2.sh"),
            ("GPS", ["GPS"], "ros2 launch gps_handler gps_publisher.cpp"),
            ("Power PCB", ["Power PCB"],
             "ros2 launch autonav_electrical_publisher electrical_publisher.launch.py"),
        ]

        self._launch_nav_buttons = []  # same tuple format as _nav_buttons
        self._launch_states = {}   # label -> False | 'starting' | True
        self._flash_timers = {}    # label -> QTimer (flashing animation)
        self._startup_timers = {}  # label -> QTimer (single-shot delay)
        self._launch_queue = []    # list of labels waiting to start

        cmd_label_style = (
            "border: none; color: #888; font-size: 10px;"
            " font-family: monospace;"
        )

        # Compact button style for launch grid (narrower in X)
        launch_btn_compact = (
            "QPushButton {"
            "  background-color: #2a2a2a; color: #dcdcdc;"
            "  border: 1px solid #555; border-radius: 4px;"
            "  padding: 10px 4px; font-size: 11px;"
            "}"
            "QPushButton:hover { background-color: #3a3a3a; }"
            "QPushButton:pressed { background-color: #1a1a1a; }"
        )

        # Two-column grid: buttons left, commands right
        launch_grid = QGridLayout()
        launch_grid.setSpacing(4)
        for i, (label, _dot_keys, cmd) in enumerate(self._launch_devices):
            btn = QPushButton(label)
            btn.setStyleSheet(launch_btn_compact)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.setFixedWidth(110)
            btn.clicked.connect(lambda checked=False, l=label: self._toggle_device(l))
            launch_grid.addWidget(btn, i, 0)
            self._launch_nav_buttons.append((btn, label, launch_btn_compact))
            self._launch_states[label] = False

            cmd_lbl = QLabel(cmd)
            cmd_lbl.setStyleSheet(cmd_label_style)
            cmd_lbl.setWordWrap(True)
            launch_grid.addWidget(cmd_lbl, i, 1)

        launch_grid.setColumnStretch(0, 0)
        launch_grid.setColumnStretch(1, 1)
        launch_layout.addLayout(launch_grid)

        # Separator bar
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("background-color: #444; border: none; max-height: 1px;")
        sep.setFixedHeight(1)
        launch_layout.addWidget(sep)

        # "Launch All in Sequence" button
        launch_all_style = (
            button_style.replace("border: 1px solid #555", "border: 1px solid #0af")
                        .replace("color: #dcdcdc", "color: #0af")
        )
        btn_launch_all = QPushButton("Launch All in Sequence")
        btn_launch_all.setStyleSheet(launch_all_style)
        btn_launch_all.setFocusPolicy(Qt.NoFocus)
        btn_launch_all.clicked.connect(self._launch_all_in_sequence)
        launch_layout.addWidget(btn_launch_all)
        self._launch_nav_buttons.append(
            (btn_launch_all, "Launch All in Sequence", launch_all_style)
        )

        launch_layout.addStretch()

        # "Exit Launch/End Processes" button at the bottom
        exit_launch_style = (
            button_style.replace("#2a2a2a", "#4a1a1a")
                        .replace("#3a3a3a", "#6a2a2a")
                        .replace("#1a1a1a", "#300a0a")
        )
        btn_exit_launch = QPushButton("Exit Launch/End Processes")
        btn_exit_launch.setStyleSheet(exit_launch_style)
        btn_exit_launch.setFocusPolicy(Qt.NoFocus)
        btn_exit_launch.clicked.connect(self._show_main_page)
        launch_layout.addWidget(btn_exit_launch)
        self._launch_nav_buttons.append(
            (btn_exit_launch, "Exit Launch/End Processes", exit_launch_style)
        )

        self._options_stack.addWidget(page_launch)  # index 1

        # Store styles for toggle (use compact style for launch grid buttons)
        self._launch_btn_style = launch_btn_compact
        self._launch_on_style = (
            launch_btn_compact
            .replace("border: 1px solid #555", "border: 1px solid #0f0")
            .replace("color: #dcdcdc", "color: #0f0")
        )
        self._launch_wait_style = (
            launch_btn_compact
            .replace("border: 1px solid #555", "border: 1px solid #ff0")
            .replace("color: #dcdcdc", "color: #ff0")
        )

        # --- Page 2: Playback CSV selection ---
        page_playback = QWidget()
        playback_layout = QVBoxLayout(page_playback)
        playback_layout.setContentsMargins(6, 0, 6, 0)

        lbl_pb = QLabel("PLAYBACK")
        lbl_pb.setFont(section_title_font)
        lbl_pb.setAlignment(Qt.AlignCenter)
        lbl_pb.setStyleSheet(section_title_style)
        playback_layout.addWidget(lbl_pb)

        csv_label_style = (
            "border: none; color: #888; font-size: 10px;"
            " font-family: monospace;"
        )
        self._playback_nav_buttons = []
        self._csv_grid = QGridLayout()
        self._csv_grid.setSpacing(4)
        self._csv_grid.setColumnStretch(0, 0)
        self._csv_grid.setColumnStretch(1, 1)
        playback_layout.addLayout(self._csv_grid)
        self._scan_csv_files(button_style, csv_label_style)

        playback_layout.addStretch()

        exit_pb_style = (
            button_style.replace("#2a2a2a", "#4a1a1a")
                        .replace("#3a3a3a", "#6a2a2a")
                        .replace("#1a1a1a", "#300a0a")
        )
        btn_exit_pb = QPushButton("Exit Playback")
        btn_exit_pb.setStyleSheet(exit_pb_style)
        btn_exit_pb.setFocusPolicy(Qt.NoFocus)
        btn_exit_pb.clicked.connect(self._show_main_page)
        playback_layout.addWidget(btn_exit_pb)
        self._playback_nav_buttons.append(
            (btn_exit_pb, "Exit Playback", exit_pb_style)
        )

        self._options_stack.addWidget(page_playback)  # index 2
        self._pb_button_style = button_style
        self._pb_csv_label_style = csv_label_style

        # --- Page 3: Test Mode ---
        page_test = QWidget()
        test_layout = QVBoxLayout(page_test)
        test_layout.setContentsMargins(6, 0, 6, 0)

        lbl_test = QLabel("TEST MODE")
        lbl_test.setFont(section_title_font)
        lbl_test.setAlignment(Qt.AlignCenter)
        lbl_test.setStyleSheet(section_title_style)
        test_layout.addWidget(lbl_test)

        self._test_status_label = QLabel("Status: idle")
        self._test_status_label.setStyleSheet(
            "border: none; color: #888; font-size: 10px;"
            " font-family: monospace;"
        )
        self._test_status_label.setAlignment(Qt.AlignCenter)
        self._test_status_label.setWordWrap(True)
        test_layout.addWidget(self._test_status_label)

        # Test definitions: (id, title, launch command, description)
        self._test_defs = [
            ("t000", "DAQ Mode",
             "ros2 launch autonav_automated_testing t000_DAQ_MODE.launch.py",
             "Data acquisition — operator drives, system logs all sensor data"),
            ("t002", "Line Compliance",
             "ros2 launch autonav_automated_testing t002_Line_Comp.launch.py",
             "Autonomous line-following — robot drives 110 ft along white lines"),
        ]

        self._test_nav_buttons = []
        self._active_test = None  # id of currently running test

        test_btn_style = (
            "QPushButton {"
            "  background-color: #2a2a2a; color: #dcdcdc;"
            "  border: 1px solid #555; border-radius: 4px;"
            "  padding: 10px 4px; font-size: 11px;"
            "}"
            "QPushButton:hover { background-color: #3a3a3a; }"
            "QPushButton:pressed { background-color: #1a1a1a; }"
        )
        self._test_btn_style = test_btn_style
        self._test_on_style = (
            test_btn_style
            .replace("border: 1px solid #555", "border: 1px solid #0f0")
            .replace("color: #dcdcdc", "color: #0f0")
        )

        test_desc_style = (
            "border: none; color: #888; font-size: 10px;"
            " font-family: monospace;"
        )

        test_grid = QGridLayout()
        test_grid.setSpacing(4)
        for i, (tid, title, cmd, desc) in enumerate(self._test_defs):
            btn = QPushButton(f"{tid}: {title}")
            btn.setStyleSheet(test_btn_style)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.setFixedWidth(140)
            btn.clicked.connect(
                lambda checked=False, t=tid: self._toggle_test(t)
            )
            test_grid.addWidget(btn, i, 0)
            self._test_nav_buttons.append((btn, f"{tid}: {title}", test_btn_style))

            desc_lbl = QLabel(desc)
            desc_lbl.setStyleSheet(test_desc_style)
            desc_lbl.setWordWrap(True)
            test_grid.addWidget(desc_lbl, i, 1)

        test_grid.setColumnStretch(0, 0)
        test_grid.setColumnStretch(1, 1)
        test_layout.addLayout(test_grid)

        # Separator
        sep_test = QFrame()
        sep_test.setFrameShape(QFrame.HLine)
        sep_test.setStyleSheet("background-color: #444; border: none; max-height: 1px;")
        sep_test.setFixedHeight(1)
        test_layout.addWidget(sep_test)

        # E-STOP button
        estop_style = (
            "QPushButton {"
            "  background-color: #8b0000; color: #fff;"
            "  border: 2px solid #f00; border-radius: 4px;"
            "  padding: 12px 4px; font-size: 13px; font-weight: bold;"
            "}"
            "QPushButton:hover { background-color: #a00000; }"
            "QPushButton:pressed { background-color: #600000; }"
        )
        btn_estop = QPushButton("E-STOP")
        btn_estop.setStyleSheet(estop_style)
        btn_estop.setFocusPolicy(Qt.NoFocus)
        btn_estop.clicked.connect(self._on_estop)
        test_layout.addWidget(btn_estop)
        self._test_nav_buttons.append((btn_estop, "E-STOP", estop_style))

        test_layout.addStretch()

        # Exit Test Mode button
        exit_test_style = (
            button_style.replace("#2a2a2a", "#4a1a1a")
                        .replace("#3a3a3a", "#6a2a2a")
                        .replace("#1a1a1a", "#300a0a")
        )
        btn_exit_test = QPushButton("Exit Test Mode")
        btn_exit_test.setStyleSheet(exit_test_style)
        btn_exit_test.setFocusPolicy(Qt.NoFocus)
        btn_exit_test.clicked.connect(self._show_main_page)
        test_layout.addWidget(btn_exit_test)
        self._test_nav_buttons.append(
            (btn_exit_test, "Exit Test Mode", exit_test_style)
        )

        self._options_stack.addWidget(page_test)  # index 3

        top_layout.addWidget(options_frame, stretch=2)

        # =====================================================================
        # SECTION 2: LIVE SENSOR DATA (center column)
        # =====================================================================
        sensor_frame = QFrame()
        sensor_frame.setStyleSheet("QFrame { border: 1px solid #444; }")
        sensor_outer = QVBoxLayout(sensor_frame)
        sensor_outer.setContentsMargins(10, 6, 10, 10)

        lbl_sensor = QLabel("LIVE SENSOR DATA")
        lbl_sensor.setFont(section_title_font)
        lbl_sensor.setAlignment(Qt.AlignCenter)
        lbl_sensor.setStyleSheet(section_title_style)
        sensor_outer.addWidget(lbl_sensor)

        sensor_body = QHBoxLayout()
        sensor_outer.addLayout(sensor_body, stretch=1)

        # -- 2a: Status Indicators (narrow left strip) --
        status_col = QVBoxLayout()
        status_col.setSpacing(4)
        self.status_dots = {}
        self._status_nav_buttons = []  # (QPushButton, name, base_style)

        status_btn_style = (
            "QPushButton {"
            "  background-color: #2a2a2a; color: #aaa;"
            "  border: 1px solid #555; border-radius: 3px;"
            "  padding: 1px 4px; font-size: 11px;"
            "}"
        )

        def _add_status_row(name, parent_layout):
            row = QHBoxLayout()
            row.setSpacing(6)
            dot = QLabel()
            dot.setFixedSize(14, 14)
            dot.setStyleSheet(
                "background-color: #555; border-radius: 7px; border: none;"
            )
            row.addWidget(dot)
            btn = QPushButton(name)
            btn.setStyleSheet(status_btn_style)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.setFixedWidth(95)
            btn.clicked.connect(lambda checked=False, n=name: self._on_status_dot_clicked(n))
            row.addWidget(btn)
            row.addStretch()
            parent_layout.addLayout(row)
            self.status_dots[name] = dot
            self._status_nav_buttons.append((btn, name, status_btn_style))

        # Physical Devices
        phys_label = QLabel("Physical Devices")
        phys_label.setStyleSheet(group_label_style)
        status_col.addWidget(phys_label)
        physical_names = [
            "Camera", "Lidar", "GPS", "Encoders",
            "Power PCB", "Arduino", "Motor Ctrl",
        ]
        for name in physical_names:
            _add_status_row(name, status_col)

        # Virtual Devices
        virt_label = QLabel("Virtual Devices")
        virt_label.setStyleSheet(group_label_style + " margin-top: 6px;")
        status_col.addWidget(virt_label)
        virtual_names = ["SLAM", "CONTROL", "NAV2", "LINE DETECT"]
        for name in virtual_names:
            _add_status_row(name, status_col)

        # Test System
        test_label = QLabel("Test System")
        test_label.setStyleSheet(group_label_style + " margin-top: 6px;")
        status_col.addWidget(test_label)
        _add_status_row("TEST", status_col)

        # Build dot-name → launch-device reverse mapping
        self._dot_to_device = {}
        for dev_label, dot_keys, _cmd in self._launch_devices:
            for key in dot_keys:
                self._dot_to_device[key] = dev_label
        self._dot_to_device["TEST"] = None

        left_col = QVBoxLayout()
        left_col.addLayout(status_col)
        left_col.addStretch()

        # -- 2b: Sensor Display Grid --
        grid = QGridLayout()
        grid.setSpacing(4)

        def _sensor_cell(title_text):
            f = QFrame()
            f.setObjectName("sensorCell")
            f.setStyleSheet(frame_style)
            f.setFrameShape(QFrame.StyledPanel)
            f.setFrameShadow(QFrame.Raised)
            v = QVBoxLayout(f)
            v.setContentsMargins(6, 4, 6, 4)
            t = QLabel(title_text)
            t.setStyleSheet("border: none; font-weight: bold; color: #ccc;")
            t.setAlignment(Qt.AlignCenter)
            v.addWidget(t)
            val = QLabel("")
            val.setStyleSheet(val_label_style)
            val.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            val.setWordWrap(True)
            v.addWidget(val)
            v.addStretch()
            return f, v, val

        def _plot_cell(title_text):
            f = QFrame()
            f.setObjectName("sensorCell")
            f.setStyleSheet(frame_style)
            f.setFrameShape(QFrame.StyledPanel)
            f.setFrameShadow(QFrame.Raised)
            v = QVBoxLayout(f)
            v.setContentsMargins(4, 2, 4, 2)
            t = QLabel(title_text)
            t.setStyleSheet("border: none; font-weight: bold; color: #ccc;")
            t.setAlignment(Qt.AlignCenter)
            v.addWidget(t)
            return f, v

        # Camera — image canvas for video playback
        cam_cell, cam_layout = _plot_cell("Camera")
        self._cam_fig, self._cam_ax, self._cam_canvas = _make_dark_canvas()
        self._cam_fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
        self._cam_ax.set_facecolor('#111111')
        self._cam_ax.axis('off')
        self._cam_no_video_txt = self._cam_ax.text(
            0.5, 0.5, 'VIDEO NOT AVAILABLE\nFOR THIS SET OF DATA',
            transform=self._cam_ax.transAxes, fontsize=8, color='#555',
            ha='center', va='center', family='monospace',
        )
        self._cam_no_video_txt.set_visible(False)
        self._cam_live_txt = self._cam_ax.text(
            0.5, 0.5, 'NO DATA AVAILABLE\nAT THE MOMENT',
            transform=self._cam_ax.transAxes, fontsize=8, color='#555',
            ha='center', va='center', family='monospace',
        )
        self._cam_live_txt.set_visible(False)
        self._cam_canvas.setMinimumSize(0, 0)
        cam_layout.addWidget(self._cam_canvas, stretch=1)

        # Lidar — image canvas for video playback
        lidar_cell, lidar_layout = _plot_cell("Lidar")
        self._lidar_fig, self._lidar_ax, self._lidar_canvas = _make_dark_canvas()
        self._lidar_fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
        self._lidar_ax.set_facecolor('#111111')
        self._lidar_ax.axis('off')
        self._lidar_no_video_txt = self._lidar_ax.text(
            0.5, 0.5, 'VIDEO NOT AVAILABLE\nFOR THIS SET OF DATA',
            transform=self._lidar_ax.transAxes, fontsize=8, color='#555',
            ha='center', va='center', family='monospace',
        )
        self._lidar_no_video_txt.set_visible(False)
        self._lidar_live_txt = self._lidar_ax.text(
            0.5, 0.5, 'NO DATA AVAILABLE\nAT THE MOMENT',
            transform=self._lidar_ax.transAxes, fontsize=8, color='#555',
            ha='center', va='center', family='monospace',
        )
        self._lidar_live_txt.set_visible(False)
        self._lidar_canvas.setMinimumSize(0, 0)
        lidar_layout.addWidget(self._lidar_canvas, stretch=1)

        # GPS — map plot with satellite background (no axis clutter)
        gps_cell, gps_layout = _plot_cell("GPS")
        self._gps_fig, self._gps_ax, self._gps_canvas = _make_dark_canvas()
        self._gps_fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
        self._gps_ax.set_facecolor('#111111')
        self._gps_ax.set_xticklabels([])
        self._gps_ax.set_yticklabels([])
        self._gps_ax.tick_params(axis='both', length=0, pad=0)
        for spine in self._gps_ax.spines.values():
            spine.set_visible(False)
        self._gps_coord_label = self._gps_ax.text(
            0.02, 0.96, '', transform=self._gps_ax.transAxes,
            fontsize=7, color='#0f0', verticalalignment='top',
            family='monospace',
            bbox=dict(facecolor='#222222', alpha=0.6, edgecolor='none', pad=2),
        )
        self._gps_no_data_txt = self._gps_ax.text(
            0.5, 0.5, 'NO GPS FOR\nTHIS DATA SET',
            transform=self._gps_ax.transAxes, fontsize=8, color='#555',
            ha='center', va='center', family='monospace',
        )
        self._gps_no_data_txt.set_visible(False)
        self._gps_live_txt = self._gps_ax.text(
            0.5, 0.5, 'NO DATA AVAILABLE\nAT THE MOMENT',
            transform=self._gps_ax.transAxes, fontsize=8, color='#555',
            ha='center', va='center', family='monospace',
        )
        self._gps_live_txt.set_visible(False)
        self._gps_map_im = None          # imshow handle for map background
        self._gps_map_extent = None       # (lon_min, lon_max, lat_min, lat_max)
        self._gps_map_img = None          # numpy RGB array
        self._gps_trail, = self._gps_ax.plot([], [], 'c-', linewidth=1, alpha=0.6)
        self._gps_dot, = self._gps_ax.plot([], [], 'ro', markersize=5, zorder=5)
        self._gps_canvas.setMinimumSize(0, 0)
        gps_layout.addWidget(self._gps_canvas, stretch=1)

        # Encoders — XY odometry plot with white trail + 1m grid
        enc_cell, enc_layout = _plot_cell("Encoders (Odom)")
        self._odom_fig, self._odom_ax, self._odom_canvas = _make_dark_canvas()
        self._odom_fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
        self._odom_ax.set_facecolor('#111111')
        self._odom_ax.tick_params(axis='both', length=0, pad=-12,
                                   labelsize=6, colors='#666', direction='in')
        for spine in self._odom_ax.spines.values():
            spine.set_visible(False)
        self._odom_ax.set_aspect('equal', adjustable='datalim')
        self._odom_ax.grid(True, which='both', color='#333', linewidth=0.5)
        self._odom_scatter = None  # PathCollection, recreated each frame
        # Inside axis label + distance readout
        self._odom_xy_label = self._odom_ax.text(
            0.02, 0.96, 'x / y (m)', transform=self._odom_ax.transAxes,
            fontsize=7, color='#888', verticalalignment='top',
            bbox=dict(facecolor='#222222', alpha=0.6, edgecolor='none', pad=2),
        )
        self._odom_dist_label = self._odom_ax.text(
            0.98, 0.96, '', transform=self._odom_ax.transAxes,
            fontsize=7, color='#0f0', verticalalignment='top',
            horizontalalignment='right', family='monospace',
            bbox=dict(facecolor='#222222', alpha=0.6, edgecolor='none', pad=2),
        )
        self._odom_live_txt = self._odom_ax.text(
            0.5, 0.5, 'NO DATA AVAILABLE\nAT THE MOMENT',
            transform=self._odom_ax.transAxes, fontsize=8, color='#555',
            ha='center', va='center', family='monospace',
        )
        self._odom_live_txt.set_visible(False)
        self._odom_canvas.setMinimumSize(0, 0)
        enc_layout.addWidget(self._odom_canvas, stretch=1)

        # Power PCB — vertical column with 3 mini oscilloscopes + SOC gauge
        power_cell, power_layout = _plot_cell("Power PCB")

        pwr_val_style = (
            "border: none; color: #0f0; font-size: 10px;"
            " font-family: monospace;"
        )
        pwr_title_style = "border: none; font-weight: bold; color: #ccc; font-size: 9px;"

        # Voltage mini-graph — fixed Y: 20-30
        self._pwr_v_fig, self._pwr_v_ax, self._pwr_v_canvas = _make_mini_canvas()
        _style_ax(self._pwr_v_ax)
        self._pwr_v_ax.set_ylim(20, 30)
        self._pwr_line_v, = self._pwr_v_ax.plot([], [], color='#4af', linewidth=1)
        self._pwr_v_live_txt = self._pwr_v_ax.text(
            0.5, 0.5, 'NO DATA', transform=self._pwr_v_ax.transAxes,
            fontsize=7, color='#555', ha='center', va='center', family='monospace',
        )
        self._pwr_v_live_txt.set_visible(False)
        self._pwr_v_canvas.setMinimumSize(0, 0)

        pwr_v_title_row = QHBoxLayout()
        pwr_v_title_lbl = QLabel("Voltage (V)")
        pwr_v_title_lbl.setStyleSheet(pwr_title_style)
        self._pwr_val_v = QLabel("V: --")
        self._pwr_val_v.setAlignment(Qt.AlignRight)
        self._pwr_val_v.setStyleSheet(pwr_val_style.replace("#0f0", "#4af"))
        pwr_v_title_row.addWidget(pwr_v_title_lbl)
        pwr_v_title_row.addWidget(self._pwr_val_v)
        power_layout.addLayout(pwr_v_title_row)
        power_layout.addWidget(self._pwr_v_canvas, stretch=1)

        # Current mini-graph — fixed Y: 0-6
        self._pwr_i_fig, self._pwr_i_ax, self._pwr_i_canvas = _make_mini_canvas()
        _style_ax(self._pwr_i_ax)
        self._pwr_i_ax.set_ylim(0, 6)
        self._pwr_line_i, = self._pwr_i_ax.plot([], [], color='#f44', linewidth=1)
        self._pwr_i_live_txt = self._pwr_i_ax.text(
            0.5, 0.5, 'NO DATA', transform=self._pwr_i_ax.transAxes,
            fontsize=7, color='#555', ha='center', va='center', family='monospace',
        )
        self._pwr_i_live_txt.set_visible(False)
        self._pwr_i_canvas.setMinimumSize(0, 0)

        pwr_i_title_row = QHBoxLayout()
        pwr_i_title_lbl = QLabel("Current (A)")
        pwr_i_title_lbl.setStyleSheet(pwr_title_style)
        self._pwr_val_i = QLabel("I: --")
        self._pwr_val_i.setAlignment(Qt.AlignRight)
        self._pwr_val_i.setStyleSheet(pwr_val_style.replace("#0f0", "#f44"))
        pwr_i_title_row.addWidget(pwr_i_title_lbl)
        pwr_i_title_row.addWidget(self._pwr_val_i)
        power_layout.addLayout(pwr_i_title_row)
        power_layout.addWidget(self._pwr_i_canvas, stretch=1)

        # Power mini-graph — fixed Y: 0-100
        self._pwr_p_fig, self._pwr_p_ax, self._pwr_p_canvas = _make_mini_canvas()
        _style_ax(self._pwr_p_ax)
        self._pwr_p_ax.set_ylim(0, 100)
        self._pwr_line_p, = self._pwr_p_ax.plot([], [], color='#4f4', linewidth=1)
        self._pwr_p_live_txt = self._pwr_p_ax.text(
            0.5, 0.5, 'NO DATA', transform=self._pwr_p_ax.transAxes,
            fontsize=7, color='#555', ha='center', va='center', family='monospace',
        )
        self._pwr_p_live_txt.set_visible(False)
        self._pwr_p_canvas.setMinimumSize(0, 0)

        pwr_p_title_row = QHBoxLayout()
        pwr_p_title_lbl = QLabel("Power (W)")
        pwr_p_title_lbl.setStyleSheet(pwr_title_style)
        self._pwr_val_p = QLabel("P: --")
        self._pwr_val_p.setAlignment(Qt.AlignRight)
        self._pwr_val_p.setStyleSheet(pwr_val_style.replace("#0f0", "#4f4"))
        pwr_p_title_row.addWidget(pwr_p_title_lbl)
        pwr_p_title_row.addWidget(self._pwr_val_p)
        power_layout.addLayout(pwr_p_title_row)
        power_layout.addWidget(self._pwr_p_canvas, stretch=1)

        # SOC fuel gauge bar — horizontal
        soc_row = QHBoxLayout()
        soc_title = QLabel("SOC")
        soc_title.setStyleSheet("border: none; font-weight: bold; color: #ccc; font-size: 9px;")
        soc_row.addWidget(soc_title)
        self._soc_bar = QFrame()
        self._soc_bar.setFixedHeight(18)
        self._soc_bar.setStyleSheet(
            "background-color: #252525; border: 1px solid #555; border-radius: 2px;"
        )
        self._soc_bar_layout = QHBoxLayout(self._soc_bar)
        self._soc_bar_layout.setContentsMargins(2, 2, 2, 2)
        self._soc_bar_layout.setSpacing(0)
        self._soc_fill = QFrame()
        self._soc_fill.setStyleSheet("background-color: #4f4; border: none; border-radius: 1px;")
        self._soc_bar_layout.addWidget(self._soc_fill, stretch=0)  # fill at 0%
        self._soc_bar_layout.addStretch(1)  # empty space on right
        soc_row.addWidget(self._soc_bar, stretch=1)
        self._soc_label = QLabel("0%")
        self._soc_label.setStyleSheet("border: none; color: #888; font-size: 9px; font-family: monospace;")
        soc_row.addWidget(self._soc_label)
        power_layout.addLayout(soc_row)

        # Add Power PCB below the device status list
        left_col.addWidget(power_cell, stretch=1)
        sensor_body.addLayout(left_col)

        # -- 2b continued: Sensor grid (Camera, Lidar, GPS, Encoders) --
        grid.addWidget(cam_cell, 0, 0)
        grid.addWidget(lidar_cell, 0, 1)
        grid.addWidget(gps_cell, 1, 0)
        grid.addWidget(enc_cell, 1, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)

        sensor_body.addLayout(grid, stretch=1)

        # -- Playback time slider --
        slider_row = QHBoxLayout()

        # Stacked time + frame labels on the left
        slider_info = QVBoxLayout()
        slider_info.setSpacing(0)
        self.pb_time_label = QLabel("0.0s / 0.0s")
        self.pb_time_label.setStyleSheet(
            "border: none; color: #888; font-size: 10px; font-family: monospace;"
        )
        self.pb_time_label.setFixedWidth(100)
        slider_info.addWidget(self.pb_time_label)
        self.pb_frame_label = QLabel("F: -- / --")
        self.pb_frame_label.setStyleSheet(
            "border: none; color: #555; font-size: 10px; font-family: monospace;"
        )
        self.pb_frame_label.setFixedWidth(100)
        slider_info.addWidget(self.pb_frame_label)
        slider_row.addLayout(slider_info)

        self.pb_slider = QSlider(Qt.Horizontal)
        self.pb_slider.setRange(0, 0)
        self.pb_slider.setEnabled(False)
        self.pb_slider.setStyleSheet(
            "QSlider { border: none; }"
            "QSlider::groove:horizontal {"
            "  background: #333; height: 6px; border-radius: 3px;"
            "}"
            "QSlider::handle:horizontal {"
            "  background: #0af; width: 14px; margin: -4px 0;"
            "  border-radius: 7px;"
            "}"
        )
        self.pb_slider.sliderPressed.connect(self._on_slider_pressed)
        self.pb_slider.sliderReleased.connect(self._on_slider_released)
        self.pb_slider.sliderMoved.connect(self._on_slider_seek)
        slider_row.addWidget(self.pb_slider, stretch=1)

        self._play_pause_style = (
            "QPushButton { background-color: #2a2a2a; color: #dcdcdc;"
            "  border: 1px solid #555; border-radius: 3px;"
            "  padding: 4px 10px; font-size: 16px;"
            "  min-width: 36px; min-height: 24px; }"
            "QPushButton:hover { background-color: #3a3a3a; }"
            "QPushButton:disabled { color: #555; }"
        )
        play_pause_style = self._play_pause_style
        self.btn_pp = QPushButton("\u25B6")
        self.btn_pp.setFixedSize(40, 28)
        self.btn_pp.setStyleSheet(play_pause_style)
        self.btn_pp.setFocusPolicy(Qt.NoFocus)
        self.btn_pp.setEnabled(False)
        self.btn_pp.clicked.connect(self._on_play_pause)
        slider_row.addWidget(self.btn_pp)

        # Playback speed button
        self._pb_speed_options = [1.0, 2.0, 4.0]
        self._pb_speed_idx = 0
        self._pb_speed = 1.0
        speed_btn_style = (
            "QPushButton { background-color: #2a2a2a; color: #dcdcdc;"
            "  border: 1px solid #555; border-radius: 3px;"
            "  padding: 4px 6px; font-size: 11px;"
            "  min-width: 36px; min-height: 24px; }"
            "QPushButton:hover { background-color: #3a3a3a; }"
        )
        self._speed_btn_style = speed_btn_style
        self.btn_speed = QPushButton("1x")
        self.btn_speed.setFixedSize(40, 28)
        self.btn_speed.setStyleSheet(speed_btn_style)
        self.btn_speed.setFocusPolicy(Qt.NoFocus)
        self.btn_speed.clicked.connect(self._cycle_playback_speed)
        slider_row.addWidget(self.btn_speed)

        sensor_outer.addLayout(slider_row)

        top_layout.addWidget(sensor_frame, stretch=5)

        # =====================================================================
        # SECTION 3: PROCESS TERMINAL (right column)
        # =====================================================================
        viz_frame = QFrame()
        viz_frame.setStyleSheet("QFrame { border: 1px solid #444; }")
        viz_layout = QVBoxLayout(viz_frame)
        viz_layout.setContentsMargins(10, 6, 10, 10)

        lbl_viz = QLabel("PROCESS TERMINAL")
        lbl_viz.setFont(section_title_font)
        lbl_viz.setAlignment(Qt.AlignCenter)
        lbl_viz.setStyleSheet(section_title_style)
        viz_layout.addWidget(lbl_viz)

        self._term_header = QLabel("Select a process from the status dots")
        self._term_header.setAlignment(Qt.AlignCenter)
        self._term_header.setStyleSheet(
            "border: none; color: #888; font-size: 12px; font-family: monospace;"
        )
        viz_layout.addWidget(self._term_header)

        self._term_display = QTextEdit()
        self._term_display.setReadOnly(True)
        self._term_display.setFocusPolicy(Qt.NoFocus)
        self._term_display.setLineWrapMode(QTextEdit.WidgetWidth)
        self._term_display.setStyleSheet(
            "QTextEdit {"
            "  background-color: #0a0a0a; color: #0f0;"
            "  border: 1px solid #333; border-radius: 3px;"
            "  font-family: monospace; font-size: 11px;"
            "  padding: 4px;"
            "}"
        )
        viz_layout.addWidget(self._term_display, stretch=1)

        self._selected_process = None
        self._gui_log = []  # GUI info log lines

        # Process management state
        self._process_objects = {}   # label → subprocess.Popen
        self._process_buffers = {}   # label → list of str lines
        self._process_readers = {}   # label → threading.Thread

        # 5 Hz timer to poll process output and refresh terminal
        self._process_poll_timer = QTimer()
        self._process_poll_timer.setInterval(200)
        self._process_poll_timer.timeout.connect(self._poll_process_output)
        self._process_poll_timer.start()

        top_layout.addWidget(viz_frame, stretch=2)

        # -- Keyboard navigation --
        self._sel_idx = 0
        # Slider highlight styles (normal vs selected)
        self._slider_base_style = (
            "QSlider { border: none; }"
            "QSlider::groove:horizontal {"
            "  background: #333; height: 6px; border-radius: 3px;"
            "}"
            "QSlider::handle:horizontal {"
            "  background: #0af; width: 14px; margin: -4px 0;"
            "  border-radius: 7px;"
            "}"
        )
        self._slider_sel_style = (
            "QSlider { border: 1px solid #0af; border-radius: 4px; }"
            "QSlider::groove:horizontal {"
            "  background: #333; height: 6px; border-radius: 3px;"
            "}"
            "QSlider::handle:horizontal {"
            "  background: #fff; width: 14px; margin: -4px 0;"
            "  border-radius: 7px;"
            "}"
        )
        self._slider_scrub_style = (
            "QSlider { border: none; }"
            "QSlider::groove:horizontal {"
            "  background: #333; height: 6px; border-radius: 3px;"
            "}"
            "QSlider::handle:horizontal {"
            "  background: #0f0; width: 14px; margin: -4px 0;"
            "  border: 2px solid #0f0; border-radius: 7px;"
            "}"
        )
        # 2D nav grid: group 0 = OPTIONS, group 1 = status dots,
        #   group 2 = slider, group 3 = play/pause, group 4 = speed
        self._nav_groups = [
            self._nav_buttons,                                             # group 0
            self._status_nav_buttons,                                      # group 1
            [(self.pb_slider, "Scrub Bar", self._slider_base_style)],      # group 2
            [(self.btn_pp, "\u25B6", play_pause_style)],                   # group 3
            [(self.btn_speed, "1x", speed_btn_style)],                     # group 4
        ]
        self._nav_col = 0   # current group
        self._nav_row = 0   # current index within group
        self._nav_last_row = [0, 0, 0, 0, 0]  # remember row per group
        self._scrub_mode = False  # True when actively scrubbing with arrows
        self._speed_mode = False  # True when selecting playback speed with arrows

        self._sel_frames_r = [' <', '< ']
        self._sel_frames_l = ['> ', ' >']
        self._sel_frame_durations = [3, 3]
        self._sel_frame_idx = 0
        self._sel_tick_count = 0
        self._sel_arrow_l = self._sel_frames_l[0]
        self._sel_arrow_r = self._sel_frames_r[0]

        # Floating directional indicators (overlays on the central widget)
        indicator_style = (
            "color: #0af; font-size: 12px; font-weight: bold;"
            " font-family: monospace; background: transparent; border: none;"
        )
        self._ind_left = QLabel("<<", central)
        self._ind_right = QLabel(">>", central)
        for ind in (self._ind_left, self._ind_right):
            ind.setStyleSheet(indicator_style)
            ind.setAlignment(Qt.AlignCenter)
            ind.adjustSize()
            ind.raise_()
            ind.hide()

        self._update_selection()

        self._nav_timer = QTimer()
        self._nav_timer.setInterval(100)  # 10 Hz animation
        self._nav_timer.timeout.connect(self._nav_anim_tick)
        self._nav_timer.start()

    # -----------------------------------------------------------------
    # Keyboard navigation
    # -----------------------------------------------------------------
    @staticmethod
    def _make_sel_style(base_style):
        """Derive a selected style from the base style by swapping bg/border colors."""
        s = base_style
        # Replace background colors with blue-highlighted versions
        for old, new in [
            ('background-color: #2a2a2a', 'background-color: #1a3a5a'),
            ('background-color: #4a1a1a', 'background-color: #1a3a5a'),
            ('background-color: #8b0000', 'background-color: #1a3a5a'),
            ('border: 1px solid #555', 'border: 1px solid #0af'),
            ('border: 2px solid #f00', 'border: 2px solid #0af'),
            ('color: #dcdcdc', 'color: #ffffff'),
            ('color: #fff', 'color: #ffffff'),
        ]:
            s = s.replace(old, new)
        return s

    # -- Launch/End Processes sub-page ------------------------------------------

    def _show_launch_page(self):
        """Switch OPTIONS column to the Launch/End Processes sub-page."""
        self._options_stack.setCurrentIndex(1)
        # Swap nav group 0 to the launch buttons
        self._nav_groups[0] = self._launch_nav_buttons
        self._nav_col = 0
        self._nav_row = 0
        self._nav_last_row[0] = 0
        self._update_selection()

    # -- Playback CSV sub-page -------------------------------------------------

    def _show_playback_page(self):
        """Switch OPTIONS column to the Playback CSV selection sub-page."""
        # Rescan for any new CSV files
        self._scan_csv_files(self._pb_button_style, self._pb_csv_label_style)
        self._options_stack.setCurrentIndex(2)
        self._nav_groups[0] = self._playback_nav_buttons
        self._nav_col = 0
        self._nav_row = 0
        self._nav_last_row[0] = 0
        self._update_selection()

    def _scan_csv_files(self, button_style, csv_label_style):
        """Scan _CSV_DIR for .csv files and populate the grid + nav list."""
        # Clear existing grid and nav buttons (except the exit button at the end)
        exit_btn_entry = None
        if self._playback_nav_buttons:
            # Preserve the exit button (last entry)
            last = self._playback_nav_buttons[-1]
            if last[1] == "Exit Playback":
                exit_btn_entry = last

        self._playback_nav_buttons.clear()

        # Remove all items from the grid
        while self._csv_grid.count():
            item = self._csv_grid.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

        csv_dir = self._CSV_DIR
        csv_entries = []  # (display_name, csv_path)
        if os.path.isdir(csv_dir):
            # Flat CSVs in the top-level directory
            csv_entries.extend(sorted(
                (f, os.path.join(csv_dir, f))
                for f in os.listdir(csv_dir) if f.lower().endswith('.csv')
            ))
            # Subdirectories containing CSVs — show one entry per folder
            for entry in sorted(os.listdir(csv_dir)):
                subdir = os.path.join(csv_dir, entry)
                if os.path.isdir(subdir):
                    # Find the first CSV in this folder
                    csvs = sorted(f for f in os.listdir(subdir) if f.lower().endswith('.csv'))
                    if csvs:
                        csv_entries.append((f'Folder:  {entry}', os.path.join(subdir, csvs[0])))

        for i, (display_name, full_path) in enumerate(csv_entries):
            btn = QPushButton("Load")
            btn.setStyleSheet(button_style)
            btn.setFocusPolicy(Qt.NoFocus)
            btn.clicked.connect(
                lambda checked=False, p=full_path: self._load_and_play_csv(p)
            )
            self._csv_grid.addWidget(btn, i, 0)
            self._playback_nav_buttons.append((btn, "Load", button_style))

            lbl = QLabel(display_name)
            lbl.setStyleSheet(csv_label_style)
            self._csv_grid.addWidget(lbl, i, 1)

        if exit_btn_entry:
            self._playback_nav_buttons.append(exit_btn_entry)

    def _load_and_play_csv(self, path):
        """Load a CSV and start playback, then return to main page."""
        self._stop_playback()
        self._load_csv(path)
        self._start_playback()
        self._show_main_page()

    def _show_main_page(self):
        """Switch OPTIONS column back to the main page."""
        self._options_stack.setCurrentIndex(0)
        # Restore nav group 0 to the main option buttons
        self._nav_groups[0] = self._nav_buttons
        self._nav_col = 0
        self._nav_row = 0
        self._nav_last_row[0] = 0
        self._update_selection()

    # -- Test Mode sub-page ---------------------------------------------------

    def _on_test_clicked(self):
        """Show the Test Mode page."""
        if self._live_active:
            self._stop_live_mode()
        if self._pb_state != 'idle':
            self._stop_playback()
        self._show_test_page()

    def _show_test_page(self):
        """Switch OPTIONS column to the Test Mode sub-page."""
        self._options_stack.setCurrentIndex(3)
        self._nav_groups[0] = self._test_nav_buttons
        self._nav_col = 0
        self._nav_row = 0
        self._nav_last_row[0] = 0
        self._update_selection()

    def _toggle_test(self, tid):
        """Start or stop a test by its ID."""
        if self._active_test == tid:
            # Stop the running test
            self._stop_test(tid)
        else:
            # Stop any other running test first
            if self._active_test is not None:
                self._stop_test(self._active_test)
            self._start_test(tid)

    def _start_test(self, tid):
        """Launch a test subprocess."""
        tdef = None
        for d in self._test_defs:
            if d[0] == tid:
                tdef = d
                break
        if tdef is None:
            return
        tid, title, cmd, _desc = tdef

        self._gui_log_msg(f"Starting test {tid}: {title}")
        self._active_test = tid
        self._test_status_label.setText(f"Status: running {tid}")

        # Highlight the active test button green
        for btn, label, base_style in self._test_nav_buttons:
            if label.startswith(tid):
                btn.setStyleSheet(self._test_on_style)
                # Update nav list entry
                for i, (b, l, _s) in enumerate(self._test_nav_buttons):
                    if b is btn:
                        self._test_nav_buttons[i] = (b, l, self._test_on_style)
                        break
                break

        # Launch subprocess (wrapped for container if connected)
        test_label = f"test:{tid}"
        exec_cmd = self._wrap_container_cmd(cmd, label=test_label)
        buf = []
        self._process_buffers[test_label] = buf
        try:
            proc = subprocess.Popen(
                exec_cmd, shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )
            self._process_objects[f"test:{tid}"] = proc

            def _reader():
                try:
                    for line in proc.stdout:
                        buf.append(line)
                except Exception:
                    pass
            t = threading.Thread(target=_reader, daemon=True)
            t.start()
            self._process_readers[f"test:{tid}"] = t
        except Exception as e:
            buf.append(f"[ERROR] Failed to launch: {e}\n")
            self._gui_log_msg(f"Test {tid} failed to launch: {e}")

        self._update_selection()

    def _stop_test(self, tid):
        """Stop a running test."""
        self._gui_log_msg(f"Stopping test {tid}")
        key = f"test:{tid}"
        proc = self._process_objects.pop(key, None)
        if proc is not None:
            self._kill_process(proc, key)
        buf = self._process_buffers.get(key)
        if buf is not None:
            buf.append("[Test stopped by user]\n")
        self._process_readers.pop(key, None)

        self._active_test = None
        self._test_status_label.setText("Status: idle")

        # Restore button style
        for btn, label, base_style in self._test_nav_buttons:
            if label.startswith(tid):
                btn.setStyleSheet(self._test_btn_style)
                for i, (b, l, _s) in enumerate(self._test_nav_buttons):
                    if b is btn:
                        self._test_nav_buttons[i] = (b, l, self._test_btn_style)
                        break
                break

        self._update_selection()
        self._refresh_terminal_display()

    def _on_estop(self):
        """E-STOP: stop active test and publish estop if ROS available."""
        self._gui_log_msg("E-STOP activated!")
        if self._active_test is not None:
            self._stop_test(self._active_test)
        # Publish to /estop topic if ROS node exists
        node = self._ros_node
        if node is not None and hasattr(node, '_estop_pub'):
            from std_msgs.msg import String
            msg = String()
            msg.data = "STOP"
            node._estop_pub.publish(msg)

    _DOT_ON = "background-color: #0f0; border-radius: 7px; border: none;"
    _DOT_OFF = "background-color: #555; border-radius: 7px; border: none;"
    _DOT_YELLOW = "background-color: #ff0; border-radius: 7px; border: none;"

    def _dot_keys_for(self, label):
        """Return the status-dot keys for a device label."""
        for dev_label, keys, _cmd in self._launch_devices:
            if dev_label == label:
                return keys
        return []

    def _update_queue_label(self):
        """Refresh the queue status text at the top of the launch page."""
        # Find what's currently starting
        starting = None
        for lbl, st in self._launch_states.items():
            if st == 'starting':
                starting = lbl
                break
        parts = []
        if starting:
            parts.append(f"Starting: {starting}")
        if self._launch_queue:
            q_str = " > ".join(self._launch_queue)
            parts.append(f"Queued: {q_str}")
        if parts:
            self._queue_label.setText(" | ".join(parts))
            self._queue_label.setStyleSheet(
                "border: none; color: #ff0; font-size: 10px;"
                " font-family: monospace;"
            )
        else:
            self._queue_label.setText("Queue: idle")
            self._queue_label.setStyleSheet(
                "border: none; color: #888; font-size: 10px;"
                " font-family: monospace;"
            )

    def _toggle_device(self, label):
        """Toggle a device on/off. Uses a queue so only one starts at a time."""
        state = self._launch_states.get(label, False)

        # --- Turning off (from any state) ---
        if state:  # True or 'starting'
            self._gui_log_msg(f"Stopping: {label}")
            self._launch_states[label] = False
            dot_keys = self._dot_keys_for(label)
            for key in dot_keys:
                if key in self.status_dots:
                    self.status_dots[key].setStyleSheet(self._DOT_OFF)
            # Cancel timers
            if label in self._startup_timers:
                self._startup_timers[label].stop()
                del self._startup_timers[label]
            if label in self._flash_timers:
                self._flash_timers[label].stop()
                del self._flash_timers[label]
            # Terminate subprocess
            proc = self._process_objects.pop(label, None)
            if proc is not None:
                self._kill_process(proc, label)
                buf = self._process_buffers.get(label)
                if buf is not None:
                    buf.append("[Process terminated by user]\n")
                self._refresh_terminal_display()
            self._process_readers.pop(label, None)
            # Remove from queue if queued
            if label in self._launch_queue:
                self._launch_queue.remove(label)
            # Restore button label and style
            for i, (btn, blabel, _s) in enumerate(self._launch_nav_buttons):
                if blabel == label:
                    btn.setText(label)
                    self._launch_nav_buttons[i] = (btn, blabel, self._launch_btn_style)
                    btn.setStyleSheet(self._launch_btn_style)
                    break
            self._update_selection()
            self._update_queue_label()
            # If we cancelled an active startup, process next in queue
            if state == 'starting':
                self._process_queue()
            return

        # --- Turning on ---
        # If something is already starting, queue this one
        any_starting = any(
            s == 'starting' for s in self._launch_states.values()
        )
        if any_starting:
            if label not in self._launch_queue:
                self._launch_queue.append(label)
                # Show "Waiting" on the button
                for i, (btn, blabel, _s) in enumerate(self._launch_nav_buttons):
                    if blabel == label:
                        btn.setText("Waiting")
                        btn.setStyleSheet(self._launch_wait_style)
                        self._launch_nav_buttons[i] = (btn, blabel, self._launch_wait_style)
                        break
                self._update_selection()
            self._update_queue_label()
            return

        # Nothing starting — launch immediately
        self._start_device(label)

    def _start_device(self, label):
        """Begin the startup sequence: flash dots, launch subprocess, check after 1.5s."""
        self._gui_log_msg(f"Launching: {label}")
        dot_keys = self._dot_keys_for(label)
        self._launch_states[label] = 'starting'
        # Restore button text (may have been "Waiting")
        for i, (btn, blabel, _s) in enumerate(self._launch_nav_buttons):
            if blabel == label:
                btn.setText(label)
                break
        self._update_queue_label()

        # Flash timer: alternate yellow/gray every 200ms
        flash_state = [True]

        def flash():
            if self._launch_states.get(label) != 'starting':
                return
            style = self._DOT_YELLOW if flash_state[0] else self._DOT_OFF
            for key in dot_keys:
                if key in self.status_dots:
                    self.status_dots[key].setStyleSheet(style)
            flash_state[0] = not flash_state[0]

        flash_timer = QTimer()
        flash_timer.setInterval(200)
        flash_timer.timeout.connect(flash)
        flash_timer.start()
        self._flash_timers[label] = flash_timer
        flash()

        # Find the command for this device
        cmd = None
        for dev_label, _keys, dev_cmd in self._launch_devices:
            if dev_label == label:
                cmd = dev_cmd
                break

        # Launch real subprocess (wrapped for container if connected)
        exec_cmd = self._wrap_container_cmd(cmd, label=label)
        buf = []
        self._process_buffers[label] = buf
        buf.append(f"$ {exec_cmd}\n")
        try:
            proc = subprocess.Popen(
                exec_cmd, shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True,
            )
            self._process_objects[label] = proc

            # Daemon reader thread: appends stdout lines to buffer
            def _reader():
                try:
                    for line in proc.stdout:
                        buf.append(line)
                except Exception:
                    pass
            t = threading.Thread(target=_reader, daemon=True)
            t.start()
            self._process_readers[label] = t
        except Exception as e:
            buf.append(f"[Failed to start: {e}]\n")
            self._finish_device_startup(label, success=False)
            return

        # After 1.5s, check if process is still running (success) or exited (failure)
        startup_timer = QTimer()
        startup_timer.setSingleShot(True)
        startup_timer.setInterval(1500)
        startup_timer.timeout.connect(lambda: self._check_startup(label))
        startup_timer.start()
        self._startup_timers[label] = startup_timer

        self._update_selection()

    def _launch_all_in_sequence(self):
        """Queue all devices that aren't already on or starting."""
        for label, _keys, _cmd in self._launch_devices:
            state = self._launch_states.get(label, False)
            if not state:  # not on, not starting
                self._toggle_device(label)

    def _process_queue(self):
        """Start the next queued device if nothing is currently starting."""
        any_starting = any(
            s == 'starting' for s in self._launch_states.values()
        )
        if any_starting:
            self._update_queue_label()
            return
        while self._launch_queue:
            next_label = self._launch_queue.pop(0)
            # Skip if it was turned off while queued
            if self._launch_states.get(next_label, False):
                continue
            self._start_device(next_label)
            return
        self._update_queue_label()

    def _check_startup(self, label):
        """Called 1.5s after launch: check if process still running."""
        if label in self._startup_timers:
            del self._startup_timers[label]
        if self._launch_states.get(label) != 'starting':
            return
        proc = self._process_objects.get(label)
        if proc is None:
            self._finish_device_startup(label, success=True)
            return
        if proc.poll() is not None:
            # Process already exited — note it in the buffer
            buf = self._process_buffers.get(label)
            if buf is not None:
                buf.append(f"[Process exited with code {proc.returncode}]\n")
            self._process_objects.pop(label, None)
            self._process_readers.pop(label, None)
        # Always mark as success so dots go green
        self._finish_device_startup(label, success=True)

    def _finish_device_startup(self, label, success):
        """Stop flashing, set dots green or gray, update button style, process queue."""
        self._gui_log_msg(f"{'Started' if success else 'Failed'}: {label}")
        dot_keys = self._dot_keys_for(label)
        # Stop flash timer
        if label in self._flash_timers:
            self._flash_timers[label].stop()
            del self._flash_timers[label]
        if success:
            self._launch_states[label] = True
            for key in dot_keys:
                if key in self.status_dots:
                    self.status_dots[key].setStyleSheet(self._DOT_ON)
            for i, (btn, blabel, _s) in enumerate(self._launch_nav_buttons):
                if blabel == label:
                    self._launch_nav_buttons[i] = (btn, blabel, self._launch_on_style)
                    btn.setStyleSheet(self._launch_on_style)
                    break
        else:
            self._launch_states[label] = False
            for key in dot_keys:
                if key in self.status_dots:
                    self.status_dots[key].setStyleSheet(self._DOT_OFF)
            for i, (btn, blabel, _s) in enumerate(self._launch_nav_buttons):
                if blabel == label:
                    self._launch_nav_buttons[i] = (btn, blabel, self._launch_btn_style)
                    btn.setStyleSheet(self._launch_btn_style)
                    break
        self._update_selection()
        self._refresh_terminal_display()
        self._process_queue()

    def _gui_log_msg(self, msg):
        """Append a timestamped info line to the GUI log and refresh terminal."""
        ts = time.strftime("%H:%M:%S")
        self._gui_log.append(f"[{ts}] {msg}\n")
        self._refresh_terminal_display()

    def _on_status_dot_clicked(self, name):
        """Handle click on a status dot button — show process output in terminal."""
        dev_label = self._dot_to_device.get(name)
        if dev_label is not None:
            self._selected_process = dev_label
        else:
            self._selected_process = name
        self._refresh_terminal_display()

    def _refresh_terminal_display(self):
        """Update the terminal QTextEdit with the selected process's output."""
        label = self._selected_process
        if label is None:
            # Show GUI log when no process is selected
            self._term_header.setText("GUI Info Log")
            if self._gui_log:
                self._term_display.setPlainText("".join(self._gui_log))
            else:
                self._term_display.setPlainText("No events yet.")
            sb = self._term_display.verticalScrollBar()
            sb.setValue(sb.maximum())
            return
        self._term_header.setText(f"Process: {label}")
        buf = self._process_buffers.get(label)
        if buf is not None:
            self._term_display.setPlainText("".join(buf))
        else:
            self._term_display.setPlainText(f"Process '{label}' is not running.\n")
        # Auto-scroll to bottom
        sb = self._term_display.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _poll_process_output(self):
        """5 Hz: clean up exited process handles, refresh terminal if selected."""
        for label in list(self._process_objects.keys()):
            proc = self._process_objects[label]
            if proc.poll() is not None:
                # Process exited — clean up handle but keep dots green
                buf = self._process_buffers.get(label)
                if buf is not None:
                    buf.append(f"[Process exited with code {proc.returncode}]\n")
                self._process_objects.pop(label, None)
                self._process_readers.pop(label, None)
        # Refresh terminal display if a process is selected
        if self._selected_process is not None:
            self._refresh_terminal_display()

    def closeEvent(self, event):
        """Terminate all subprocesses on window close."""
        for label, proc in list(self._process_objects.items()):
            self._kill_process(proc, label)
        self._process_objects.clear()
        super().closeEvent(event)

    def _cur_btn(self):
        return self._nav_groups[self._nav_col][self._nav_row]

    def _is_slider_selected(self):
        """Return True if the slider group is currently selected."""
        widget, _, _ = self._cur_btn()
        return widget is self.pb_slider

    def _is_speed_selected(self):
        """Return True if the speed button is currently selected."""
        widget, _, _ = self._cur_btn()
        return widget is self.btn_speed

    def keyPressEvent(self, event):
        key = event.key()

        # --- Scrub mode: arrows move the slider, Enter exits ---
        if self._scrub_mode:
            slider = self.pb_slider
            rng = slider.maximum() - slider.minimum()
            if rng <= 0:
                # No data loaded, exit scrub mode
                self._scrub_mode = False
                self._update_selection()
                return
            big_step = max(1, rng // 20)   # ~5% jump
            if key == Qt.Key_Right:
                slider.setValue(min(slider.value() + big_step, slider.maximum()))
                self._on_slider_seek(slider.value())
                self._position_indicators()
            elif key == Qt.Key_Left:
                slider.setValue(max(slider.value() - big_step, slider.minimum()))
                self._on_slider_seek(slider.value())
                self._position_indicators()
            elif key in (Qt.Key_Down, Qt.Key_Up):
                # Fine scrub: advance/retreat exactly one data row.
                # _pb_row_idx points to the NEXT unplayed row after a seek,
                # so Down uses it directly, Up goes back two (one before last played).
                idx = self._pb_row_idx
                if key == Qt.Key_Down:
                    idx = min(idx, len(self._pb_rows) - 1)
                else:
                    idx = max(idx - 2, 0)
                self._seek_to_row(idx)
                self._position_indicators()
            elif key in (Qt.Key_Return, Qt.Key_Enter):
                self._scrub_mode = False
                self._update_selection()
            return

        # --- Speed select mode: Up increases speed, Down decreases, Enter exits ---
        if self._speed_mode:
            if key == Qt.Key_Up:
                self._pb_speed_idx = min(self._pb_speed_idx + 1, len(self._pb_speed_options) - 1)
                self._apply_speed()
            elif key == Qt.Key_Down:
                self._pb_speed_idx = max(self._pb_speed_idx - 1, 0)
                self._apply_speed()
            elif key in (Qt.Key_Return, Qt.Key_Enter):
                self._speed_mode = False
                self._update_selection()
            return

        # --- Normal navigation ---
        group = self._nav_groups[self._nav_col]
        if key == Qt.Key_Up:
            self._nav_row = (self._nav_row - 1) % len(group)
            self._nav_last_row[self._nav_col] = self._nav_row
            self._update_selection()
        elif key == Qt.Key_Down:
            self._nav_row = (self._nav_row + 1) % len(group)
            self._nav_last_row[self._nav_col] = self._nav_row
            self._update_selection()
        elif key == Qt.Key_Right:
            if self._nav_col < len(self._nav_groups) - 1:
                self._nav_last_row[self._nav_col] = self._nav_row
                self._nav_col += 1
                self._nav_row = self._nav_last_row[self._nav_col]
                self._update_selection()
        elif key == Qt.Key_Left:
            if self._nav_col > 0:
                self._nav_last_row[self._nav_col] = self._nav_row
                self._nav_col -= 1
                self._nav_row = self._nav_last_row[self._nav_col]
                self._update_selection()
        elif key in (Qt.Key_Return, Qt.Key_Enter):
            if self._is_slider_selected():
                # Enter scrub mode — auto-pause if playing
                if self._pb_state == 'playing':
                    self._pause_playback()
                self._scrub_mode = True
                self._update_selection()
            elif self._is_speed_selected():
                # Enter speed select mode
                self._speed_mode = True
                self._update_selection()
            else:
                btn, _, _ = self._cur_btn()
                if btn.isEnabled():
                    btn.click()
        elif key == Qt.Key_Space:
            if self._pb_state in ('playing', 'paused', 'ended'):
                self._on_play_pause()
        else:
            super().keyPressEvent(event)

    def _update_selection(self):
        """Restyle all buttons: selected gets highlight + mirrored arrows, others reset."""
        for g, group in enumerate(self._nav_groups):
            for r, (widget, base_label, base_style) in enumerate(group):
                is_selected = (g == self._nav_col and r == self._nav_row)
                # Slider uses stylesheet only (no setText)
                if widget is self.pb_slider:
                    if is_selected and self._scrub_mode:
                        widget.setStyleSheet(self._slider_scrub_style)
                    elif is_selected:
                        widget.setStyleSheet(self._slider_sel_style)
                    else:
                        widget.setStyleSheet(self._slider_base_style)
                elif widget is self.btn_speed and is_selected and self._speed_mode:
                    widget.setText(
                        f"\u25B2  {base_label}  \u25BC"
                    )
                    widget.setStyleSheet(
                        self._speed_btn_style
                        .replace("border: 1px solid #555", "border: 1px solid #0f0")
                        .replace("color: #dcdcdc", "color: #0f0")
                    )
                else:
                    if is_selected:
                        widget.setText(
                            f"{self._sel_arrow_l}  {base_label}  {self._sel_arrow_r}"
                        )
                        widget.setStyleSheet(self._make_sel_style(base_style))
                    else:
                        widget.setText(base_label)
                        widget.setStyleSheet(base_style)
        # Position floating directional indicators around the selected widget
        self._position_indicators()

    def _position_indicators(self):
        """Show << and >> around the slider knob only when in scrub mode."""
        if not self._scrub_mode:
            self._ind_left.hide()
            self._ind_right.hide()
            return

        slider = self.pb_slider
        central = self.centralWidget()

        try:
            # Use Qt's style system to get the exact handle rect
            opt = QStyleOptionSlider()
            slider.initStyleOption(opt)
            handle_rect = slider.style().subControlRect(
                QStyle.CC_Slider, opt, QStyle.SC_SliderHandle, slider
            )
            # Map handle center to central widget coordinates
            handle_center = slider.mapTo(central, handle_rect.center())
            hx = handle_center.x()
            hy = handle_center.y()
            hw = handle_rect.width()
        except RuntimeError:
            self._ind_left.hide()
            self._ind_right.hide()
            return

        gap = 4

        # Left << (big step back)
        self._ind_left.adjustSize()
        self._ind_left.move(hx - hw // 2 - self._ind_left.width() - gap,
                            hy - self._ind_left.height() // 2)
        self._ind_left.show()
        self._ind_left.raise_()

        # Right >> (big step forward)
        self._ind_right.adjustSize()
        self._ind_right.move(hx + hw // 2 + gap,
                             hy - self._ind_right.height() // 2)
        self._ind_right.show()
        self._ind_right.raise_()

    def _nav_anim_tick(self):
        """Advance the spinning arrow animation."""
        self._sel_tick_count += 1
        if self._sel_tick_count >= self._sel_frame_durations[self._sel_frame_idx]:
            self._sel_tick_count = 0
            self._sel_frame_idx = (self._sel_frame_idx + 1) % len(self._sel_frames_r)
            self._sel_arrow_l = self._sel_frames_l[self._sel_frame_idx]
            self._sel_arrow_r = self._sel_frames_r[self._sel_frame_idx]
            widget, base_label, _ = self._cur_btn()
            # Slider doesn't support setText — animation only applies to buttons
            if widget is not self.pb_slider:
                widget.setText(
                    f"{self._sel_arrow_l}  {base_label}  {self._sel_arrow_r}"
                )

    def _set_btn_label(self, btn, new_label):
        """Update a nav button's base label and refresh display."""
        for g, group in enumerate(self._nav_groups):
            for r, (b, old_label, style) in enumerate(group):
                if b is btn:
                    self._nav_groups[g][r] = (b, new_label, style)
                    self._update_selection()
                    return

    # -- Topic -> sensor cell mapping --
    _TOPIC_CELL_MAP = {
        # Camera and Lidar handled via video mp4 playback
        '/gps_fix': 'GPS',
        '/encoders': 'Encoders',
        '/odom': 'Encoders',
        '/electrical/voltage': 'Power PCB',
        '/electrical/current': 'Power PCB',
        '/electrical/power': 'Power PCB',
    }

    POWER_WINDOW_S = 3.0

    # Default directory for playback CSVs
    # On the Jetson: /autonav/logs; fallback to example data next to this package
    _CSV_DIR = (
        '/autonav/logs'
        if os.path.isdir('/autonav/logs')
        else os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', 'python-gui-example', 'example-playback-csv',
        )
    )

    # -----------------------------------------------------------------
    # Playback engine
    # -----------------------------------------------------------------
    # -- Container connection ------------------------------------------------

    def _on_connect_container(self):
        """Toggle connection to the Docker container."""
        if self._container_connected:
            self._disconnect_container()
        else:
            self._connect_container()

    def _connect_container(self):
        """Check if the container is running and enable container features."""
        try:
            result = subprocess.run(
                ['docker', 'ps', '--quiet', '--filter', 'status=running',
                 '--filter', f'name=^/{self._container_name}$'],
                capture_output=True, text=True, timeout=5,
            )
            if not result.stdout.strip():
                self._gui_log_msg(
                    f"Container '{self._container_name}' is not running. "
                    "Start it with ./env/docker/run-container.sh"
                )
                return
        except FileNotFoundError:
            self._gui_log_msg("Docker not found on this system.")
            return
        except subprocess.TimeoutExpired:
            self._gui_log_msg("Docker command timed out.")
            return

        self._container_connected = True
        self._gui_log_msg(f"Connected to container '{self._container_name}'")

        # Update button to show connected state
        connected_style = (
            self._connect_style
            .replace("#1a2a3a", "#1a3a1a")
            .replace("#2a3a4a", "#2a4a2a")
            .replace("#0a1a2a", "#0a2a0a")
        )
        self.btn_connect.setText("Disconnect Container")
        self.btn_connect.setStyleSheet(connected_style)
        for i, (b, lbl, _s) in enumerate(self._nav_buttons):
            if b is self.btn_connect:
                self._nav_buttons[i] = (b, "Disconnect Container", connected_style)
                break

        # Enable container-dependent buttons
        for btn_ref, base_label in self._container_buttons.items():
            btn_ref.setEnabled(True)
            for _b, _l, base_s in self._nav_buttons:
                if _b is btn_ref:
                    btn_ref.setStyleSheet(base_s)
                    break
        # Turn container dots green
        for dot in self._container_dots:
            dot.setStyleSheet(
                "background-color: #4f4; border-radius: 5px; border: none;"
            )

    def _disconnect_container(self):
        """Disconnect from the container and disable container features."""
        self._container_connected = False
        self._gui_log_msg(f"Disconnected from container '{self._container_name}'")

        # Restore connect button
        self.btn_connect.setText("Connect to Container")
        self.btn_connect.setStyleSheet(self._connect_style)
        for i, (b, lbl, _s) in enumerate(self._nav_buttons):
            if b is self.btn_connect:
                self._nav_buttons[i] = (b, "Connect to Container", self._connect_style)
                break

        # Disable container-dependent buttons and show warnings
        for btn_ref, base_label in self._container_buttons.items():
            btn_ref.setEnabled(False)
            btn_ref.setStyleSheet(self._disabled_btn_style)
        # Turn container dots red
        for dot in self._container_dots:
            dot.setStyleSheet(
                "background-color: #f44; border-radius: 5px; border: none;"
            )

        # Stop any active container modes
        if self._live_active:
            self._stop_live_mode()

    def _wrap_container_cmd(self, cmd, label=None):
        """Wrap a command to run inside the Docker container via docker exec.
        Writes the shell PID to /tmp/gui_pid_{label} so we can kill the
        entire process tree later."""
        if not self._container_connected:
            return cmd
        # Escape single quotes in cmd for safe embedding
        safe_cmd = cmd.replace("'", "'\\''")
        # Use a sanitized label for the PID file
        pid_tag = (label or 'unknown').replace(' ', '_').replace('/', '_')
        return (
            f"docker exec "
            f"-u {self._container_user} "
            f"-e HOME=/home/{self._container_user} "
            f"-e USER={self._container_user} "
            f"--workdir {self._container_workdir} "
            f"{self._container_name} "
            f"/bin/bash -lc "
            f"'echo $$ > /tmp/gui_pid_{pid_tag} && "
            f"source /opt/ros/humble/setup.bash && "
            f"if [ -f {self._container_workdir}/install/setup.bash ]; then "
            f"source {self._container_workdir}/install/setup.bash; fi && "
            f"exec {safe_cmd}'"
        )

    def _kill_process(self, proc, label):
        """Kill a launched process. If connected to a container, read the
        saved PID and kill the entire process tree inside the container."""
        if self._container_connected:
            pid_tag = label.replace(' ', '_').replace('/', '_')
            try:
                # Read the PID we saved at launch, then kill its process
                # tree with SIGINT (graceful ROS shutdown)
                kill_cmd = (
                    f"docker exec -u root {self._container_name} "
                    f"/bin/bash -c \""
                    f"PID=$(cat /tmp/gui_pid_{pid_tag} 2>/dev/null) && "
                    f"if [ -n \\\"$PID\\\" ]; then "
                    f"  kill -INT -- -$PID 2>/dev/null; "
                    f"  sleep 2; "
                    f"  kill -9 -- -$PID 2>/dev/null; "
                    f"  rm -f /tmp/gui_pid_{pid_tag}; "
                    f"fi\""
                )
                subprocess.run(kill_cmd, shell=True, timeout=10,
                               capture_output=True)
            except Exception:
                pass
        # Also terminate the local wrapper process
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    def _launch_cmd_for(self, label):
        """Return the raw command string for a device/test label."""
        for dev_label, _keys, dev_cmd in self._launch_devices:
            if dev_label == label:
                return dev_cmd
        return label

    def _on_playback_clicked(self):
        """Show the playback CSV selection page."""
        if self._live_active:
            self._stop_live_mode()
        self._show_playback_page()

    def _load_csv(self, path):
        self._gui_log_msg(f"Loading CSV: {os.path.basename(path)}")
        rows = []
        gps_lats, gps_lons = [], []
        odom_xs, odom_ys = [], []
        with open(path, newline='') as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for r in reader:
                if len(r) < 4:
                    continue
                ts = int(r[0])
                topic = r[1]
                keys = r[2].split(',')
                values = r[3:]
                rows.append((ts, topic, keys, values))
                if topic == '/gps_fix' and len(values) >= 2:
                    try:
                        gps_lats.append(float(values[0]))
                        gps_lons.append(float(values[1]))
                    except ValueError:
                        pass
                elif topic == '/odom' and len(values) >= 2:
                    try:
                        odom_xs.append(float(values[0]))
                        odom_ys.append(float(values[1]))
                    except ValueError:
                        pass
        rows.sort(key=lambda x: x[0])
        t0 = rows[0][0]
        self._pb_rows = [(ts - t0, topic, keys, vals) for ts, topic, keys, vals in rows]
        self._pb_duration_ns = self._pb_rows[-1][0]
        duration_ms = int(self._pb_duration_ns / 1e6)
        self.pb_slider.setRange(0, duration_ms)
        self.pb_slider.setValue(0)
        self.pb_slider.setEnabled(True)
        total_s = self._pb_duration_ns / 1e9
        self.pb_time_label.setText(f"0.0s / {total_s:.1f}s")
        self._pb_total_frames = len(self._pb_rows)
        self.pb_frame_label.setText(f"F: 0 / {self._pb_total_frames}")

        # GPS and Odom windows adjust dynamically in _redraw_plots

        # Discover companion mp4 video files
        self._release_video_captures()
        if _HAS_CV2:
            csv_stem = os.path.splitext(path)[0]
            cam_mp4 = csv_stem + '_camera.mp4'
            lidar_mp4 = csv_stem + '_lidar_bev.mp4'
            if os.path.isfile(cam_mp4):
                self._camera_cap = cv2.VideoCapture(cam_mp4)
            if os.path.isfile(lidar_mp4):
                self._lidar_cap = cv2.VideoCapture(lidar_mp4)

        # Show "no video" text when mp4 files are missing
        has_cam = self._camera_cap is not None
        has_lidar = self._lidar_cap is not None
        self._cam_no_video_txt.set_visible(not has_cam)
        self._lidar_no_video_txt.set_visible(not has_lidar)
        self._cam_canvas.draw_idle()
        self._lidar_canvas.draw_idle()

        # Pre-fetch OSM map tiles for GPS background
        if gps_lats and gps_lons:
            self._gps_no_data_txt.set_visible(False)
            img, extent = _fetch_map_for_gps(gps_lats, gps_lons)
            if img is not None:
                self._gps_map_img = img
                self._gps_map_extent = extent
                # Draw the map background once
                if self._gps_map_im is not None:
                    self._gps_map_im.remove()
                self._gps_map_im = self._gps_ax.imshow(
                    self._gps_map_img,
                    extent=[extent[0], extent[1], extent[2], extent[3]],
                    aspect='auto', zorder=0,
                )
        else:
            self._gps_no_data_txt.set_visible(True)
        self._gps_canvas.draw_idle()

    def _clear_buffers(self):
        self._power_buf = {'t': deque(), 'V': deque(), 'I': deque(), 'P': deque()}
        self._gps_buf = {'lat': [], 'lon': []}
        self._odom_buf = {'x': [], 'y': [], 'theta': []}
        self._update_soc(0.0, force=True)
        if self._odom_tri_patch:
            self._odom_tri_patch.remove()
            self._odom_tri_patch = None
        # Clear video imshow handles
        self._cam_im = None
        self._lidar_im = None

    def _release_video_captures(self):
        """Release any open VideoCapture objects."""
        if self._camera_cap is not None:
            self._camera_cap.release()
            self._camera_cap = None
        if self._lidar_cap is not None:
            self._lidar_cap.release()
            self._lidar_cap = None

    def _update_video_frames(self, elapsed_ns):
        """Seek both video captures to the correct frame and update canvases."""
        if not _HAS_CV2:
            return
        elapsed_s = elapsed_ns / 1e9
        frame_idx = int(elapsed_s * self._video_fps)

        for cap, ax, canvas, attr, no_txt, rotate in [
            (self._camera_cap, self._cam_ax, self._cam_canvas, '_cam_im',
             self._cam_no_video_txt, False),
            (self._lidar_cap, self._lidar_ax, self._lidar_canvas, '_lidar_im',
             self._lidar_no_video_txt, True),
        ]:
            if cap is None or not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            ret, bgr = cap.read()
            if not ret:
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            if rotate:
                rgb = np.rot90(rgb, 2)
                rgb = np.fliplr(rgb)
                # Crop bottom 1/3 (behind robot, no data with 180° FOV)
                h, w = rgb.shape[:2]
                rgb = rgb[:h * 2 // 3, :]
                # Display with meter-based extent so axes show distance
                half_m = 10.0  # max range in meters
                # Full image spans -10 to 10; cropped top 2/3 spans ~+10 down to -3.3
                y_bottom = -half_m + (2.0 * half_m) / 3.0  # ≈ -3.33
                extent = [-half_m, half_m, y_bottom, half_m]
                no_txt.set_visible(False)
                im_handle = getattr(self, attr)
                if im_handle is None:
                    ax.axis('on')
                    im_handle = ax.imshow(rgb, aspect='equal', extent=extent)
                    ax.set_xlim(-half_m, half_m)
                    ax.set_ylim(y_bottom, half_m)
                    ax.tick_params(axis='both', length=2, pad=2,
                                  labelsize=6, colors='#888', direction='in')
                    ax.set_xlabel('m', fontsize=6, color='#888', labelpad=1)
                    for spine in ax.spines.values():
                        spine.set_color('#444')
                    setattr(self, attr, im_handle)
                else:
                    im_handle.set_data(rgb)
                    im_handle.set_extent(extent)
                canvas.draw_idle()
                continue
            no_txt.set_visible(False)
            im_handle = getattr(self, attr)
            if im_handle is None:
                im_handle = ax.imshow(rgb, aspect='equal')
                setattr(self, attr, im_handle)
            else:
                im_handle.set_data(rgb)
            canvas.draw_idle()

        # Light up Camera/Lidar dots if videos are playing
        if self._camera_cap is not None and self._camera_cap.isOpened():
            dot = self.status_dots.get('Camera')
            if dot:
                dot.setStyleSheet(self._DOT_ON)
        if self._lidar_cap is not None and self._lidar_cap.isOpened():
            dot = self.status_dots.get('Lidar')
            if dot:
                dot.setStyleSheet(self._DOT_ON)

    def _on_play_pause(self):
        if self._pb_state == 'playing':
            self._pause_playback()
        elif self._pb_state == 'paused':
            self._resume_playback()
        elif self._pb_state == 'ended':
            # Restart from beginning
            self._seek_to_row(0)
            self._resume_playback()

    def _cycle_playback_speed(self):
        """Cycle through playback speed options (used by mouse click)."""
        self._pb_speed_idx = (self._pb_speed_idx + 1) % len(self._pb_speed_options)
        self._apply_speed()

    def _apply_speed(self):
        """Apply the current speed index: update label, adjust wall-clock."""
        self._pb_speed = self._pb_speed_options[self._pb_speed_idx]
        self._gui_log_msg(f"Playback speed: {self._pb_speed:g}x")
        label = f"{self._pb_speed:g}x"
        self.btn_speed.setText(label)
        self._set_btn_label(self.btn_speed, label)
        # Adjust wall-clock start so position stays consistent
        if self._pb_state == 'playing':
            elapsed_ns = self._pb_elapsed_ns
            self._pb_wall_start = time.monotonic() - elapsed_ns / (1e9 * self._pb_speed)
        elif self._pb_state == 'paused':
            elapsed_ns = self._pb_pause_elapsed_ns
            self._pb_wall_start = time.monotonic() - elapsed_ns / (1e9 * self._pb_speed)

    def _start_playback(self):
        self._gui_log_msg("Playback started")
        if self._live_active:
            self._stop_live_mode()
        self._pb_state = 'playing'
        self._pb_row_idx = 0
        self._pb_elapsed_ns = 0
        self._pb_wall_start = time.monotonic()
        self._clear_buffers()
        self.btn_pp.setEnabled(True)
        self._set_btn_label(self.btn_pp, "\u275A\u275A")
        # Highlight Playback Mode button green
        self._set_nav_btn_style(self.btn_playback, self._toggle_on_style)
        # Mark TEST dot as active (green) during playback
        if "TEST" in self.status_dots:
            self.status_dots["TEST"].setStyleSheet(
                "background-color: #0f0; border-radius: 7px; border: none;"
            )
        self._pb_timer = QTimer()
        self._pb_timer.setInterval(50)
        self._pb_timer.timeout.connect(self._playback_tick)
        self._pb_timer.start()

    def _pause_playback(self):
        self._gui_log_msg("Playback paused")
        self._pb_state = 'paused'
        if self._pb_timer:
            self._pb_timer.stop()
        self._pb_pause_elapsed_ns = (time.monotonic() - self._pb_wall_start) * 1e9 * self._pb_speed
        self._set_btn_label(self.btn_pp, "\u25B6")

    def _resume_playback(self):
        self._gui_log_msg("Playback resumed")
        self._pb_state = 'playing'
        self._pb_wall_start = time.monotonic() - self._pb_pause_elapsed_ns / (1e9 * self._pb_speed)
        self._set_btn_label(self.btn_pp, "\u275A\u275A")
        if self._pb_timer:
            self._pb_timer.start()

    def _stop_playback(self):
        self._pb_state = 'idle'
        if self._pb_timer:
            self._pb_timer.stop()
            self._pb_timer = None
        self._release_video_captures()
        self._reset_all_plots()
        self._set_btn_label(self.btn_pp, "\u25B6")
        self.btn_pp.setEnabled(False)
        # Restore Playback Mode button to normal style
        self._set_nav_btn_style(self.btn_playback, self._pb_button_style)
        for dot in self.status_dots.values():
            dot.setStyleSheet(
                "background-color: #555; border-radius: 7px; border: none;"
            )

    def _reset_all_plots(self):
        """Reset all sensor plots back to their default empty state."""
        # -- Camera --
        if self._cam_im is not None:
            self._cam_im.remove()
            self._cam_im = None
        self._cam_no_video_txt.set_visible(False)
        self._cam_live_txt.set_visible(False)
        self._cam_ax.set_facecolor('#111111')
        self._cam_canvas.draw_idle()

        # -- Lidar --
        if self._lidar_im is not None:
            self._lidar_im.remove()
            self._lidar_im = None
        self._lidar_no_video_txt.set_visible(False)
        self._lidar_live_txt.set_visible(False)
        self._lidar_ax.set_facecolor('#111111')
        self._lidar_canvas.draw_idle()

        # -- GPS --
        self._gps_trail.set_data([], [])
        self._gps_dot.set_data([], [])
        self._gps_coord_label.set_text('')
        if self._gps_map_im is not None:
            self._gps_map_im.remove()
            self._gps_map_im = None
        self._gps_map_img = None
        self._gps_map_extent = None
        self._gps_no_data_txt.set_visible(False)
        self._gps_live_txt.set_visible(False)
        self._gps_ax.set_facecolor('#111111')
        self._gps_canvas.draw_idle()

        # -- Encoders (Odom) --
        if self._odom_scatter is not None:
            self._odom_scatter.remove()
            self._odom_scatter = None
        if self._odom_tri_patch is not None:
            self._odom_tri_patch.remove()
            self._odom_tri_patch = None
        self._odom_dist_label.set_text('')
        self._odom_live_txt.set_visible(False)
        self._odom_ax.set_xlim(-5, 5)
        self._odom_ax.set_ylim(-5, 5)
        self._odom_canvas.draw_idle()

        # -- Power PCB --
        self._pwr_line_v.set_data([], [])
        self._pwr_line_i.set_data([], [])
        self._pwr_line_p.set_data([], [])
        self._pwr_v_live_txt.set_visible(False)
        self._pwr_i_live_txt.set_visible(False)
        self._pwr_p_live_txt.set_visible(False)
        self._pwr_val_v.setText("V: --")
        self._pwr_val_i.setText("I: --")
        self._pwr_val_p.setText("P: --")
        self._pwr_v_canvas.draw_idle()
        self._pwr_i_canvas.draw_idle()
        self._pwr_p_canvas.draw_idle()
        self._update_soc(0.0, force=True)

        # -- Clear data buffers --
        self._clear_buffers()

    def _playback_tick(self):
        elapsed_ns = (time.monotonic() - self._pb_wall_start) * 1e9 * self._pb_speed
        self._pb_elapsed_ns = elapsed_ns

        while self._pb_row_idx < len(self._pb_rows):
            rel_ns, topic, keys, vals = self._pb_rows[self._pb_row_idx]
            if rel_ns > elapsed_ns:
                break
            self._apply_row(rel_ns, topic, keys, vals)
            self._pb_row_idx += 1

        if self._plots_dirty:
            self._redraw_plots()
            self._plots_dirty = False

        self._update_video_frames(elapsed_ns)

        elapsed_ms = int(elapsed_ns / 1e6)
        duration_ms = int(self._pb_duration_ns / 1e6)
        self.pb_slider.blockSignals(True)
        self.pb_slider.setValue(min(elapsed_ms, duration_ms))
        self.pb_slider.blockSignals(False)
        elapsed_s = elapsed_ns / 1e9
        total_s = self._pb_duration_ns / 1e9
        self.pb_time_label.setText(f"{elapsed_s:.1f}s / {total_s:.1f}s")
        self.pb_frame_label.setText(
            f"F: {self._pb_row_idx} / {self._pb_total_frames}"
        )

        if self._pb_row_idx >= len(self._pb_rows):
            # Pause at the end — user can press play to restart from beginning
            self._pb_state = 'ended'
            if self._pb_timer:
                self._pb_timer.stop()
            self._set_btn_label(self.btn_pp, "\u25B6")

    def _apply_row(self, rel_ns, topic, keys, values):
        cell_name = self._TOPIC_CELL_MAP.get(topic)
        if not cell_name:
            return

        dot = self.status_dots.get(cell_name)
        if dot:
            dot.setStyleSheet(
                "background-color: #0f0; border-radius: 7px; border: none;"
            )

        t_s = rel_ns / 1e9
        self._plots_dirty = True

        if topic == '/electrical/voltage':
            self._power_buf['t'].append(t_s)
            self._power_buf['V'].append(float(values[0]))
            self._power_buf['I'].append(self._power_buf['I'][-1] if self._power_buf['I'] else 0)
            self._power_buf['P'].append(self._power_buf['P'][-1] if self._power_buf['P'] else 0)
            self._trim_power_buf(t_s)
        elif topic == '/electrical/current':
            self._power_buf['t'].append(t_s)
            self._power_buf['V'].append(self._power_buf['V'][-1] if self._power_buf['V'] else 0)
            self._power_buf['I'].append(float(values[0]))
            self._power_buf['P'].append(self._power_buf['P'][-1] if self._power_buf['P'] else 0)
            self._trim_power_buf(t_s)
        elif topic == '/electrical/power':
            self._power_buf['t'].append(t_s)
            self._power_buf['V'].append(self._power_buf['V'][-1] if self._power_buf['V'] else 0)
            self._power_buf['I'].append(self._power_buf['I'][-1] if self._power_buf['I'] else 0)
            self._power_buf['P'].append(float(values[0]))
            self._trim_power_buf(t_s)
        elif topic == '/gps_fix':
            try:
                lat = float(values[0])
                lon = float(values[1])
                self._gps_buf['lat'].append(lat)
                self._gps_buf['lon'].append(lon)
            except (IndexError, ValueError):
                pass
        elif topic == '/odom':
            try:
                x = float(values[0])
                y = float(values[1])
                qz = float(values[2]) if len(values) > 2 else 0.0
                # Convert quaternion z to yaw: yaw = 2 * asin(qz)
                qz_clamped = max(-1.0, min(1.0, qz))
                yaw = 2.0 * math.asin(qz_clamped)
                self._odom_buf['x'].append(x)
                self._odom_buf['y'].append(y)
                self._odom_buf['theta'].append(yaw)
            except (IndexError, ValueError):
                pass
        # Camera and Lidar handled via video mp4 playback

    def _trim_power_buf(self, t_now):
        cutoff = t_now - self.POWER_WINDOW_S
        while self._power_buf['t'] and self._power_buf['t'][0] < cutoff:
            self._power_buf['t'].popleft()
            self._power_buf['V'].popleft()
            self._power_buf['I'].popleft()
            self._power_buf['P'].popleft()

    def _update_soc(self, fraction, force=False):
        """Update SOC gauge bar. fraction is 0.0–1.0.
        Hysteresis: only update display if change > 1% to avoid flicker."""
        pct = int(round(fraction * 100))
        if not force and hasattr(self, '_soc_display_pct') and abs(pct - self._soc_display_pct) <= 1:
            return
        self._soc_display_pct = pct
        fill = max(pct, 0)
        empty = max(100 - pct, 0)
        # Fill widget (index 0) = filled portion, spacer (index 1) = empty portion
        self._soc_bar_layout.setStretch(0, fill)
        self._soc_bar_layout.setStretch(1, empty)
        self._soc_label.setText(f"{pct}%")
        # Color: green > 50%, yellow 20-50%, red < 20%
        if fraction > 0.5:
            color = "#4f4"
        elif fraction > 0.2:
            color = "#ff4"
        else:
            color = "#f44"
        self._soc_fill.setStyleSheet(f"background-color: {color}; border: none; border-radius: 1px;")

    def _redraw_plots(self):
        # --- Power mini oscilloscopes (fixed Y axes) ---
        t = list(self._power_buf['t'])
        if t:
            self._pwr_v_live_txt.set_visible(False)
            self._pwr_i_live_txt.set_visible(False)
            self._pwr_p_live_txt.set_visible(False)
            xlim = (t[-1] - self.POWER_WINDOW_S, t[-1])
            for line, buf_key, ax, canvas in [
                (self._pwr_line_v, 'V', self._pwr_v_ax, self._pwr_v_canvas),
                (self._pwr_line_i, 'I', self._pwr_i_ax, self._pwr_i_canvas),
                (self._pwr_line_p, 'P', self._pwr_p_ax, self._pwr_p_canvas),
            ]:
                line.set_data(t, list(self._power_buf[buf_key]))
                ax.set_xlim(*xlim)
                ax.set_xticklabels([])
                canvas.draw_idle()
            # Update numeric readouts with latest values
            self._pwr_val_v.setText(f"V: {self._power_buf['V'][-1]:.2f}")
            self._pwr_val_i.setText(f"I: {self._power_buf['I'][-1]:.2f}")
            self._pwr_val_p.setText(f"P: {self._power_buf['P'][-1]:.2f}")
            # Derive SOC from voltage (linear 20V–29.4V → 0%–100%)
            v = self._power_buf['V'][-1]
            soc = max(0.0, min(1.0, (v - 20.0) / (29.4 - 20.0)))
            self._update_soc(soc)
        else:
            self._pwr_v_live_txt.set_visible(True)
            self._pwr_i_live_txt.set_visible(True)
            self._pwr_p_live_txt.set_visible(True)
            self._pwr_val_v.setText("V: --")
            self._pwr_val_i.setText("I: --")
            self._pwr_val_p.setText("P: --")
            self._pwr_v_canvas.draw_idle()
            self._pwr_i_canvas.draw_idle()
            self._pwr_p_canvas.draw_idle()

        # --- GPS with satellite map, single dot + faint trail (100 ft window) ---
        lats = self._gps_buf['lat']
        lons = self._gps_buf['lon']
        if lons:
            # Faint trail of all previous points
            self._gps_trail.set_data(lons, lats)
            # Current position dot
            self._gps_dot.set_data([lons[-1]], [lats[-1]])
            # 100 ft window centered on current position
            cur_lat, cur_lon = lats[-1], lons[-1]
            dlat = _GPS_VIEW_RADIUS_M / 111320.0
            dlon = _GPS_VIEW_RADIUS_M / (111320.0 * math.cos(math.radians(cur_lat)))
            self._gps_ax.set_xlim(cur_lon - dlon, cur_lon + dlon)
            self._gps_ax.set_ylim(cur_lat - dlat, cur_lat + dlat)
            # Show lat/lon truncated to 4 decimals
            self._gps_coord_label.set_text(
                f"Lat: {cur_lat:.4f}\nLon: {cur_lon:.4f}"
            )
            self._gps_canvas.draw_idle()

        # --- Odom XY with time-colored scatter + direction triangle ---
        xs = self._odom_buf['x']
        ys = self._odom_buf['y']
        thetas = self._odom_buf['theta']
        if xs:
            # Remove old scatter and recreate with color gradient
            if self._odom_scatter is not None:
                self._odom_scatter.remove()
                self._odom_scatter = None
            colors = np.linspace(0, 1, len(xs))
            self._odom_scatter = self._odom_ax.scatter(
                xs, ys, c=colors, cmap='coolwarm', s=4, zorder=3,
            )
            if self._odom_tri_patch:
                self._odom_tri_patch.remove()
                self._odom_tri_patch = None
            # Adjust window to fit all current points with padding
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            dx = max_x - min_x
            dy = max_y - min_y
            span = max(dx, dy) * 1.3
            span = max(span, 1.0)
            half = span / 2
            cx_view = (min_x + max_x) / 2
            cy_view = (min_y + max_y) / 2
            self._odom_ax.set_xlim(cx_view - half, cx_view + half)
            self._odom_ax.set_ylim(cy_view - half, cy_view + half)
            # Adaptive grid: pick from 1,2,5,10,20,50,... so ~4-8 lines visible
            import matplotlib.ticker as mticker
            nice = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]
            target = span / 5
            grid_sp = nice[0]
            for n in nice:
                grid_sp = n
                if n >= target:
                    break
            self._odom_ax.xaxis.set_major_locator(mticker.MultipleLocator(grid_sp))
            self._odom_ax.yaxis.set_major_locator(mticker.MultipleLocator(grid_sp))
            # Distance traveled
            dist = 0.0
            for i in range(1, len(xs)):
                dist += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
            self._odom_dist_label.set_text(f"Dist: {dist:.2f} m")

            cx, cy = xs[-1], ys[-1]
            theta = thetas[-1] if thetas else 0.0
            s = span * 0.05
            s = max(s, 0.1)
            nose = (cx + s * math.cos(theta),
                    cy + s * math.sin(theta))
            bl = (cx + s * 0.5 * math.cos(theta + 2.5),
                  cy + s * 0.5 * math.sin(theta + 2.5))
            br = (cx + s * 0.5 * math.cos(theta - 2.5),
                  cy + s * 0.5 * math.sin(theta - 2.5))
            tri = Polygon([nose, bl, br], closed=True,
                          facecolor='red', edgecolor='white',
                          linewidth=0.8, zorder=5)
            self._odom_ax.add_patch(tri)
            self._odom_tri_patch = tri
            self._odom_canvas.draw_idle()

    def _on_slider_pressed(self):
        if self._pb_state == 'playing' and self._pb_timer:
            self._pb_timer.stop()

    def _on_slider_released(self):
        if self._pb_state == 'playing' and self._pb_timer:
            self._pb_timer.start()

    def _seek_to_row(self, target_idx):
        """Seek directly to a specific row index (nanosecond-precise)."""
        target_idx = max(0, min(target_idx, len(self._pb_rows) - 1))
        # Replay all rows up to and including target_idx
        target_ns = self._pb_rows[target_idx][0]
        self._pb_wall_start = time.monotonic() - target_ns / 1e9
        if self._pb_state == 'paused':
            self._pb_pause_elapsed_ns = target_ns
        self._pb_elapsed_ns = target_ns

        self._clear_buffers()
        for i in range(target_idx + 1):
            rel_ns, topic, keys, vals = self._pb_rows[i]
            self._apply_row(rel_ns, topic, keys, vals)
        self._pb_row_idx = target_idx + 1

        self._redraw_plots()
        self._update_video_frames(target_ns)

        # Update slider and labels
        self.pb_slider.blockSignals(True)
        self.pb_slider.setValue(int(target_ns / 1e6))
        self.pb_slider.blockSignals(False)
        elapsed_s = target_ns / 1e9
        total_s = self._pb_duration_ns / 1e9
        self.pb_time_label.setText(f"{elapsed_s:.1f}s / {total_s:.1f}s")
        self.pb_frame_label.setText(
            f"F: {self._pb_row_idx} / {self._pb_total_frames}"
        )

    def _on_slider_seek(self, position_ms):
        target_ns = position_ms * 1e6
        self._pb_wall_start = time.monotonic() - target_ns / 1e9
        if self._pb_state == 'paused':
            self._pb_pause_elapsed_ns = target_ns
        self._pb_elapsed_ns = target_ns

        self._clear_buffers()
        self._pb_row_idx = 0
        for i, (rel_ns, topic, keys, vals) in enumerate(self._pb_rows):
            if rel_ns > target_ns:
                self._pb_row_idx = i
                break
            self._apply_row(rel_ns, topic, keys, vals)
        else:
            self._pb_row_idx = len(self._pb_rows)

        # Reset all dots then re-light active ones
        for dot in self.status_dots.values():
            dot.setStyleSheet(
                "background-color: #555; border-radius: 7px; border: none;"
            )
        active_cells = set()
        if self._gps_buf['lat']:
            active_cells.add('GPS')
        if self._odom_buf['x']:
            active_cells.add('Encoders')
        if self._power_buf['t']:
            active_cells.add('Power PCB')
        if self._camera_cap is not None:
            active_cells.add('Camera')
        if self._lidar_cap is not None:
            active_cells.add('Lidar')
        for cell in active_cells:
            dot = self.status_dots.get(cell)
            if dot:
                dot.setStyleSheet(
                    "background-color: #0f0; border-radius: 7px; border: none;"
                )

        self._redraw_plots()
        self._update_video_frames(target_ns)

        elapsed_s = target_ns / 1e9
        total_s = self._pb_duration_ns / 1e9
        self.pb_time_label.setText(f"{elapsed_s:.1f}s / {total_s:.1f}s")
        self.pb_frame_label.setText(
            f"F: {self._pb_row_idx} / {self._pb_total_frames}"
        )

    # -----------------------------------------------------------------
    # Live Mode
    # -----------------------------------------------------------------
    def _on_live_clicked(self):
        """Toggle Live Mode on/off."""
        if self._live_active:
            self._stop_live_mode()
        else:
            self._start_live_mode()

    def _start_live_mode(self):
        """Activate Live Mode: subscribe to ROS topics at 10 Hz."""
        self._gui_log_msg("Live Mode activated")
        # Stop playback if running
        if self._pb_state != 'idle':
            self._stop_playback()

        self._live_active = True
        self._live_t0 = time.monotonic()
        self._clear_buffers()

        # Button styles: Live green, Playback normal
        self._set_nav_btn_style(self.btn_live, self._toggle_on_style)
        self._set_nav_btn_style(self.btn_playback, self._pb_button_style)

        # Slider: show LIVE label, disable scrubbing
        self.pb_time_label.setText("LIVE")
        self.pb_frame_label.setText("")
        self.pb_slider.setEnabled(False)
        self.pb_slider.setRange(0, 0)
        self.btn_pp.setEnabled(False)

        # Show "NO DATA AVAILABLE" placeholders (hidden when data arrives)
        self._cam_live_txt.set_visible(True)
        self._cam_no_video_txt.set_visible(False)
        self._cam_canvas.draw_idle()
        self._lidar_live_txt.set_visible(True)
        self._lidar_no_video_txt.set_visible(False)
        self._lidar_canvas.draw_idle()
        self._gps_live_txt.set_visible(True)
        self._gps_no_data_txt.set_visible(False)
        # Clear GPS map background for a clean black screen
        if self._gps_map_im is not None:
            self._gps_map_im.remove()
            self._gps_map_im = None
        self._gps_map_extent = None
        self._gps_trail.set_data([], [])
        self._gps_dot.set_data([], [])
        self._gps_coord_label.set_text('')
        self._gps_canvas.draw_idle()
        # Encoders: show live placeholder and clear plot
        self._odom_live_txt.set_visible(True)
        self._odom_ax.grid(False)
        self._odom_ax.tick_params(labelbottom=False, labelleft=False)
        self._odom_xy_label.set_visible(False)
        if self._odom_scatter is not None:
            self._odom_scatter.remove()
            self._odom_scatter = None
        if self._odom_tri_patch is not None:
            self._odom_tri_patch.remove()
            self._odom_tri_patch = None
        self._odom_dist_label.set_text('')
        self._odom_canvas.draw_idle()
        self._pwr_v_live_txt.set_visible(True)
        self._pwr_i_live_txt.set_visible(True)
        self._pwr_p_live_txt.set_visible(True)
        self._pwr_v_canvas.draw_idle()
        self._pwr_i_canvas.draw_idle()
        self._pwr_p_canvas.draw_idle()

        # Flash all sensor dots yellow/gray until data arrives
        self._live_received = set()  # dot names that have received data
        waiting = [n for n in self.status_dots if n != "TEST"]
        self._gui_log_msg("Awaiting live data from: " + ", ".join(waiting))
        self._live_flash_state = True
        self._live_dot_flash_timer = QTimer()
        self._live_dot_flash_timer.setInterval(200)
        self._live_dot_flash_timer.timeout.connect(self._live_dot_flash_tick)
        self._live_dot_flash_timer.start()
        # Set all dots to yellow initially (except TEST)
        for name, dot in self.status_dots.items():
            if name != "TEST":
                dot.setStyleSheet(self._DOT_YELLOW)

        # Start 10 Hz timer
        self._live_timer = QTimer()
        self._live_timer.setInterval(100)
        self._live_timer.timeout.connect(self._live_tick)
        self._live_timer.start()

    def _live_dot_flash_tick(self):
        """Flash dots yellow/gray for sensors that haven't received data yet."""
        self._live_flash_state = not self._live_flash_state
        style = self._DOT_YELLOW if self._live_flash_state else self._DOT_OFF
        for name, dot in self.status_dots.items():
            if name == "TEST":
                continue
            if name not in self._live_received:
                dot.setStyleSheet(style)

    def _live_set_dot_received(self, name):
        """Mark a sensor dot as having received data — stop flashing, go green."""
        if name not in self._live_received:
            self._live_received.add(name)
            self._gui_log_msg(f"Receiving live data from: {name}")
            dot = self.status_dots.get(name)
            if dot:
                dot.setStyleSheet(self._DOT_ON)

    def _stop_live_mode(self):
        """Deactivate Live Mode."""
        self._gui_log_msg("Live Mode deactivated")
        self._live_active = False
        if self._live_timer is not None:
            self._live_timer.stop()
            self._live_timer = None

        # Stop dot flash timer
        if hasattr(self, '_live_dot_flash_timer') and self._live_dot_flash_timer is not None:
            self._live_dot_flash_timer.stop()
            self._live_dot_flash_timer = None

        # Restore button style
        self._set_nav_btn_style(self.btn_live, self._pb_button_style)

        # Reset dots
        for dot in self.status_dots.values():
            dot.setStyleSheet(self._DOT_OFF)

        # Hide live placeholders, clear imshow handles
        self._cam_live_txt.set_visible(False)
        self._cam_canvas.draw_idle()
        self._lidar_live_txt.set_visible(False)
        self._lidar_canvas.draw_idle()
        self._gps_live_txt.set_visible(False)
        self._gps_canvas.draw_idle()
        self._odom_live_txt.set_visible(False)
        self._odom_ax.grid(True, which='both', color='#333', linewidth=0.5)
        self._odom_ax.tick_params(labelbottom=True, labelleft=True)
        self._odom_xy_label.set_visible(True)
        self._odom_canvas.draw_idle()
        self._pwr_v_live_txt.set_visible(False)
        self._pwr_i_live_txt.set_visible(False)
        self._pwr_p_live_txt.set_visible(False)
        self._pwr_v_canvas.draw_idle()
        self._pwr_i_canvas.draw_idle()
        self._pwr_p_canvas.draw_idle()

        self._cam_im = None
        self._lidar_im = None

        # Reset time/frame labels
        self.pb_time_label.setText("0.0s / 0.0s")
        self.pb_frame_label.setText("F: -- / --")

    def _set_nav_btn_style(self, btn, style):
        """Update a navigation button's base style in all nav groups."""
        for g, group in enumerate(self._nav_groups):
            for r, (b, label, _old) in enumerate(group):
                if b is btn:
                    self._nav_groups[g][r] = (b, label, style)
                    break
        # Also update the master _nav_buttons list
        for i, (b, label, _old) in enumerate(self._nav_buttons):
            if b is btn:
                self._nav_buttons[i] = (b, label, style)
                break
        self._update_selection()

    def _live_tick(self):
        """10 Hz poll: consume latest ROS data and update GUI cells."""
        node = self._ros_node
        if node is None:
            return  # Placeholders stay visible

        t_s = time.monotonic() - self._live_t0
        any_scalar_changed = False

        # --- GPS ---
        gps = node.latest_gps
        if gps is not None:
            node.latest_gps = None
            lat, lon = gps
            self._gps_buf['lat'].append(lat)
            self._gps_buf['lon'].append(lon)
            # Trim to maxlen
            if len(self._gps_buf['lat']) > self._live_gps_maxlen:
                self._gps_buf['lat'] = self._gps_buf['lat'][-self._live_gps_maxlen:]
                self._gps_buf['lon'] = self._gps_buf['lon'][-self._live_gps_maxlen:]
            self._gps_live_txt.set_visible(False)
            self._live_set_dot_received('GPS')
            # Fetch map tiles on first point or if position leaves extent
            if self._gps_map_extent is None:
                img, extent = _fetch_map_for_gps([lat], [lon])
                if img is not None:
                    self._gps_map_img = img
                    self._gps_map_extent = extent
                    if self._gps_map_im is not None:
                        self._gps_map_im.remove()
                    self._gps_map_im = self._gps_ax.imshow(
                        self._gps_map_img,
                        extent=[extent[0], extent[1], extent[2], extent[3]],
                        aspect='auto', zorder=0,
                    )
            elif self._gps_map_extent is not None:
                e = self._gps_map_extent
                if not (e[2] <= lat <= e[3] and e[0] <= lon <= e[1]):
                    img, extent = _fetch_map_for_gps(
                        self._gps_buf['lat'], self._gps_buf['lon'],
                    )
                    if img is not None:
                        self._gps_map_img = img
                        self._gps_map_extent = extent
                        if self._gps_map_im is not None:
                            self._gps_map_im.remove()
                        self._gps_map_im = self._gps_ax.imshow(
                            self._gps_map_img,
                            extent=[extent[0], extent[1], extent[2], extent[3]],
                            aspect='auto', zorder=0,
                        )
            any_scalar_changed = True

        # --- Odom ---
        odom = node.latest_odom
        if odom is not None:
            node.latest_odom = None
            self._odom_live_txt.set_visible(False)
            self._odom_ax.grid(True, which='both', color='#333', linewidth=0.5)
            self._odom_ax.tick_params(labelbottom=True, labelleft=True)
            self._odom_xy_label.set_visible(True)
            x, y, qz = odom
            qz_clamped = max(-1.0, min(1.0, qz))
            yaw = 2.0 * math.asin(qz_clamped)
            self._odom_buf['x'].append(x)
            self._odom_buf['y'].append(y)
            self._odom_buf['theta'].append(yaw)
            if len(self._odom_buf['x']) > self._live_odom_maxlen:
                self._odom_buf['x'] = self._odom_buf['x'][-self._live_odom_maxlen:]
                self._odom_buf['y'] = self._odom_buf['y'][-self._live_odom_maxlen:]
                self._odom_buf['theta'] = self._odom_buf['theta'][-self._live_odom_maxlen:]
            self._live_set_dot_received('Encoders')
            any_scalar_changed = True

        # --- Power (Voltage / Current / Power) ---
        voltage = node.latest_voltage
        current = node.latest_current
        power = node.latest_power
        if voltage is not None or current is not None or power is not None:
            self._power_buf['t'].append(t_s)
            self._power_buf['V'].append(
                voltage if voltage is not None
                else (self._power_buf['V'][-1] if self._power_buf['V'] else 0)
            )
            self._power_buf['I'].append(
                current if current is not None
                else (self._power_buf['I'][-1] if self._power_buf['I'] else 0)
            )
            self._power_buf['P'].append(
                power if power is not None
                else (self._power_buf['P'][-1] if self._power_buf['P'] else 0)
            )
            node.latest_voltage = None
            node.latest_current = None
            node.latest_power = None
            self._trim_power_buf(t_s)
            self._pwr_v_live_txt.set_visible(False)
            self._pwr_i_live_txt.set_visible(False)
            self._pwr_p_live_txt.set_visible(False)
            self._live_set_dot_received('Power PCB')
            any_scalar_changed = True

        # --- Camera ---
        img_rgb = node.latest_image_rgb
        if img_rgb is not None:
            node.latest_image_rgb = None
            self._cam_live_txt.set_visible(False)
            if self._cam_im is None:
                self._cam_im = self._cam_ax.imshow(img_rgb, aspect='equal')
            else:
                self._cam_im.set_data(img_rgb)
            self._cam_canvas.draw_idle()
            self._live_set_dot_received('Camera')

        # --- LiDAR ---
        scan = node.latest_scan
        if scan is not None:
            node.latest_scan = None
            bev = self._render_lidar_bev(scan)
            self._lidar_live_txt.set_visible(False)
            if self._lidar_im is None:
                self._lidar_im = self._lidar_ax.imshow(bev, aspect='equal')
            else:
                self._lidar_im.set_data(bev)
            self._lidar_canvas.draw_idle()
            self._live_set_dot_received('Lidar')

        # --- Redraw scalar plots ---
        if any_scalar_changed:
            self._redraw_plots()

    @staticmethod
    def _render_lidar_bev(scan, size=480):
        """Render a LaserScan as a bird's-eye-view RGB image.

        Draws shadow lines (gray), hit dots (green), and robot origin (red)
        on a black canvas. Returns an RGB numpy array.
        """
        img = np.zeros((size, size, 3), dtype=np.uint8)
        cx, cy = size // 2, size // 2

        # Determine scale: fit max range into half the canvas
        max_range = scan.range_max
        if max_range <= 0 or not np.isfinite(max_range):
            max_range = 10.0
        scale = (size // 2 - 2) / max_range

        angles = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
        ranges = np.array(scan.ranges, dtype=np.float32)

        for i in range(len(ranges)):
            r = ranges[i]
            a = angles[i]
            if not np.isfinite(r) or r < scan.range_min:
                continue
            # End point
            ex = int(cx + r * math.cos(a) * scale)
            ey = int(cy - r * math.sin(a) * scale)
            # Shadow line (gray)
            _bresenham_line(img, cx, cy, ex, ey, (40, 40, 40))
            # Hit dot (green) if within valid range
            if r <= scan.range_max:
                if 0 <= ex < size and 0 <= ey < size:
                    img[ey, ex] = (0, 255, 0)

        # Robot origin (red dot)
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < size and 0 <= nx < size:
                    img[ny, nx] = (255, 0, 0)

        return img


def _bresenham_line(img, x0, y0, x1, y1, color):
    """Draw a line on a numpy image using Bresenham's algorithm."""
    h, w = img.shape[:2]
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if 0 <= y0 < h and 0 <= x0 < w:
            img[y0, x0] = color
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


if _HAS_ROS:
    class HudNode(Node):
        """ROS2 node for the AutoNav HUD with live sensor subscriptions."""

        def __init__(self):
            super().__init__('autonav_hud')
            self.get_logger().info('AutoNav HUD node started')

            # Latest values (consumed by _live_tick on the GUI thread)
            self.latest_image_rgb = None   # numpy RGB array
            self.latest_scan = None        # LaserScan message
            self.latest_gps = None         # (lat, lon)
            self.latest_odom = None        # (x, y, qz)
            self.latest_voltage = None     # float
            self.latest_current = None     # float
            self.latest_power = None       # float

            self._cv_bridge = CvBridge() if _HAS_CV_BRIDGE else None

            self.create_subscription(
                Image,
                '/zed2i/zed_node/rgb/image_rect_color',
                self._cb_image, 1,
            )
            self.create_subscription(
                LaserScan, '/scan', self._cb_scan, 1,
            )
            self.create_subscription(
                NavSatFix, '/gps_fix', self._cb_gps, 1,
            )
            self.create_subscription(
                Odometry, '/odom', self._cb_odom, 1,
            )
            self.create_subscription(
                Float32, '/electrical/voltage', self._cb_voltage, 1,
            )
            self.create_subscription(
                Float32, '/electrical/current', self._cb_current, 1,
            )
            self.create_subscription(
                Float32, '/electrical/power', self._cb_power, 1,
            )

        def _cb_image(self, msg):
            if self._cv_bridge is not None:
                try:
                    self.latest_image_rgb = self._cv_bridge.imgmsg_to_cv2(
                        msg, desired_encoding='rgb8',
                    )
                except Exception:
                    pass

        def _cb_scan(self, msg):
            self.latest_scan = msg

        def _cb_gps(self, msg):
            self.latest_gps = (msg.latitude, msg.longitude)

        def _cb_odom(self, msg):
            p = msg.pose.pose.position
            qz = msg.pose.pose.orientation.z
            self.latest_odom = (p.x, p.y, qz)

        def _cb_voltage(self, msg):
            self.latest_voltage = msg.data

        def _cb_current(self, msg):
            self.latest_current = msg.data

        def _cb_power(self, msg):
            self.latest_power = msg.data


def main(args=None):
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    node = None
    if _HAS_ROS:
        rclpy.init(args=args)
        node = HudNode()

    app = QApplication(sys.argv)
    window = HudWindow(ros_node=node)
    window.show()

    if node:
        timer = QTimer()
        timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
        timer.start(50)

    exit_code = app.exec_()

    if node:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
