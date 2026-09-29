"""Stub publisher for the CV-pipeline RViz suite.

TESTING / DEMO ONLY. Publishes placeholder data on the perception topic
contract (see README.md) so every panel in rviz/cv_pipeline.rviz can be
checked before the real detectors exist. Each output has its own enable
parameter: switch a topic off here as soon as a real publisher owns it so
the two never fight over the same topic.

All geometry goes through one pinhole camera model built from the
zed_camera_link mount in bringup/description/shogi.urdf, so the synthetic
frame, the projected 2D boxes and the BEV warp agree with each other. When
/zed/.../camera_info is available its intrinsics replace the HFOV fallback.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# Class → (marker type, BGR for images, RGBA for markers). This is the
# convention real detectors should follow on /perception/objects/markers.
CLASS_STYLE = {
    'barrel':        (Marker.CYLINDER, (0, 128, 255),   (1.0, 0.5, 0.0, 0.9)),
    'pothole':       (Marker.CYLINDER, (255, 255, 255), (1.0, 1.0, 1.0, 0.9)),
    'pedestrian':    (Marker.CUBE,     (255, 102, 51),  (0.2, 0.4, 1.0, 0.9)),
    'stop_sign':     (Marker.CUBE,     (0, 0, 255),     (1.0, 0.0, 0.0, 0.9)),
    'road_sign':     (Marker.CUBE,     (0, 255, 255),   (1.0, 1.0, 0.0, 0.9)),
    'traffic_light': (Marker.CUBE,     (128, 128, 128), (0.5, 0.5, 0.5, 0.9)),
}

LIGHT_STATES = [
    ('RED',    (0, 0, 255),   (1.0, 0.0, 0.0, 1.0)),
    ('GREEN',  (0, 255, 0),   (0.0, 1.0, 0.0, 1.0)),
    ('YELLOW', (0, 215, 255), (1.0, 0.85, 0.0, 1.0)),
]

YELLOW_BGR = (0, 215, 255)


class CameraModel:
    """Pinhole camera fixed to base_link, looking at a flat ground plane."""

    # camera link (x fwd, y left, z up) → optical (x right, y down, z fwd)
    LINK_TO_OPTICAL = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)

    def __init__(self, position, pitch, ground_z):
        self.c = np.asarray(position, dtype=float)
        cp, sp = math.cos(pitch), math.sin(pitch)
        # +pitch about y tilts the camera's x axis toward -z (nose down).
        rot = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        self.base_to_optical = self.LINK_TO_OPTICAL @ rot.T
        self.ground_z = ground_z
        self.K = None
        self.size = None

    def set_intrinsics(self, K, width, height):
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        self.size = (int(width), int(height))

    def set_hfov(self, hfov, width, height):
        f = (width / 2.0) / math.tan(hfov / 2.0)
        self.set_intrinsics([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]],
                            width, height)

    def scaled_K(self, width, height):
        """Intrinsics rescaled to an image of a different resolution."""
        sx, sy = width / self.size[0], height / self.size[1]
        return np.diag([sx, sy, 1.0]) @ self.K

    def project(self, pts_base, width, height):
        """base_link points (N,3) → pixels (N,2); NaN where behind camera."""
        cam = (self.base_to_optical @ (np.asarray(pts_base) - self.c).T).T
        uvw = (self.scaled_K(width, height) @ cam.T).T
        out = uvw[:, :2] / uvw[:, 2:3]
        out[cam[:, 2] <= 0.05] = np.nan
        return out

    def ground_homography(self, width, height):
        """3x3 H mapping ground (x, y, 1) in base_link → image pixels."""
        G = np.array([[1, 0, -self.c[0]],
                      [0, 1, -self.c[1]],
                      [0, 0, self.ground_z - self.c[2]]])
        return self.scaled_K(width, height) @ self.base_to_optical @ G


class GroundGrid:
    """Top-down raster of the ground: row 0 = far, col 0 = left."""

    def __init__(self, x_min, x_max, y_half, res):
        self.x_min, self.x_max, self.y_half, self.res = x_min, x_max, y_half, res
        self.width = int(round(2 * y_half / res))
        self.height = int(round((x_max - x_min) / res))
        # grid pixel (col, row, 1) → ground (x, y, 1)
        self.A = np.array([[0, -res, x_max], [-res, 0, y_half], [0, 0, 1]])

    def px(self, x, y):
        return (int(round((self.y_half - y) / self.res)),
                int(round((self.x_max - x) / self.res)))


class PerceptionVizStub(Node):

    def __init__(self):
        super().__init__('perception_viz_stub')
        p = self.declare_parameter
        self.rate_hz = p('rate_hz', 5.0).value
        classes = p('classes', 'all').value
        self.classes = (set(CLASS_STYLE) if classes.strip() in ('', 'all')
                        else {c.strip() for c in classes.split(',')})
        self.watermark = p('watermark', True).value
        self.base_frame = p('base_frame', 'base_link').value
        self.camera_frame = p('camera_frame', 'zed_left_camera_optical_frame').value
        self.light_period_s = p('light_period_s', 3.0).value

        self.en_yellow = p('publish_yellow', True).value
        self.en_mask_color = p('publish_mask_color', True).value
        self.en_objects = p('publish_objects', True).value
        self.en_signals = p('publish_signals', True).value
        self.en_det_image = p('publish_detection_image', True).value
        self.en_bev = p('publish_bev', True).value

        raw_topic = p('raw_image_topic', '/line_detection/debug/raw').value
        white_topic = p('white_mask_topic', '/line_detection/debug/mask').value
        info_topic = p('camera_info_topic',
                       '/zed/zed_node/rgb/color/rect/camera_info').value

        # Defaults: zed_camera_link joint in shogi.urdf; ground = base_footprint.
        self.cam = CameraModel(
            [p('cam_x', 0.6576).value, p('cam_y', 0.009).value, p('cam_z', 0.3072).value],
            p('cam_pitch', 0.349).value,
            p('ground_z', -0.113).value)
        self.fallback_hfov = p('fallback_hfov', 1.4538).value
        self.fallback_size = (p('fallback_width', 427).value,
                              p('fallback_height', 240).value)
        self.cam.set_hfov(self.fallback_hfov, *self.fallback_size)
        self.have_info = False

        self.bev = GroundGrid(p('bev_x_min', 0.7).value, p('bev_x_max', 6.0).value,
                              p('bev_y_half', 3.0).value, p('bev_resolution', 0.02).value)
        self.lane_half_width = p('lane_half_width', 1.5).value

        self.bridge = CvBridge()
        self.raw = None
        self.white_mask = None
        self.ext_yellow = None
        self.synthetic_cache = {}
        self.t0 = self.get_clock().now()

        self.create_subscription(Image, raw_topic, self._on_raw, qos_profile_sensor_data)
        self.create_subscription(Image, white_topic, self._on_white, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        if self.en_mask_color and not self.en_yellow:
            # A real yellow detector owns the topic; composite from it instead.
            self.create_subscription(Image, '/perception/lanes/yellow_mask',
                                     self._on_ext_yellow, qos_profile_sensor_data)

        self.pub_yellow = self.create_publisher(Image, '/perception/lanes/yellow_mask', 5)
        self.pub_mask_color = self.create_publisher(Image, '/perception/lanes/mask_color', 5)
        self.pub_det_image = self.create_publisher(Image, '/perception/detections/image', 5)
        self.pub_bev = self.create_publisher(Image, '/perception/bev/image', 5)
        self.pub_objects = self.create_publisher(MarkerArray, '/perception/objects/markers', 5)
        self.pub_signals = self.create_publisher(MarkerArray, '/perception/signals/markers', 5)

        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().warn(
            'perception_viz_stub is publishing PLACEHOLDER perception data '
            f'(classes={sorted(self.classes)}). Disable per-topic publish_* '
            'params once real detectors own a topic.')

    # ── subscriptions ────────────────────────────────────────────────────
    def _on_raw(self, msg):
        self.raw = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_white(self, msg):
        self.white_mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')

    def _on_ext_yellow(self, msg):
        self.ext_yellow = self.bridge.imgmsg_to_cv2(msg, 'mono8')

    def _on_info(self, msg):
        if msg.width == 0 or msg.k[0] == 0.0:
            return
        if not self.have_info:
            self.get_logger().info(
                f'Using camera_info intrinsics ({msg.width}x{msg.height}).')
        self.cam.set_intrinsics(msg.k, msg.width, msg.height)
        self.have_info = True
        self.synthetic_cache.clear()

    # ── scenario ─────────────────────────────────────────────────────────
    def _light_state(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        return LIGHT_STATES[int(t // self.light_period_s) % len(LIGHT_STATES)]

    def _scenario(self):
        """(class, center xyz in base_link, size xyz, confidence)."""
        # Spread along the lane so each one lands inside the ZED's view; the
        # camera is pitched 20° down, so tall objects must sit far away.
        g = self.cam.ground_z
        objs = [
            ('barrel',        (3.0,  0.9, g + 0.45), (0.58, 0.58, 0.9), 0.91),
            ('pothole',       (4.5, -0.6, g + 0.01), (0.6, 0.6, 0.02), 0.84),
            ('pedestrian',    (9.0, -1.0, g + 0.85), (0.45, 0.45, 1.7), 0.88),
            ('stop_sign',     (10.0, -2.2, g + 1.2), (0.05, 0.75, 0.75), 0.88),
            ('traffic_light', (13.0, 2.2, g + 1.4),  (0.3, 0.3, 0.8), 0.93),
        ]
        return [o for o in objs if o[0] in self.classes]

    # ── images ───────────────────────────────────────────────────────────
    def _frame_size(self):
        if self.raw is not None:
            return self.raw.shape[1], self.raw.shape[0]
        return self.cam.size if self.have_info else self.fallback_size

    def _synthetic(self, width, height):
        """Synthetic camera frame + white/yellow lane masks, rendered by
        warping a top-down road through the ground homography."""
        key = (width, height)
        if key in self.synthetic_cache:
            return self.synthetic_cache[key]
        # Canvas starts ahead of the camera so no ray samples ground behind it.
        near, far = self.cam.c[0] + 0.05, 25.0
        road = GroundGrid(near, far, 6.0, 0.02)
        top = np.full((road.height, road.width, 3), (60, 110, 60), np.uint8)   # grass
        white = np.zeros((road.height, road.width), np.uint8)
        yellow = np.zeros_like(white)
        w = self.lane_half_width
        cv2.rectangle(top, road.px(far, w + 0.3), road.px(near, -w - 0.3), (90, 90, 90), -1)
        line_px = max(1, int(0.08 / road.res))
        for y in (w, -w):
            cv2.line(white, road.px(far, y), road.px(near, y), 255, line_px)
        for x in np.arange(near, far, 1.5):   # dashed center line
            cv2.line(yellow, road.px(x + 0.8, 0.0), road.px(x, 0.0), 255, line_px)
        top[white > 0] = (255, 255, 255)
        top[yellow > 0] = YELLOW_BGR

        H = self.cam.ground_homography(width, height) @ road.A
        warp = lambda im, border: cv2.warpPerspective(
            im, H, (width, height), flags=cv2.INTER_NEAREST, borderValue=border)
        frame = warp(top, (200, 170, 120))   # sky
        out = (frame, warp(white, 0), warp(yellow, 0))
        self.synthetic_cache[key] = out
        return out

    def _stamp(self, img, text='STUB'):
        if self.watermark:
            cv2.putText(img, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 255), 2, cv2.LINE_AA)
        return img

    def _publish_image(self, pub, img, encoding, frame_id, stamp):
        msg = self.bridge.cv2_to_imgmsg(img, encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        pub.publish(msg)

    def _lane_images(self, width, height, stamp):
        _, syn_white, syn_yellow = self._synthetic(width, height)
        if self.en_yellow:
            self._publish_image(self.pub_yellow, syn_yellow, 'mono8',
                                self.camera_frame, stamp)
        if not self.en_mask_color:
            return
        yellow = syn_yellow if self.en_yellow else self.ext_yellow
        white = self.white_mask if self.white_mask is not None else syn_white
        white = cv2.resize(white, (width, height), interpolation=cv2.INTER_NEAREST)
        color = np.zeros((height, width, 3), np.uint8)
        color[white > 0] = (255, 255, 255)
        if yellow is not None:
            yellow = cv2.resize(yellow, (width, height), interpolation=cv2.INTER_NEAREST)
            color[yellow > 0] = YELLOW_BGR
        src = 'white=line_detector' if self.white_mask is not None else 'white=STUB'
        cv2.putText(color, src, (6, height - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (200, 200, 200), 1, cv2.LINE_AA)
        self._publish_image(self.pub_mask_color, self._stamp(color), 'bgr8',
                            self.camera_frame, stamp)

    def _detection_image(self, base, stamp, light):
        img = base.copy()
        h, w = img.shape[:2]
        for cls, center, size, conf in self._scenario():
            corners = np.array([[center[0] + sx * size[0] / 2,
                                 center[1] + sy * size[1] / 2,
                                 center[2] + sz * size[2] / 2]
                                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            uv = self.cam.project(corners, w, h)
            if np.isnan(uv).any():
                continue
            uv = np.clip(uv, -10000, 10000)
            u0, v0 = (int(v) for v in uv.min(axis=0))
            u1, v1 = (int(v) for v in uv.max(axis=0))
            if u1 < 0 or v1 < 0 or u0 >= w or v0 >= h:
                continue
            color = CLASS_STYLE[cls][1]
            label = f'{cls} {conf:.2f}'
            if cls == 'traffic_light':
                color, label = light[1], f'light:{light[0]} {conf:.2f}'
            cv2.rectangle(img, (u0, v0), (u1, v1), color, 2)
            # Label above the box, or just inside it when the box is clipped.
            ty = v0 - 4 if v0 >= 16 else max(v0, 0) + 14
            cv2.putText(img, label, (max(u0, 0) + 2, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, color, 1, cv2.LINE_AA)
        self._publish_image(self.pub_det_image, self._stamp(img), 'bgr8',
                            self.camera_frame, stamp)

    def _bev_image(self, base, stamp):
        h, w = base.shape[:2]
        H = self.cam.ground_homography(w, h) @ self.bev.A
        bev = cv2.warpPerspective(base, H, (self.bev.width, self.bev.height),
                                  flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        # 1 m grid in base_link coordinates.
        for x in range(math.ceil(self.bev.x_min), int(self.bev.x_max) + 1):
            a, b = self.bev.px(x, self.bev.y_half), self.bev.px(x, -self.bev.y_half)
            cv2.line(bev, a, b, (0, 200, 255), 1)
            cv2.putText(bev, f'{x}m', (a[0] + 2, a[1] - 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, (0, 200, 255), 1, cv2.LINE_AA)
        for y in range(-int(self.bev.y_half), int(self.bev.y_half) + 1):
            cv2.line(bev, self.bev.px(self.bev.x_max, y), self.bev.px(self.bev.x_min, y),
                     (0, 200, 255), 1)
        cv2.putText(bev, 'camera preview, not planner input', (6, bev.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1, cv2.LINE_AA)
        self._publish_image(self.pub_bev, self._stamp(bev), 'bgr8', self.base_frame, stamp)

    # ── markers ──────────────────────────────────────────────────────────
    def _marker(self, ns, mid, mtype, stamp, pos, scale, rgba):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = (float(s) for s in scale)
        m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        # Outlive one publish period so markers don't flicker, then expire.
        life = 0.25 + 1.0 / self.rate_hz
        m.lifetime.sec = int(life)
        m.lifetime.nanosec = int((life % 1.0) * 1e9)
        return m

    def _text(self, ns, mid, stamp, pos, text, rgba, height=0.25):
        m = self._marker(ns, mid, Marker.TEXT_VIEW_FACING, stamp, pos,
                         (0, 0, height), rgba)
        m.text = text
        return m

    def _object_markers(self, stamp, light):
        arr = MarkerArray()
        for i, (cls, center, size, conf) in enumerate(self._scenario()):
            mtype, _, rgba = CLASS_STYLE[cls]
            arr.markers.append(self._marker(cls, i, mtype, stamp, center, size, rgba))
            label = f'{cls} {conf:.2f}'
            if cls == 'traffic_light':
                label = f'traffic_light [{light[0]}] {conf:.2f}'
            top = (center[0] + size[0] / 2 + 0.35, center[1], center[2] + size[2] / 2 + 0.25)
            arr.markers.append(self._text(cls + '_label', i, stamp, top, label,
                                          (1.0, 1.0, 1.0, 1.0)))
        self.pub_objects.publish(arr)

    def _signal_markers(self, stamp, light):
        arr = MarkerArray()
        g = self.cam.ground_z
        if 'traffic_light' in self.classes:
            arr.markers.append(self._text('signal_readout', 0, stamp, (-1.0, 0.0, g + 1.0),
                                          f'TRAFFIC LIGHT: {light[0]} (0.93)', light[2], 0.3))
            arr.markers.append(self._marker('light_state', 0, Marker.SPHERE, stamp,
                                            (-1.0, 1.9, g + 1.0), (0.3, 0.3, 0.3), light[2]))
            for cls, center, size, _ in self._scenario():
                if cls == 'traffic_light':   # lamp on the light's front face
                    lamp = (center[0] - size[0] / 2 - 0.05, center[1], center[2])
                    arr.markers.append(self._marker('light_state', 1, Marker.SPHERE, stamp,
                                                    lamp, (0.2, 0.2, 0.2), light[2]))
        if 'stop_sign' in self.classes:
            arr.markers.append(self._text('signal_readout', 1, stamp, (-1.6, 0.0, g + 1.0),
                                          'SIGN: STOP (0.88)', (1.0, 0.2, 0.2, 1.0), 0.3))
        self.pub_signals.publish(arr)

    # ── main loop ────────────────────────────────────────────────────────
    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        light = self._light_state()
        width, height = self._frame_size()
        base = self.raw if self.raw is not None else self._synthetic(width, height)[0]

        if self.en_yellow or self.en_mask_color:
            self._lane_images(width, height, stamp)
        if self.en_det_image:
            self._detection_image(base, stamp, light)
        if self.en_bev:
            self._bev_image(base, stamp)
        if self.en_objects:
            self._object_markers(stamp, light)
        if self.en_signals:
            self._signal_markers(stamp, light)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionVizStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
