# uav_camera_websend3.py
# ROS2 + YOLO + WebSocket + PX4
# 버티포트(헬리패드) 감지 → 위치추정 → /helipad/position Publish + 터미널 출력 + HOLD 전환

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import asyncio
import websockets
import base64
import json
import threading
import time
import math
import numpy as np

# PX4 uORB (via px4_msgs)
from px4_msgs.msg import VehicleCommand, VehicleOdometry

# =========================
# PX4 명령/모드 정의
# =========================
MAV_CMD_DO_SET_MODE              = 176
PX4_CUSTOM_MAIN_MODE_AUTO        = 4.0
PX4_CUSTOM_SUBMODE_AUTO_HOLD     = 2.0
PX4_CUSTOM_SUBMODE_AUTO_LOITER   = 3.0  # PX4에서 HOLD가 LOITER로 표시될 수 있음

# =========================
# YOLO 모델 (버티포트 학습 가중치 경로)
# =========================
MODEL_PATH = '/home/hj926217/yolo11-python-tutorial/examples/dataset/01train/runs/detect/train3/weights/best.pt'
model = YOLO(MODEL_PATH)

# =========================
# WebSocket 공유 데이터
# =========================
latest_frame_data = {"image": None, "objects": []}

# =========================
# 감지/제어 파라미터
# =========================
CONF_THRESHOLD = 0.5
CMD_DEBOUNCE_SEC = 1.0
HOLD_TRIGGER_SEC = 0.01
# 사용 중인 버티포트 클래스 ID(학습 결과에 맞게 수정)
VERTIPORT_CLASS_ID = 0
CUSTOM_TARGET_CLASS_IDS = {VERTIPORT_CLASS_ID}

# =========================
# 카메라 FOV 기본값 (라디안)
# =========================
DEFAULT_HFOV = math.radians(80.0)  # 수평 FOV
DEFAULT_VFOV = math.radians(60.0)  # 수직 FOV


# =========================
# WebSocket 서버
# =========================
async def websocket_handler(websocket):
    while True:
        try:
            if latest_frame_data["image"] is not None:
                safe_objects = []
                for obj in latest_frame_data["objects"]:
                    safe_objects.append({
                        "label": obj["label"],
                        "confidence": float(obj["confidence"]),
                        "bbox": list(map(int, obj["bbox"]))
                    })
                await websocket.send(json.dumps({
                    "frame": latest_frame_data["image"],
                    "objects": safe_objects
                }))
            await asyncio.sleep(0.1)
        except Exception as e:
            print("❌ WebSocket 전송 중 오류:", e)
            break

def start_websocket_server():
    async def run_server():
        async with websockets.serve(websocket_handler, "0.0.0.0", 8765):
            print("✅ WebSocket 서버가 8765 포트에서 실행 중입니다.")
            await asyncio.Future()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_server())


# =========================
# 보조 함수
# =========================
def quat_to_yaw(q):
    """PX4 VehicleOdometry.q (w,x,y,z) -> yaw[rad]"""
    w, x, y, z = q
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)


# =========================
# 메인 노드
# =========================
class HelipadLocator(Node):
    def __init__(self):
        super().__init__('helipad_locator')

        # ---- 파라미터 선언 ----
        self.declare_parameter('hfov', DEFAULT_HFOV)
        self.declare_parameter('vfov', DEFAULT_VFOV)
        self.declare_parameter('image_topic', '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image')
        self.declare_parameter('odometry_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('publish_topic', '/helipad/position')

        # ---- 파라미터 안전 접근(버전 호환) ----
        self.HFOV = self._get_float_param('hfov', DEFAULT_HFOV)
        self.VFOV = self._get_float_param('vfov', DEFAULT_VFOV)
        self.image_topic = self._get_str_param('image_topic', '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image')
        self.odom_topic  = self._get_str_param('odometry_topic', '/fmu/out/vehicle_odometry')
        self.pub_topic   = self._get_str_param('publish_topic', '/helipad/position')

        # ---- Publishers/Subscriptions ----
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.pos_pub = self.create_publisher(PointStamped, self.pub_topic, 10)
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.odom_sub = self.create_subscription(VehicleOdometry, self.odom_topic, self.odom_cb, 10)

        self.br = CvBridge()

        # ---- 내부 상태 ----
        self.last_cmd_ts = 0.0
        self.detect_start_ts = None
        self.vehicle_pos_xy = np.array([0.0, 0.0])   # 로컬 NED (x=north, y=east)
        self.vehicle_alt = 10.0                      # Alt = -z(NED)
        self.vehicle_yaw = 0.0                       # rad

        self.get_logger().info("HelipadLocator ready: detect → estimate → publish.")

    # --------- 파라미터 버전 호환 접근 ---------
    def _get_float_param(self, name, default):
        p = self.get_parameter(name)
        if hasattr(p, 'value'):
            v = p.value
        elif hasattr(p, 'get_parameter_value'):
            v = p.get_parameter_value().double_value
        else:
            v = None
        return float(v if v is not None else default)

    def _get_str_param(self, name, default):
        p = self.get_parameter(name)
        if hasattr(p, 'value'):
            v = p.value
        elif hasattr(p, 'get_parameter_value'):
            v = p.get_parameter_value().string_value
        else:
            v = None
        return str(v if v else default)

    # --------- PX4 VehicleCommand 송신 ---------
    def send_vehicle_command(self, cmd, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0, from_external=True):
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        m = VehicleCommand()
        m.timestamp = now_us
        m.command = cmd
        m.param1 = float(p1); m.param2 = float(p2); m.param3 = float(p3); m.param4 = float(p4)
        m.param5 = float(p5); m.param6 = float(p6); m.param7 = float(p7)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = from_external
        self.cmd_pub.publish(m)

    def switch_to_hold(self):
        now = time.time()
        if now - self.last_cmd_ts < CMD_DEBOUNCE_SEC:
            return
        self.send_vehicle_command(MAV_CMD_DO_SET_MODE, 1.0, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUBMODE_AUTO_LOITER)
        self.last_cmd_ts = now
        self.get_logger().warn("HOLD triggered (AUTO.LOITER)")

    # --------- 오도메트리 콜백 ---------
    def odom_cb(self, msg: VehicleOdometry):
        # PX4 local NED: x=north(+), y=east(+), z=down(+)
        self.vehicle_pos_xy = np.array([float(msg.position[0]), float(msg.position[1])])
        self.vehicle_alt = max(0.0, -float(msg.position[2]))  # Alt = -z
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        self.vehicle_yaw = quat_to_yaw(q)

    # --------- 이미지 콜백 ---------
    def image_cb(self, data: Image):
        frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        h_img, w_img = frame.shape[:2]

        # YOLO: 버티포트 클래스만
        results = model.predict(frame, imgsz=640, conf=CONF_THRESHOLD, classes=list(CUSTOM_TARGET_CLASS_IDS))
        r0 = results[0]
        plotted = r0.plot()

        # 최고 신뢰도 버티포트 선택
        target = None
        for b in r0.boxes:
            cls_id = int(b.cls[0])
            conf = float(b.conf[0])
            if cls_id in CUSTOM_TARGET_CLASS_IDS and conf >= CONF_THRESHOLD:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                if (target is None) or (conf > target["confidence"]):
                    target = {
                        "label": model.names[cls_id] if hasattr(model, "names") else f"class_{cls_id}",
                        "confidence": conf,
                        "bbox": [x1, y1, x2 - x1, y2 - y1],
                        "center": (cx, cy)
                    }

        # WebSocket 버퍼
        _, jpeg = cv2.imencode('.jpg', plotted)
        latest_frame_data["image"] = base64.b64encode(jpeg.tobytes()).decode('utf-8')
        latest_frame_data["objects"] = ([] if target is None else [{
            "label": target["label"],
            "confidence": round(target["confidence"], 3),
            "bbox": list(map(int, target["bbox"]))
        }])

        # 감지 유지 → HOLD
        now = time.time()
        if target is not None:
            if self.detect_start_ts is None:
                self.detect_start_ts = now
            elif now - self.detect_start_ts >= HOLD_TRIGGER_SEC:
                self.switch_to_hold()
        else:
            self.detect_start_ts = None

        # ===== 위치 추정 & Publish =====
        if target is not None and self.vehicle_alt > 0.01:
            cx, cy = target["center"]

            # (1) 투영 면적(세계 좌표) 계산
            width_world  = 2.0 * math.tan(self.HFOV * 0.5) * self.vehicle_alt
            height_world = 2.0 * math.tan(self.VFOV * 0.5) * self.vehicle_alt

            # (2) 픽셀→미터 스케일
            pixelX_length = width_world  / float(w_img)
            pixelY_length = height_world / float(h_img)

            # (3) 이미지 중심 대비 픽셀 오차
            px_err = cx - (w_img / 2.0)
            py_err = cy - (h_img / 2.0)

            # (4) 카메라 기준 평면 좌표
            helipad_x_cam = px_err * pixelX_length
            helipad_y_cam = -py_err * pixelY_length  # 영상 y아래가 + → 부호 반전

            # (5) yaw 회전(Rz) → 로컬맵(NED:x=north,y=east)
            yaw = self.vehicle_yaw
            c, s = math.cos(yaw), math.sin(yaw)
            R = np.array([[c, -s],
                          [s,  c]])
            vec_body = np.array([helipad_x_cam, helipad_y_cam])
            vec_world = R @ vec_body

            # (6) 드론 위치 더해 월드 좌표
            helipad_world_xy = self.vehicle_pos_xy + vec_world

            # Publish
            msg = PointStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "local_world"
            msg.point.x = float(helipad_world_xy[0])
            msg.point.y = float(helipad_world_xy[1])
            msg.point.z = 0.0
            self.pos_pub.publish(msg)

            # 터미널 출력
            self.get_logger().info(
                f"[HELIPAD] conf={target['confidence']:.2f} | "
                f"img({w_img}x{h_img}) Alt={self.vehicle_alt:.2f}m | "
                f"px_err=({px_err:.1f},{py_err:.1f}) | "
                f"cam_xy=({helipad_x_cam:.2f},{helipad_y_cam:.2f})m | "
                f"yaw={math.degrees(yaw):.1f}deg | "
                f"world_xy=({helipad_world_xy[0]:.2f},{helipad_world_xy[1]:.2f})m"
            )

        # 로컬 시각화
        cv2.namedWindow('Detected Frame', flags=cv2.WINDOW_NORMAL)
        cv2.imshow('Detected Frame', plotted)
        cv2.waitKey(1)


# =========================
# main
# =========================
def main(args=None):
    # WebSocket 서버 백그라운드 실행
    t = threading.Thread(target=start_websocket_server, daemon=True)
    t.start()

    # ROS2 실행
    rclpy.init(args=args)
    node = HelipadLocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
