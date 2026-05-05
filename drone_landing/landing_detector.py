"""
降落目标检测 ROS2 节点
Landing Target Detector Node

检测 ArUco 标记, 估计 6DOF 位姿, 发布降落偏差

订阅: 摄像头图像
发布: /landing/target_pose (降落目标位姿)
      /landing/detection_image (标注图像)
"""
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from drone_landing.aruco_pose import ArUcoDetector, PnPPoseEstimator


class LandingDetectorNode(Node):
    def __init__(self):
        super().__init__("landing_detector")

        # 参数
        self.declare_parameter("video_source", 0)
        self.declare_parameter("aruco_dict", "4x4_50")
        self.declare_parameter("marker_size", 0.2)  # meters
        self.declare_parameter("fx", 500.0)
        self.declare_parameter("fy", 500.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("publish_annotated", True)

        src = self.get_parameter("video_source").value
        dict_name = self.get_parameter("aruco_dict").value
        marker_size = self.get_parameter("marker_size").value
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
        self.publish_annotated = self.get_parameter("publish_annotated").value

        # ArUco 检测器 + PnP 估计器
        self.detector = ArUcoDetector(dict_name, marker_size)
        K = PnPPoseEstimator.create_default_camera_matrix(fx, fy, cx, cy)
        dist = np.zeros(5, dtype=np.float64)
        self.pose_estimator = PnPPoseEstimator(K, dist, marker_size)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_detection = None
        self.img_w = int(cx * 2)
        self.img_h = int(cy * 2)

        # 视频源
        if isinstance(src, str) and src.isdigit():
            src = int(src)
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video: {src}")

        # 发布
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pose_pub = self.create_publisher(String, "/landing/target_pose", 10)
        self.img_pub = self.create_publisher(Image, "/landing/detection_image", qos)

        # 30fps
        self.timer = self.create_timer(1.0/30.0, self._process)

        self.get_logger().info(
            f"LandingDetector started (dict={dict_name}, size={marker_size}m)")

    def _process(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        h, w = frame.shape[:2]
        self.img_w, self.img_h = w, h

        # 检测 ArUco
        detections = self.detector.detect(frame)

        result = {
            "frame": self.frame_count,
            "timestamp": time.time(),
            "detected": False,
            "markers": [],
        }

        if detections:
            # 取最大/最近的标记
            best = max(detections, key=lambda d: d["area"])
            self.last_detection = best

            # PnP 位姿估计
            pose = self.pose_estimator.estimate(best["corners"])

            if pose:
                # 像素偏差 (相对于图像中心)
                pixel_err_x = best["center"][0] - w / 2.0
                pixel_err_y = best["center"][1] - h / 2.0

                result["detected"] = True
                result["markers"] = [{
                    "id": best["id"],
                    "center": best["center"],
                    "area": best["area"],
                    "pose": pose,
                    "pixel_error": [pixel_err_x, pixel_err_y],
                }]

        msg = String()
        msg.data = json.dumps(result)
        self.pose_pub.publish(msg)

        # 标注图像
        if self.publish_annotated:
            annotated = self._annotate(frame, detections, result)
            img_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            self.img_pub.publish(img_msg)

    def _annotate(self, frame, detections, result):
        annotated = frame.copy()
        h, w = frame.shape[:2]

        # 画中心十字线
        cv2.drawMarker(annotated, (w//2, h//2), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)

        if detections:
            best = max(detections, key=lambda d: d["area"])
            corners = best["corners"]
            cx, cy = int(best["center"][0]), int(best["center"][1])

            # 画标记边框
            pts = corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(annotated, [pts], True, (0, 255, 0), 2)

            # 画中心点
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)

            # 画偏差线 (从图像中心到标记中心)
            cv2.line(annotated, (w//2, h//2), (cx, cy), (0, 0, 255), 2)

            # 显示位姿信息
            if result["markers"]:
                pose = result["markers"][0].get("pose", {})
                pe = result["markers"][0].get("pixel_error", [0, 0])
                info_lines = [
                    f"ID: {best['id']}",
                    f"PixErr: ({pe[0]:.0f}, {pe[1]:.0f})",
                    f"Dist: {pose.get('distance', 0):.2f}m",
                    f"TX: {pose.get('tx', 0):.3f}m",
                    f"TY: {pose.get('ty', 0):.3f}m",
                    f"TZ: {pose.get('tz', 0):.3f}m",
                ]
                for i, line in enumerate(info_lines):
                    cv2.putText(annotated, line, (10, 30 + i*25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(annotated, "NO MARKER", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        return annotated


def main(args=None):
    rclpy.init(args=args)
    node = LandingDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
