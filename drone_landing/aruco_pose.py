"""
ArUco 标记检测与 6DOF 位姿估计
ArUco Marker Detection + PnP Pose Estimation

核心原理:
1. 检测图像中的 ArUco 标记 (4个角点)
2. 用 PnP (Perspective-n-Point) 算法求解相机到标记的位姿
3. 输出: 平移向量 t = [tx, ty, tz] + 旋转向量 r = [rx, ry, rz]

PnP 问题:
  已知: 标记的 3D 坐标 (世界系) + 图像中的 2D 坐标 (像素)
  求解: 相机在世界系中的位姿 (R, t)

在精准降落中的应用:
  - 标记固定在降落点, 相机安装在无人机底部
  - 通过 PnP 得到相机(无人机)相对于标记(降落点)的位移
  - tx, ty → 水平偏差, tz → 高度
  - 用这些偏差控制无人机飞向降落点

参考: OpenCV solvePnP / solvePnPRefineLM
"""
import numpy as np
import cv2
from typing import Tuple, Optional, List


class ArUcoDetector:
    """ArUco 标记检测器

    支持多种 ArUco 字典:
    - DICT_4X4_50: 4x4 位, 50 个标记 (推荐, 小标记也能检测)
    - DICT_5X5_100: 5x5 位, 100 个标记
    - DICT_6X6_250: 6x6 位, 250 个标记
    """
    DICTS = {
        "4x4_50": cv2.aruco.DICT_4X4_50,
        "4x4_100": cv2.aruco.DICT_4X4_100,
        "5x5_50": cv2.aruco.DICT_5X5_50,
        "5x5_100": cv2.aruco.DICT_5X5_100,
        "6x6_250": cv2.aruco.DICT_6X6_250,
    }

    def __init__(self, dict_name: str = "4x4_50", marker_size: float = 0.2):
        """
        Args:
            dict_name: ArUco 字典名称
            marker_size: 标记物理尺寸 (米), 用于 PnP 计算
        """
        dict_id = self.DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_size = marker_size  # meters

    def detect(self, image: np.ndarray) -> List[dict]:
        """检测图像中的 ArUco 标记

        Returns:
            list of dict: [{
                "id": int,              标记 ID
                "corners": ndarray,     4个角点 (4x2)
                "center": (cx, cy),     中心像素坐标
                "area": float,          像素面积
            }, ...]
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        results = []
        if ids is not None:
            for i, (corner, marker_id) in enumerate(zip(corners, ids)):
                pts = corner[0]  # shape: (4, 2)
                cx = pts[:, 0].mean()
                cy = pts[:, 1].mean()
                area = cv2.contourArea(pts.astype(np.float32))
                results.append({
                    "id": int(marker_id[0]),
                    "corners": pts,
                    "center": (cx, cy),
                    "area": area,
                })
        return results


class PnPPoseEstimator:
    """PnP 6DOF 位姿估计器

    用 ArUco 4个角点的 3D-2D 对应关系, 求解相机位姿

    输出:
    - tvec: 平移向量 [tx, ty, tz] (米), 相机到标记
    - rvec: 旋转向量 [rx, ry, rz] (弧度), 相机到标记
    - 可选: 转换为欧拉角 (roll, pitch, yaw)
    """
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                 marker_size: float = 0.2):
        """
        Args:
            camera_matrix: 3x3 相机内参矩阵
            dist_coeffs: 畸变系数
            marker_size: 标记物理尺寸 (米)
        """
        self.K = camera_matrix
        self.dist = dist_coeffs
        self.marker_size = marker_size

        # 标记 3D 坐标 (以标记中心为原点, 标记在 z=0 平面)
        half = marker_size / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],  # 左上
            [ half,  half, 0],  # 右上
            [ half, -half, 0],  # 右下
            [-half, -half, 0],  # 左下
        ], dtype=np.float64)

    def estimate(self, corners_2d: np.ndarray) -> Optional[dict]:
        """从 4 个角点估计 6DOF 位姿

        Args:
            corners_2d: shape (4, 2), 像素坐标

        Returns:
            dict with:
                "tvec": [tx, ty, tz] 平移 (米)
                "rvec": [rx, ry, rz] 旋转 (弧度)
                "euler_deg": [roll, pitch, yaw] 欧拉角 (度)
                "distance": 到标记的距离 (米)
                "success": bool
        """
        corners = corners_2d.astype(np.float64)

        # solvePnP: 已知 3D 点和对应 2D 点, 求相机位姿
        success, rvec, tvec = cv2.solvePnP(
            self.obj_points, corners, self.K, self.dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)

        if not success:
            return None

        # 精化 (Levenberg-Marquardt)
        rvec, tvec = cv2.solvePnPRefineLM(
            self.obj_points, corners, self.K, self.dist, rvec, tvec)

        # 转换为欧拉角
        euler = self._rotation_vec_to_euler(rvec)

        # 计算欧氏距离
        distance = float(np.linalg.norm(tvec))

        return {
            "tvec": tvec.flatten().tolist(),
            "rvec": rvec.flatten().tolist(),
            "euler_deg": euler,
            "distance": distance,
            "tx": float(tvec[0]),
            "ty": float(tvec[1]),
            "tz": float(tvec[2]),
            "success": True,
        }

    def _rotation_vec_to_euler(self, rvec) -> list:
        """旋转向量 → 欧拉角 (度)"""
        R, _ = cv2.Rodrigues(rvec)
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        if sy > 1e-6:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0
        return [np.degrees(roll), np.degrees(pitch), np.degrees(yaw)]

    @staticmethod
    def create_default_camera_matrix(fx: float = 500, fy: float = 500,
                                      cx: float = 320, cy: float = 240) -> np.ndarray:
        """创建默认相机内参矩阵"""
        return np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0,  0,  1],
        ], dtype=np.float64)
