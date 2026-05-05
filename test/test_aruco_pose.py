"""ArUco pose estimation unit tests"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import cv2
from drone_landing.aruco_pose import ArUcoDetector, PnPPoseEstimator


def test_create_default_matrix():
    K = PnPPoseEstimator.create_default_camera_matrix(500, 500, 320, 240)
    assert K.shape == (3, 3)
    assert K[0, 0] == 500  # fx
    assert K[1, 1] == 500  # fy
    assert K[0, 2] == 320  # cx
    assert K[1, 2] == 240  # cy
    print("  Default camera matrix: OK")


def test_pnp_known_pose():
    """Test PnP with synthetically generated correspondences"""
    K = PnPPoseEstimator.create_default_camera_matrix(500, 500, 320, 240)
    dist = np.zeros(5)
    estimator = PnPPoseEstimator(K, dist, marker_size=0.2)

    # Create known 3D points (marker corners)
    half = 0.1  # 0.2m marker
    obj_pts = np.array([
        [-half, half, 0],
        [half, half, 0],
        [half, -half, 0],
        [-half, -half, 0],
    ])

    # Project to 2D with known pose (1m in front, centered)
    rvec_true = np.array([0.0, 0.0, 0.0])
    tvec_true = np.array([0.0, 0.0, 1.0])
    img_pts, _ = cv2.projectPoints(obj_pts, rvec_true, tvec_true, K, dist)
    img_pts = img_pts.reshape(-1, 2)

    # Estimate pose
    result = estimator.estimate(img_pts)

    assert result is not None
    assert result["success"]
    # Distance should be ~1.0m
    assert abs(result["distance"] - 1.0) < 0.05
    # tx, ty should be ~0
    assert abs(result["tx"]) < 0.05
    assert abs(result["ty"]) < 0.05
    print(f"  PnP known pose: dist={result['distance']:.3f}m, "
          f"tx={result['tx']:.4f}, ty={result['ty']:.4f}")


def test_pnp_offset_pose():
    """Test PnP when marker is offset from camera center"""
    K = PnPPoseEstimator.create_default_camera_matrix(500, 500, 320, 240)
    dist = np.zeros(5)
    estimator = PnPPoseEstimator(K, dist, marker_size=0.2)

    half = 0.1
    obj_pts = np.array([
        [-half, half, 0],
        [half, half, 0],
        [half, -half, 0],
        [-half, -half, 0],
    ])

    # Marker 0.3m to the right, 1m forward
    tvec_true = np.array([0.3, 0.0, 1.0])
    rvec_true = np.array([0.0, 0.0, 0.0])
    img_pts, _ = cv2.projectPoints(obj_pts, rvec_true, tvec_true, K, dist)
    img_pts = img_pts.reshape(-1, 2)

    result = estimator.estimate(img_pts)
    assert result is not None
    assert abs(result["tx"] - 0.3) < 0.05
    print(f"  PnP offset: tx={result['tx']:.3f} (expected 0.3)")


if __name__ == "__main__":
    print("=== ArUco Pose Tests ===")
    test_create_default_matrix()
    test_pnp_known_pose()
    test_pnp_offset_pose()
    print("=== ALL TESTS PASSED ===")
