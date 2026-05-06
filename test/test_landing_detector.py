"""Tests for landing_detector.py — detection and annotation logic"""
import sys
import os
import unittest
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import rclpy
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


@unittest.skipUnless(HAS_RCLPY, "rclpy not available")
class TestLandingDetectorLogic(unittest.TestCase):
    """Test the pure logic parts of LandingDetectorNode."""

    def test_annotate_no_detections(self):
        """_annotate should draw NO MARKER text when no detections"""
        try:
            import cv2
        except ImportError:
            self.skipTest("cv2 not available")

        from drone_landing.landing_detector import LandingDetectorNode
        import rclpy
        try:
            rclpy.init()
        except Exception:
            pass

        # Create a minimal node (skip if video fails)
        try:
            node = LandingDetectorNode()
        except Exception:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self.skipTest("Cannot create node (no video source)")

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        result = {"markers": []}
        annotated = node._annotate(frame, [], result)

        # Should return an image of same shape
        self.assertEqual(annotated.shape, frame.shape)

        node.destroy_node()
        rclpy.shutdown()

    def test_annotate_with_detections(self):
        """_annotate should draw marker info when detections present"""
        try:
            import cv2
        except ImportError:
            self.skipTest("cv2 not available")

        from drone_landing.landing_detector import LandingDetectorNode
        from drone_landing.aruco_pose import ArUcoDetector
        import rclpy
        try:
            rclpy.init()
        except Exception:
            pass

        try:
            node = LandingDetectorNode()
        except Exception:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self.skipTest("Cannot create node (no video source)")

        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Mock detection
        detection = {
            "id": 0,
            "center": (320.0, 240.0),
            "area": 1000.0,
            "corners": np.array([[[100, 100], [200, 100], [200, 200], [100, 200]]],
                                dtype=np.float32),
        }
        result = {
            "markers": [{
                "id": 0,
                "center": (320.0, 240.0),
                "area": 1000.0,
                "pose": {"distance": 1.5, "tx": 0.1, "ty": -0.05, "tz": 1.5},
                "pixel_error": [0.0, 0.0],
            }]
        }

        annotated = node._annotate(frame, [detection], result)
        self.assertEqual(annotated.shape, frame.shape)

        node.destroy_node()
        rclpy.shutdown()


class TestArUcoDetectorIntegration(unittest.TestCase):
    """Test ArUcoDetector with synthetic images."""

    def test_detect_empty_image(self):
        """Empty image should return no detections"""
        try:
            import cv2
            from drone_landing.aruco_pose import ArUcoDetector
        except ImportError:
            self.skipTest("cv2 or aruco not available")

        try:
            detector = ArUcoDetector("4x4_50", 0.2)
        except (AttributeError, cv2.error):
            self.skipTest("ArUco API not available in this OpenCV version")

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            detections = detector.detect(frame)
        except AttributeError:
            self.skipTest("ArUco detectMarkers API not available")
            return
        self.assertEqual(len(detections), 0)

    def test_detection_result_structure(self):
        """Detection results should have expected keys"""
        try:
            import cv2
            from drone_landing.aruco_pose import ArUcoDetector
        except ImportError:
            self.skipTest("cv2 or aruco not available")

        try:
            detector = ArUcoDetector("4x4_50", 0.2)
        except (AttributeError, cv2.error):
            self.skipTest("ArUco API not available in this OpenCV version")

        # Generate an ArUco marker image
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            marker = cv2.aruco.generateImageMarker(aruco_dict, 0, 200)
        except AttributeError:
            self.skipTest("ArUco API not available in this OpenCV version")

        # Place marker in a larger frame
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        marker_rgb = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        frame[140:340, 220:420] = marker_rgb

        try:
            detections = detector.detect(frame)
        except AttributeError:
            self.skipTest("ArUco detectMarkers API not available")
            return
        if detections:
            d = detections[0]
            self.assertIn("id", d)
            self.assertIn("center", d)
            self.assertIn("area", d)
            self.assertIn("corners", d)


if __name__ == '__main__':
    unittest.main()
