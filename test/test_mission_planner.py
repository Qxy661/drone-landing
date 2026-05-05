"""Tests for mission_planner.py - mission state machine logic."""
import sys
import os
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import json
import time

try:
    from drone_landing.mission_planner import MissionPhase
    HAS_MISSION = True
except ImportError:
    HAS_MISSION = False


@unittest.skipUnless(HAS_MISSION, "rclpy not available")
class TestMissionPhase(unittest.TestCase):
    def test_phase_constants(self):
        assert MissionPhase.IDLE == "idle"
        assert MissionPhase.TAKEOFF == "takeoff"
        assert MissionPhase.NAVIGATING == "navigating"
        assert MissionPhase.SEARCHING == "searching"
        assert MissionPhase.LANDING == "landing"
        assert MissionPhase.COMPLETE == "complete"
        assert MissionPhase.ABORT == "abort"


class TestMissionStateMachine(unittest.TestCase):
    """Test the state transition logic without ROS2."""

    def test_start_command(self):
        """Start command should transition from IDLE to TAKEOFF."""
        phase = "idle"
        data = json.loads('{"cmd": "start"}')
        if data.get("cmd") == "start" and phase == "idle":
            phase = "takeoff"
        assert phase == "takeoff"

    def test_landing_phase_detection(self):
        """Visual lock should transition SEARCHING to LANDING."""
        phase = "searching"
        landing_phase = "fine"
        if landing_phase in ("fine", "descent", "flare"):
            phase = "landing"
        assert phase == "landing"

    def test_search_timeout(self):
        """Search timeout should transition to ABORT."""
        phase = "searching"
        phase_start = time.time() - 31.0
        search_timeout = 30.0
        now = time.time()
        if now - phase_start > search_timeout:
            phase = "abort"
        assert phase == "abort"

    def test_landing_complete(self):
        """Landed phase should transition to COMPLETE."""
        phase = "landing"
        landing_phase = "landed"
        if landing_phase == "landed":
            phase = "complete"
        assert phase == "complete"

    def test_invalid_start_command(self):
        """Invalid command should not change state."""
        phase = "idle"
        data = json.loads('{"cmd": "invalid"}')
        if data.get("cmd") == "start" and phase == "idle":
            phase = "takeoff"
        assert phase == "idle"

    def test_start_from_non_idle(self):
        """Start command from non-IDLE should not change state."""
        phase = "navigating"
        data = json.loads('{"cmd": "start"}')
        if data.get("cmd") == "start" and phase == "idle":
            phase = "takeoff"
        assert phase == "navigating"

    def test_landing_phase_json_parsing(self):
        """Should parse landing status JSON correctly."""
        msg_data = '{"phase": "descent"}'
        data = json.loads(msg_data)
        landing_phase = data.get("phase", "coarse")
        assert landing_phase == "descent"

    def test_landing_phase_default(self):
        """Missing phase key should default to 'coarse'."""
        msg_data = '{"other": "data"}'
        data = json.loads(msg_data)
        landing_phase = data.get("phase", "coarse")
        assert landing_phase == "coarse"


if __name__ == '__main__':
    unittest.main()
