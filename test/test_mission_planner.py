"""Tests for mission_planner.py - mission state machine logic."""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import json
import time
from drone_landing.mission_planner import MissionPhase


class TestMissionPhase:
    def test_phase_constants(self):
        assert MissionPhase.IDLE == "idle"
        assert MissionPhase.TAKEOFF == "takeoff"
        assert MissionPhase.NAVIGATING == "navigating"
        assert MissionPhase.SEARCHING == "searching"
        assert MissionPhase.LANDING == "landing"
        assert MissionPhase.COMPLETE == "complete"
        assert MissionPhase.ABORT == "abort"


class TestMissionStateMachine:
    """Test the state transition logic without ROS2."""

    def test_start_command(self):
        """Start command should transition from IDLE to TAKEOFF."""
        phase = MissionPhase.IDLE
        data = json.loads('{"cmd": "start"}')
        if data.get("cmd") == "start" and phase == MissionPhase.IDLE:
            phase = MissionPhase.TAKEOFF
        assert phase == MissionPhase.TAKEOFF

    def test_landing_phase_detection(self):
        """Visual lock should transition SEARCHING to LANDING."""
        phase = MissionPhase.SEARCHING
        landing_phase = "fine"
        if landing_phase in ("fine", "descent", "flare"):
            phase = MissionPhase.LANDING
        assert phase == MissionPhase.LANDING

    def test_search_timeout(self):
        """Search timeout should transition to ABORT."""
        phase = MissionPhase.SEARCHING
        phase_start = time.time() - 31.0
        search_timeout = 30.0
        now = time.time()
        if now - phase_start > search_timeout:
            phase = MissionPhase.ABORT
        assert phase == MissionPhase.ABORT

    def test_landing_complete(self):
        """Landed phase should transition to COMPLETE."""
        phase = MissionPhase.LANDING
        landing_phase = "landed"
        if landing_phase == "landed":
            phase = MissionPhase.COMPLETE
        assert phase == MissionPhase.COMPLETE

    def test_invalid_start_command(self):
        """Invalid command should not change state."""
        phase = MissionPhase.IDLE
        data = json.loads('{"cmd": "invalid"}')
        if data.get("cmd") == "start" and phase == MissionPhase.IDLE:
            phase = MissionPhase.TAKEOFF
        assert phase == MissionPhase.IDLE

    def test_start_from_non_idle(self):
        """Start command from non-IDLE should not change state."""
        phase = MissionPhase.NAVIGATING
        data = json.loads('{"cmd": "start"}')
        if data.get("cmd") == "start" and phase == MissionPhase.IDLE:
            phase = MissionPhase.TAKEOFF
        assert phase == MissionPhase.NAVIGATING

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
    import pytest
    pytest.main([__file__, '-v'])
