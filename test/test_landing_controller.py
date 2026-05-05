"""Tests for landing_controller.py - PID controller and landing phases."""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
from drone_landing.landing_controller import PIDController, LandingPhase


class TestPIDController:
    def test_proportional(self):
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, limit=10.0)
        pid.prev_time = time.time() - 0.02
        out = pid.update(error=5.0, dt=0.02)
        assert abs(out - 5.0) < 0.1

    def test_integral(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        pid.prev_time = time.time() - 0.02
        for _ in range(5):
            out = pid.update(error=1.0, dt=0.1)
        assert abs(out - 0.5) < 0.01

    def test_derivative(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, limit=100.0)
        pid.prev_time = time.time() - 0.02
        pid.update(error=0.0, dt=0.02)
        out = pid.update(error=1.0, dt=0.02)
        # derivative = (1.0 - 0.0) / 0.02 = 50
        assert abs(out - 50.0) < 1.0

    def test_output_limit(self):
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, limit=1.0)
        pid.prev_time = time.time() - 0.02
        out = pid.update(error=10.0, dt=0.02)
        assert abs(out - 1.0) < 0.01

    def test_integral_anti_windup(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, limit=1.0, integral_limit=0.5)
        pid.prev_time = time.time() - 0.02
        for _ in range(100):
            pid.update(error=100.0, dt=0.02)
        assert abs(pid.integral) <= 0.5 + 0.01

    def test_reset(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        pid.prev_time = time.time() - 0.02
        pid.update(error=5.0, dt=0.02)
        pid.reset()
        assert pid.integral == 0.0
        assert pid.prev_error == 0.0

    def test_dt_protection(self):
        """dt=0 should not cause division by zero."""
        pid = PIDController(kp=1.0, ki=0.0, kd=1.0, limit=10.0)
        pid.prev_time = time.time()
        out = pid.update(error=1.0, dt=0.0)  # dt=0
        assert isinstance(out, float)  # should not crash

    def test_negative_dt(self):
        """Negative dt should be handled."""
        pid = PIDController(kp=1.0, ki=0.0, kd=1.0, limit=10.0)
        pid.prev_time = time.time()
        out = pid.update(error=1.0, dt=-0.01)
        assert isinstance(out, float)


class TestLandingPhase:
    def test_phase_constants(self):
        assert LandingPhase.COARSE == "coarse"
        assert LandingPhase.FINE == "fine"
        assert LandingPhase.DESCENT == "descent"
        assert LandingPhase.FLARE == "flare"
        assert LandingPhase.LANDED == "landed"
        assert LandingPhase.ABORT == "abort"


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
