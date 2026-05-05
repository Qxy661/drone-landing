"""
降落任务规划器 ROS2 节点
Landing Mission Planner

管理完整的降落任务流程:
1. 起飞到巡航高度
2. GPS 导航到目标区域
3. 切换视觉导引模式
4. 精准降落
5. 锁定电机

状态机:
  IDLE -> TAKEOFF -> NAVIGATING -> SEARCHING -> LANDING -> COMPLETE
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, SetMode


class MissionPhase:
    IDLE = "idle"
    TAKEOFF = "takeoff"
    NAVIGATING = "navigating"
    SEARCHING = "searching"
    LANDING = "landing"
    COMPLETE = "complete"
    ABORT = "abort"


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner")

        self.declare_parameter("test_mode", True)
        self.declare_parameter("cruise_altitude", 5.0)
        self.declare_parameter("target_lat", 0.0)
        self.declare_parameter("target_lon", 0.0)
        self.declare_parameter("search_timeout", 30.0)

        self.test_mode = self.get_parameter("test_mode").value
        self.cruise_alt = self.get_parameter("cruise_altitude").value
        self.target_lat = self.get_parameter("target_lat").value
        self.target_lon = self.get_parameter("target_lon").value
        self.search_timeout = self.get_parameter("search_timeout").value

        self.phase = MissionPhase.IDLE
        self.phase_start = 0.0
        self.landing_phase = "coarse"
        self.fcu_connected = False
        self.armed = False

        # 订阅
        self.create_subscription(String, "/landing/status",
                                 self._landing_cb, 10)
        self.create_subscription(String, "/mission/start",
                                 self._start_cb, 10)

        # 发布
        self.status_pub = self.create_publisher(
            String, "/mission/status", 10)

        # 服务
        if not self.test_mode:
            self.arming_client = self.create_client(
                CommandBool, "/mavros/cmd/arming")
            self.set_mode_client = self.create_client(
                SetMode, "/mavros/set_mode")

        self.timer = self.create_timer(0.5, self._update)
        self.get_logger().info(
            f"MissionPlanner started (test_mode={self.test_mode})")

    def _start_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("cmd") == "start":
                self.phase = MissionPhase.TAKEOFF
                self.phase_start = time.time()
                self.get_logger().info("Mission started -> TAKEOFF")
        except json.JSONDecodeError:
            pass

    def _landing_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.landing_phase = data.get("phase", "coarse")
        except json.JSONDecodeError:
            pass

    def _update(self):
        now = time.time()

        if self.phase == MissionPhase.IDLE:
            pass

        elif self.phase == MissionPhase.TAKEOFF:
            if self.test_mode:
                self.get_logger().info("[TEST] Takeoff -> NAVIGATING")
                self.phase = MissionPhase.NAVIGATING
                self.phase_start = now
            else:
                self._set_mode("GUIDED")
                self._arm(True)
                self.phase = MissionPhase.NAVIGATING
                self.phase_start = now

        elif self.phase == MissionPhase.NAVIGATING:
            if self.test_mode:
                self.get_logger().info("[TEST] Navigating -> SEARCHING")
                self.phase = MissionPhase.SEARCHING
                self.phase_start = now
            else:
                # TODO: 发送 GPS 航点到目标上方
                self.phase = MissionPhase.SEARCHING
                self.phase_start = now

        elif self.phase == MissionPhase.SEARCHING:
            if self.landing_phase in ("fine", "descent", "flare"):
                self.phase = MissionPhase.LANDING
                self.phase_start = now
                self.get_logger().info("Visual lock -> LANDING")
            elif now - self.phase_start > self.search_timeout:
                self.get_logger().warn("Search timeout -> ABORT")
                self.phase = MissionPhase.ABORT

        elif self.phase == MissionPhase.LANDING:
            if self.landing_phase == "landed":
                self.phase = MissionPhase.COMPLETE
                self.get_logger().info("Mission COMPLETE!")
                if not self.test_mode:
                    self._arm(False)

        elif self.phase == MissionPhase.COMPLETE:
            pass

        elif self.phase == MissionPhase.ABORT:
            pass

        self._publish_status()

    def _arm(self, arm):
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            return
        req = CommandBool.Request()
        req.value = arm
        self.arming_client.call_async(req)

    def _set_mode(self, mode):
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def _publish_status(self):
        status = {
            "mission_phase": self.phase,
            "landing_phase": self.landing_phase,
            "test_mode": self.test_mode,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
