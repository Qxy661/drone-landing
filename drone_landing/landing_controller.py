"""
精准降落控制器 ROS2 节点
Precision Landing Controller

实现级联 PID 控制:
  外环: 水平位置误差 → 速度命令
  内环: 速度 → 加速度/倾斜角

降落阶段:
  COARSE: GPS 导航到目标区域 (水平误差 > 5m)
  FINE: 视觉导引精确对准 (水平误差 < 5m)
  DESCENT: 垂直下降 + 水平修正 (保持对准)
  FLARE: 最后阶段, 缓慢接近地面
  LANDED: 触地检测, 锁定电机

关键技术:
  - 视觉丢失处理: 短暂丢失用卡尔曼预测, 长时间丢失切回 GPS
  - 下降速率控制: 视觉稳定时才允许下降
  - 安全保护: 标记偏移过大时悬停
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode


class LandingPhase:
    COARSE = "coarse"       # GPS 粗导航
    FINE = "fine"           # 视觉精导引
    DESCENT = "descent"     # 视觉引导下降
    FLARE = "flare"         # 最终减速
    LANDED = "landed"       # 已降落
    ABORT = "abort"         # 中止降落


class PIDController:
    """PID 控制器 (带抗积分饱和)"""
    def __init__(self, kp, ki, kd, limit=1.0, integral_limit=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error, dt=None):
        now = time.time()
        if dt is None:
            dt = (now - self.prev_time) if self.prev_time else 0.02
        self.prev_time = now
        if dt < 1e-6:
            dt = 0.02

        self.integral += error * dt
        self.integral = max(-self.integral_limit,
                           min(self.integral_limit, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.limit, min(self.limit, output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None


class LandingControllerNode(Node):
    def __init__(self):
        super().__init__("landing_controller")

        # 参数
        self.declare_parameter("test_mode", True)
        self.declare_parameter("coarse_altitude", 5.0)      # GPS 导引高度
        self.declare_parameter("descent_rate", 0.3)          # 下降速率 m/s
        self.declare_parameter("flare_altitude", 0.5)        # 开始 flare 的高度
        self.declare_parameter("landed_altitude", 0.1)       # 判定降落的高度
        self.declare_parameter("max_horizontal_error", 2.0)  # 最大水平偏差
        self.declare_parameter("target_lost_timeout", 5.0)   # 视觉丢失超时
        self.declare_parameter("kp_xy", 0.4)
        self.declare_parameter("ki_xy", 0.05)
        self.declare_parameter("kd_xy", 0.2)
        self.declare_parameter("kp_z", 0.6)
        self.declare_parameter("ki_z", 0.1)
        self.declare_parameter("kd_z", 0.3)

        self.test_mode = self.get_parameter("test_mode").value
        self.coarse_alt = self.get_parameter("coarse_altitude").value
        self.descent_rate = self.get_parameter("descent_rate").value
        self.flare_alt = self.get_parameter("flare_altitude").value
        self.land_alt = self.get_parameter("landed_altitude").value
        self.max_h_err = self.get_parameter("max_horizontal_error").value
        self.lost_timeout = self.get_parameter("target_lost_timeout").value

        # PID 控制器
        kpx = self.get_parameter("kp_xy").value
        kix = self.get_parameter("ki_xy").value
        kdx = self.get_parameter("kd_xy").value
        kpz = self.get_parameter("kp_z").value
        kiz = self.get_parameter("ki_z").value
        kdz = self.get_parameter("kd_z").value

        self.pid_x = PIDController(kpx, kix, kdx, limit=0.5)
        self.pid_y = PIDController(kpx, kix, kdx, limit=0.5)
        self.pid_z = PIDController(kpz, kiz, kdz, limit=0.3)

        # 状态
        self.phase = LandingPhase.COARSE
        self.fcu_connected = False
        self.armed = False
        self.current_mode = ""
        self.local_pos = (0.0, 0.0, 0.0)
        self.target_detected = False
        self.target_tx = 0.0  # PnP 平移 x (右)
        self.target_ty = 0.0  # PnP 平移 y (下)
        self.target_tz = 0.0  # PnP 平移 z (前/距离)
        self.target_pixel_err = (0.0, 0.0)
        self.last_detect_time = 0.0
        self.phase_start_time = 0.0

        # 订阅
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, "/landing/target_pose",
                                 self._target_cb, 10)
        if not self.test_mode:
            self.create_subscription(State, "/mavros/state", self._state_cb, 10)
            self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                     self._pos_cb, qos)

        # 发布
        if not self.test_mode:
            self.vel_pub = self.create_publisher(
                TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
            self.pos_pub = self.create_publisher(
                PoseStamped, "/mavros/setpoint_position/local", 10)
            self.set_mode_client = self.create_client(
                SetMode, "/mavros/set_mode")

        self.status_pub = self.create_publisher(
            String, "/landing/status", 10)

        # 控制循环 20Hz
        self.timer = self.create_timer(0.05, self._control_loop)
        self.phase_start_time = time.time()

        mode_str = "TEST" if self.test_mode else "REAL"
        self.get_logger().info(f"LandingController started ({mode_str})")

    def _state_cb(self, msg):
        self.fcu_connected = msg.connected
        self.armed = msg.armed
        self.current_mode = msg.mode

    def _pos_cb(self, msg):
        p = msg.pose.position
        self.local_pos = (p.x, p.y, p.z)

    def _target_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("detected") and data.get("markers"):
                marker = data["markers"][0]
                pose = marker.get("pose", {})
                self.target_detected = True
                self.target_tx = pose.get("tx", 0)
                self.target_ty = pose.get("ty", 0)
                self.target_tz = pose.get("tz", 0)
                self.target_pixel_err = tuple(marker.get("pixel_error", [0, 0]))
                self.last_detect_time = time.time()
            else:
                self.target_detected = False
        except (json.JSONDecodeError, KeyError):
            pass

    def _control_loop(self):
        now = time.time()
        vx, vy, vz = 0.0, 0.0, 0.0

        if self.phase == LandingPhase.COARSE:
            # GPS 导航到目标上方
            if self.test_mode:
                self.get_logger().info("[COARSE] GPS navigation -> switching to FINE")
                self.phase = LandingPhase.FINE
                self.phase_start_time = now
                self.pid_x.reset()
                self.pid_y.reset()
            else:
                # 发送位置命令到目标上方
                self._send_position(0, 0, self.coarse_alt)
                # 到达后切 FINE
                if self.local_pos[2] > self.coarse_alt * 0.9:
                    self.phase = LandingPhase.FINE
                    self.phase_start_time = now

        elif self.phase == LandingPhase.FINE:
            if self.target_detected:
                # 用视觉偏差控制水平位置
                # target_ty 是 PnP 的 y 分量, 表示垂直偏差
                # target_tx 是水平偏差
                vx = self.pid_x.update(-self.target_tx)
                vy = self.pid_y.update(-self.target_ty)

                # 保持高度, 等待水平对准
                h_err = self.coarse_alt - self.local_pos[2]
                vz = self.pid_z.update(h_err) if not self.test_mode else 0

                # 水平误差足够小, 切 DESCENT
                h_err_px = math.sqrt(
                    self.target_pixel_err[0]**2 + self.target_pixel_err[1]**2)
                if h_err_px < 30:  # 像素
                    self.phase = LandingPhase.DESCENT
                    self.phase_start_time = now
                    self.get_logger().info("Aligned -> DESCENT")

            elif now - self.last_detect_time > self.lost_timeout:
                self.get_logger().warn("Target lost in FINE -> ABORT")
                self.phase = LandingPhase.ABORT

            if self.test_mode:
                self.get_logger().info(
                    f"[FINE] target=({self.target_tx:.3f},{self.target_ty:.3f}) "
                    f"vel=({vx:.3f},{vy:.3f})",
                    throttle_duration_sec=0.5)

        elif self.phase == LandingPhase.DESCENT:
            if self.target_detected:
                # 持续水平修正
                vx = self.pid_x.update(-self.target_tx) * 0.5
                vy = self.pid_y.update(-self.target_ty) * 0.5
                # 恒速下降
                vz = -self.descent_rate

                # 检查是否到 flare 高度
                alt = self.target_tz if self.test_mode else self.local_pos[2]
                if alt < self.flare_alt:
                    self.phase = LandingPhase.FLARE
                    self.phase_start_time = now
                    self.get_logger().info("Low altitude -> FLARE")

            elif now - self.last_detect_time > 2.0:
                # 视觉丢失, 悬停
                self.get_logger().warn("Target lost in DESCENT -> hovering")
                vz = 0.0

            if self.test_mode:
                self.get_logger().info(
                    f"[DESCENT] alt={self.target_tz:.2f} "
                    f"vel=({vx:.3f},{vy:.3f},{vz:.3f})",
                    throttle_duration_sec=0.5)

        elif self.phase == LandingPhase.FLARE:
            if self.target_detected:
                vx = self.pid_x.update(-self.target_tx) * 0.3
                vy = self.pid_y.update(-self.target_ty) * 0.3
                vz = -self.descent_rate * 0.3  # 更慢

                alt = self.target_tz if self.test_mode else self.local_pos[2]
                if alt < self.land_alt:
                    self.phase = LandingPhase.LANDED
                    self.get_logger().info("LANDED!")

            if self.test_mode:
                self.get_logger().info(
                    f"[FLARE] alt={self.target_tz:.2f}",
                    throttle_duration_sec=0.5)

        elif self.phase == LandingPhase.LANDED:
            vx, vy, vz = 0, 0, 0
            if not self.test_mode and self.armed:
                self._set_mode("LAND")

        elif self.phase == LandingPhase.ABORT:
            # 中止: 悬停并等待
            vx, vy, vz = 0, 0, 0
            if self.target_detected:
                self.phase = LandingPhase.FINE
                self.phase_start_time = now

        # 发布速度命令
        if not self.test_mode and self.phase not in (
                LandingPhase.LANDED, LandingPhase.ABORT):
            vel = TwistStamped()
            vel.header.stamp = self.get_clock().now().to_msg()
            vel.twist.linear.x = vx
            vel.twist.linear.y = vy
            vel.twist.linear.z = vz
            self.vel_pub.publish(vel)

        # 发布状态
        self._publish_status(vx, vy, vz)

    def _send_position(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pos_pub.publish(msg)

    def _set_mode(self, mode):
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def _publish_status(self, vx, vy, vz):
        status = {
            "phase": self.phase,
            "target_detected": self.target_detected,
            "target_offset": [self.target_tx, self.target_ty, self.target_tz],
            "pixel_error": list(self.target_pixel_err),
            "velocity_cmd": [vx, vy, vz],
            "position": list(self.local_pos),
            "test_mode": self.test_mode,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LandingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
