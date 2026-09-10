"""Pi actuation node (design section 6): Twist -> diff_drive_mix -> PWM.

Subscribes ``/osoyoo_4/cmd_vel`` (geometry_msgs/Twist, robot frame: linear.x m/s,
angular.z rad/s) and ``/estop`` (std_msgs/Bool, latching stop).  A watchdog stops the
motors when no Twist has arrived for ``watchdog_s`` (0.3 s).  ``/osoyoo_4/wheel_cmd``
(std_msgs/Float64MultiArray ``[left, right]`` in [-1, 1]) drives the wheels directly for
the motor calibration script.  ``stopcar`` + ``GPIO.cleanup()`` in ``destroy_node``.

No Vicon, no model, no vicon_receiver dependency: only rclpy, geometry_msgs, std_msgs and
the motor layer.  Without ``RPi.GPIO`` / ``Adafruit_PCA9685`` the node runs with
:class:`FakeHardware` (parameter ``fake_hardware:=true`` forces it).
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

from .motors import MotorParams, custom_speed, diff_drive_mix, setup, stopcar


class OsoyooBase(Node):
    def __init__(self, hardware=None):
        super().__init__("osoyoo_base")
        defaults = MotorParams()
        self.declare_parameter("cmd_vel_topic", "/osoyoo_4/cmd_vel")
        self.declare_parameter("wheel_cmd_topic", "/osoyoo_4/wheel_cmd")
        self.declare_parameter("estop_topic", "/estop")
        self.declare_parameter("watchdog_s", 0.3)
        self.declare_parameter("watchdog_rate_hz", 20.0)
        self.declare_parameter("fake_hardware", False)
        self.declare_parameter("wheel_base_m", defaults.wheel_base_m)
        self.declare_parameter("max_lin", defaults.max_lin)
        self.declare_parameter("max_ang", defaults.max_ang)
        self.declare_parameter("min_pwm", defaults.min_pwm)
        self.declare_parameter("max_pwm", defaults.max_pwm)
        self.declare_parameter("deadband", defaults.deadband)
        self.declare_parameter("deadband_cmd", defaults.deadband_cmd)
        self.declare_parameter("left_dir_sign", defaults.left_dir_sign)
        self.declare_parameter("right_dir_sign", defaults.right_dir_sign)
        self.params = MotorParams(
            wheel_base_m=float(self.get_parameter("wheel_base_m").value),
            max_lin=float(self.get_parameter("max_lin").value),
            max_ang=float(self.get_parameter("max_ang").value),
            min_pwm=int(self.get_parameter("min_pwm").value),
            max_pwm=int(self.get_parameter("max_pwm").value),
            deadband=int(self.get_parameter("deadband").value),
            deadband_cmd=float(self.get_parameter("deadband_cmd").value),
            left_dir_sign=int(self.get_parameter("left_dir_sign").value),
            right_dir_sign=int(self.get_parameter("right_dir_sign").value),
        )
        self.hw = hardware if hardware is not None else setup(self.params, force_fake=bool(self.get_parameter("fake_hardware").value))
        self.watchdog_s = float(self.get_parameter("watchdog_s").value)
        self.estopped = False
        self.last_cmd_t = None
        self.moving = False
        self.last_wheels = ((0, 0), (0, 0))
        self.n_cmds = 0
        self.n_ignored = 0
        self.create_subscription(Twist, self.get_parameter("cmd_vel_topic").value, self.cmd_cb, 1)
        self.create_subscription(Float64MultiArray, self.get_parameter("wheel_cmd_topic").value, self.wheel_cb, 1)
        self.create_subscription(Bool, self.get_parameter("estop_topic").value, self.estop_cb, 1)
        self.create_timer(1.0 / float(self.get_parameter("watchdog_rate_hz").value), self.watchdog)
        self.get_logger().info(
            f"osoyoo_base ready ({type(self.hw).__name__}) wheel_base={self.params.wheel_base_m} "
            f"max_lin={self.params.max_lin} max_ang={self.params.max_ang} watchdog={self.watchdog_s}s"
        )

    # ------------------------------------------------------------------ callbacks
    def cmd_cb(self, msg: Twist) -> None:
        if self.estopped:
            self.n_ignored += 1
            return
        self.last_cmd_t = self.get_clock().now()
        left, right = diff_drive_mix(float(msg.linear.x), float(msg.angular.z), self.params)
        self.last_wheels = custom_speed(self.hw, left, right, self.params)
        self.moving = any(pwm > 0 for _, pwm in self.last_wheels)
        self.n_cmds += 1

    def wheel_cb(self, msg: Float64MultiArray) -> None:
        if self.estopped or len(msg.data) < 2:
            self.n_ignored += 1
            return
        self.last_cmd_t = self.get_clock().now()
        self.last_wheels = custom_speed(self.hw, float(msg.data[0]), float(msg.data[1]), self.params)
        self.moving = any(pwm > 0 for _, pwm in self.last_wheels)
        self.n_cmds += 1

    def watchdog(self) -> None:
        if self.last_cmd_t is None:
            return
        if (self.get_clock().now() - self.last_cmd_t).nanoseconds > self.watchdog_s * 1e9:
            if self.moving:
                self.get_logger().warning("watchdog: no command for %.2fs, stopping" % self.watchdog_s)
            self._stop()

    def estop_cb(self, msg: Bool) -> None:
        self.estopped = bool(msg.data)
        if self.estopped:
            self.get_logger().warning("ESTOP latched")
            self._stop()
        else:
            self.get_logger().info("ESTOP released")

    def _stop(self) -> None:
        stopcar(self.hw)
        self.moving = False
        self.last_wheels = ((0, 0), (0, 0))

    def destroy_node(self):
        try:
            stopcar(self.hw)
            self.hw.cleanup()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OsoyooBase()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
