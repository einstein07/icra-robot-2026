"""osoyoo_base plumbing test on the PC with FakeHardware (no GPIO).

Publish a Twist -> wheel commands logged with the right signs; stop publishing -> stopcar
within 0.35 s; publish /estop true -> commands ignored.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray  # noqa: E402

from osoyoo_base.base_node import CMD_QOS, OsoyooBase  # noqa: E402
from osoyoo_base.motors import (  # noqa: E402
    FakeHardware,
    MotorParams,
    cmd_to_pwm,
    diff_drive_mix,
    wheel_channels,
)


def test_diff_drive_mix_and_pwm_map():
    p = MotorParams(wheel_base_m=0.12, max_lin=0.30)
    l, r = diff_drive_mix(0.15, 0.0, p)
    assert l == pytest.approx(0.5) and r == pytest.approx(0.5)
    l, r = diff_drive_mix(0.0, 1.0, p)                 # left turn (CCW): left wheel backwards
    assert l < 0 < r and abs(l) == pytest.approx(abs(r))
    l, r = diff_drive_mix(0.30, 5.0, p)                 # saturates but keeps the ratio sign
    assert max(abs(l), abs(r)) == pytest.approx(1.0) and l < r
    assert cmd_to_pwm(0.0, p) == (0, 0)
    assert cmd_to_pwm(0.01, p) == (0, 0)                # below deadband_cmd
    d, pwm = cmd_to_pwm(0.5, p)
    assert d == 1 and p.min_pwm < pwm < p.max_pwm
    d, pwm = cmd_to_pwm(-1.0, p)
    assert d == -1 and pwm == p.max_pwm


def test_wheel_channels_swap():
    """This chassis is wired A=right / B=left; without the swap the turn sign inverts."""
    p = MotorParams()
    assert p.swap_motor_channels is True
    assert wheel_channels("left", p) == (p.enb, p.in3, p.in4)
    assert wheel_channels("right", p) == (p.ena, p.in1, p.in2)
    straight = MotorParams(swap_motor_channels=False)
    assert wheel_channels("left", straight) == (p.ena, p.in1, p.in2)
    assert wheel_channels("right", straight) == (p.enb, p.in3, p.in4)


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.try_shutdown()


def _spin(executor, stop_event):
    while not stop_event.is_set():
        executor.spin_once(timeout_sec=0.02)


def test_node_plumbing(ros_context):
    hw = FakeHardware(MotorParams())
    node = OsoyooBase(hardware=hw)
    node.set_parameters([rclpy.parameter.Parameter("watchdog_s", value=0.3)])
    tester = rclpy.create_node("plumbing_tester")
    cmd_pub = tester.create_publisher(Twist, "/osoyoo_4/cmd_vel", CMD_QOS)
    wheel_pub = tester.create_publisher(Float64MultiArray, "/osoyoo_4/wheel_cmd", 1)
    estop_pub = tester.create_publisher(Bool, "/estop", 1)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(tester)
    stop = threading.Event()
    thread = threading.Thread(target=_spin, args=(executor, stop), daemon=True)
    thread.start()
    try:
        time.sleep(0.3)   # discovery
        # forward: both wheels forward with equal PWM
        msg = Twist()
        msg.linear.x = 0.2
        t0 = time.time()
        while node.n_cmds == 0 and time.time() - t0 < 2.0:
            cmd_pub.publish(msg)
            time.sleep(0.05)
        assert node.n_cmds > 0, "no Twist received"
        assert hw.left[0] == 1 and hw.right[0] == 1 and hw.left[1] == hw.right[1] > 0
        # left turn: left wheel backwards, right forwards
        msg = Twist()
        msg.angular.z = 1.0
        n = node.n_cmds
        while node.n_cmds == n and time.time() - t0 < 4.0:
            cmd_pub.publish(msg)
            time.sleep(0.05)
        assert hw.left[0] == -1 and hw.right[0] == 1
        assert not hw.stopped
        # stop publishing -> watchdog stops within 0.35 s
        t_last = time.time()
        while not hw.stopped and time.time() - t_last < 0.6:
            time.sleep(0.01)
        assert hw.stopped, "watchdog did not stop the motors"
        assert time.time() - t_last <= 0.35 + 0.1   # 0.3 s watchdog at 20 Hz polling
        # estop: commands ignored
        estop = Bool()
        estop.data = True
        for _ in range(5):
            estop_pub.publish(estop)
            time.sleep(0.05)
        assert node.estopped
        n_ign = node.n_ignored
        for _ in range(5):
            cmd_pub.publish(msg)
            time.sleep(0.05)
        assert node.n_ignored > n_ign and hw.stopped
        # wheel_cmd debug topic after estop release
        estop.data = False
        for _ in range(5):
            estop_pub.publish(estop)
            time.sleep(0.05)
        wm = Float64MultiArray()
        wm.data = [0.5, -0.5]
        n = node.n_cmds
        while node.n_cmds == n and time.time() - t0 < 8.0:
            wheel_pub.publish(wm)
            time.sleep(0.05)
        assert hw.left[0] == 1 and hw.right[0] == -1
    finally:
        stop.set()
        thread.join(timeout=2)
        node.destroy_node()
        tester.destroy_node()
        assert hw.cleaned_up
