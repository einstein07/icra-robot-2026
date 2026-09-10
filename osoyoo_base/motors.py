"""Motor layer of the Osoyoo robot (Raspberry Pi + PCA9685 + IN1..IN4 H-bridge).

This is the motor code of the original ``target_follow_v2.py`` (``setup``,
``_apply_signed_pwm``, ``stopcar``, ``diff_drive_mix``, ``custom_speed``) with the geometry
and the PWM map as parameters, and the hardware imported inside ``try/except`` so the node
runs on a PC with :class:`FakeHardware` for plumbing tests.

Conventions
    ``cmd`` is a per-wheel command in ``[-1, 1]``; ``|cmd| < DEADBAND_CMD`` stops the wheel,
    otherwise the PWM is ``MIN_PWM + (MAX_PWM - MIN_PWM) * |cmd|``.  ``LEFT_DIR_SIGN`` /
    ``RIGHT_DIR_SIGN`` flip a wheel whose wiring runs backwards; confirm with a ``+0.2``
    linear-only Twist (the robot must move toward its Vicon subject +x after the psi0
    correction, design section 6).

Heading convention: the robot drives TAIL-FIRST, and that is correct
    The tracker's "forward" is the chassis tail: the model's heading, as psi0 puts it, is
    the tail direction, so ``linear.x > 0`` moves the robot tail-first and ``angular.z > 0``
    turns it CCW about the vertical.  A unicycle driven backwards is still a unicycle - the
    kinematics the tracker closes the loop on are unaffected, and runs recorded this way are
    valid.  Both direction signs therefore stay ``+1`` and the two H-bridge groups stay as
    wired (A = left, B = right).

    Do not "fix" this at the motor layer.  Flipping BOTH dir signs to -1 makes the robot run
    nose-first but inverts the turn sign as well, and exchanging the two H-bridge groups
    inverts the turn sign again: stacked, the two cancel and leave linear.x reversed with
    respect to the model, which is the configuration this note exists to prevent.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

log = logging.getLogger("osoyoo_base.motors")

try:  # pragma: no cover - hardware only
    import RPi.GPIO as GPIO  # type: ignore
    import Adafruit_PCA9685  # type: ignore

    HARDWARE_AVAILABLE = True
except ImportError:  # pragma: no cover - the PC path
    GPIO = None
    Adafruit_PCA9685 = None
    HARDWARE_AVAILABLE = False


@dataclass
class MotorParams:
    """Everything the base needs to turn a Twist into wheel PWM."""

    wheel_base_m: float = 0.12      # measure: 0.12 (ARGoS plugin) vs 0.15 (target_follow_v2) - pick one
    max_lin: float = 0.30           # m/s at cmd = 1 (calibrate, design section 7.2)
    max_ang: float = 2.5            # rad/s at cmd = (1, -1)
    min_pwm: int = 1200             # PWM at the stiction floor
    max_pwm: int = 4095
    deadband: int = 100             # legacy: PWM below (min_pwm - deadband) is treated as 0
    deadband_cmd: float = 0.03      # |cmd| below this -> wheel stopped
    left_dir_sign: int = 1
    right_dir_sign: int = 1
    pwm_freq_hz: int = 60
    # PCA9685 channels / BCM pins of the original wiring: group A = ena/in1-in2 drives the
    # LEFT wheel, group B = enb/in3-in4 the RIGHT one.  Do NOT exchange the two groups to
    # correct a turn sign: see the tail-first note in the module docstring.
    ena: int = 0
    enb: int = 1
    in1: int = 23
    in2: int = 24
    in3: int = 27
    in4: int = 22


class FakeHardware:
    """Records the PWM calls instead of driving GPIO (PC plumbing tests)."""

    def __init__(self, params: MotorParams):
        self.params = params
        self.calls: list[tuple[str, tuple]] = []
        self.left: tuple[int, int] = (0, 0)    # (signed direction, pwm)
        self.right: tuple[int, int] = (0, 0)
        self.stopped = True
        self.cleaned_up = False

    def set_wheel(self, side: str, direction: int, pwm: int) -> None:
        self.calls.append(("set_wheel", (side, direction, pwm)))
        if side == "left":
            self.left = (direction, pwm)
        else:
            self.right = (direction, pwm)
        self.stopped = self.left[1] == 0 and self.right[1] == 0
        log.info("FakeHardware %s dir=%+d pwm=%d", side, direction, pwm)

    def stop(self) -> None:
        self.calls.append(("stop", ()))
        self.left = (0, 0)
        self.right = (0, 0)
        self.stopped = True
        log.info("FakeHardware stop")

    def cleanup(self) -> None:
        self.stop()
        self.cleaned_up = True
        self.calls.append(("cleanup", ()))


class PiHardware:  # pragma: no cover - hardware only
    """The original PCA9685 + GPIO layer of target_follow_v2.py."""

    def __init__(self, params: MotorParams):
        self.params = params
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in (params.in1, params.in2, params.in3, params.in4):
            GPIO.setup(pin, GPIO.OUT)
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(params.pwm_freq_hz)
        self.stop()

    def set_wheel(self, side: str, direction: int, pwm: int) -> None:
        p = self.params
        if side == "left":
            enable, pin_a, pin_b = p.ena, p.in1, p.in2
        else:
            enable, pin_a, pin_b = p.enb, p.in3, p.in4
        if direction > 0:
            GPIO.output(pin_a, GPIO.HIGH)
            GPIO.output(pin_b, GPIO.LOW)
        elif direction < 0:
            GPIO.output(pin_a, GPIO.LOW)
            GPIO.output(pin_b, GPIO.HIGH)
        else:
            GPIO.output(pin_a, GPIO.LOW)
            GPIO.output(pin_b, GPIO.LOW)
        self.pwm.set_pwm(enable, 0, int(max(0, min(4095, pwm))))

    def stop(self) -> None:
        for pin in (self.params.in1, self.params.in2, self.params.in3, self.params.in4):
            GPIO.output(pin, GPIO.LOW)
        self.pwm.set_pwm(self.params.ena, 0, 0)
        self.pwm.set_pwm(self.params.enb, 0, 0)

    def cleanup(self) -> None:
        self.stop()
        GPIO.cleanup()


def setup(params: MotorParams | None = None, force_fake: bool = False):
    """Return the hardware object (PiHardware on the Pi, FakeHardware elsewhere)."""
    params = params or MotorParams()
    if HARDWARE_AVAILABLE and not force_fake:
        return PiHardware(params)
    if not force_fake:
        log.warning("RPi.GPIO / Adafruit_PCA9685 not importable: using FakeHardware")
    return FakeHardware(params)


def cmd_to_pwm(cmd: float, params: MotorParams) -> tuple[int, int]:
    """``(direction, pwm)`` for a wheel command in [-1, 1]."""
    cmd = max(-1.0, min(1.0, float(cmd)))
    if abs(cmd) < params.deadband_cmd:
        return 0, 0
    pwm = params.min_pwm + (params.max_pwm - params.min_pwm) * abs(cmd)
    pwm = int(round(pwm))
    if pwm < params.min_pwm - params.deadband:
        return 0, 0
    return (1 if cmd > 0 else -1), max(params.min_pwm, min(params.max_pwm, pwm))


def _apply_signed_pwm(hw, side: str, cmd: float, params: MotorParams, dir_sign: int) -> tuple[int, int]:
    direction, pwm = cmd_to_pwm(cmd, params)
    hw.set_wheel(side, direction * dir_sign, pwm)
    return direction * dir_sign, pwm


def stopcar(hw) -> None:
    hw.stop()


def diff_drive_mix(v: float, w: float, params: MotorParams) -> tuple[float, float]:
    """Twist (m/s, rad/s) -> normalised wheel commands ``(left, right)`` in [-1, 1].

    ``v_l = v - w * L / 2``, ``v_r = v + w * L / 2``; normalised by ``max_lin`` and scaled down
    together if either exceeds 1 so the turn ratio is preserved.
    """
    half = 0.5 * params.wheel_base_m
    v_l = v - w * half
    v_r = v + w * half
    scale = params.max_lin if params.max_lin > 0 else 1.0
    left, right = v_l / scale, v_r / scale
    biggest = max(abs(left), abs(right))
    if biggest > 1.0:
        left /= biggest
        right /= biggest
    return left, right


def custom_speed(hw, left: float, right: float, params: MotorParams) -> tuple[tuple[int, int], tuple[int, int]]:
    """Drive both wheels with normalised commands; returns the applied ``(dir, pwm)`` pairs."""
    l = _apply_signed_pwm(hw, "left", left, params, params.left_dir_sign)
    r = _apply_signed_pwm(hw, "right", right, params, params.right_dir_sign)
    return l, r
