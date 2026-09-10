# osoyoo_base

Actuation node for the Osoyoo robot (Raspberry Pi + PCA9685 + IN1..IN4 H-bridge): the motor
layer of `target_follow_v2.py` (`setup`, `_apply_signed_pwm`, `stopcar`, `diff_drive_mix`,
`custom_speed`) in `osoyoo_base/motors.py`, wrapped by `base_node.py`.

* subscribes `/osoyoo_4/cmd_vel` (`geometry_msgs/Twist`, `linear.x` m/s, `angular.z` rad/s,
  robot frame) and `/estop` (`std_msgs/Bool`, latching);
* `/osoyoo_4/wheel_cmd` (`std_msgs/Float64MultiArray [left, right]` in [-1, 1]) drives the
  wheels directly (used by `ra_embodied/tools/calib_motors.py`);
* watchdog: motors stop when no command has arrived for 0.3 s; `stopcar` + `GPIO.cleanup()`
  on shutdown.

Parameters (ROS): `wheel_base_m` (0.12 — measure; the old script used 0.15), `max_lin`,
`max_ang` (define the `cmd ∈ [-1, 1]` normalisation; calibrate, design section 7.2),
`min_pwm`, `max_pwm`, `deadband`, `deadband_cmd`, `left_dir_sign`, `right_dir_sign`,
`watchdog_s`, `fake_hardware`.

### The robot drives tail-first, and that is correct

The tracker's "forward" is the chassis **tail**: the model's heading, as `psi0` defines it,
is the tail direction. So `linear.x > 0` drives the robot tail-first and `angular.z > 0`
turns it CCW. A unicycle driven backwards is still a unicycle — the kinematics the tracker
closes the loop on are unaffected, and runs recorded this way are valid.

Keep **`left_dir_sign = right_dir_sign = +1`** and the H-bridge groups **as wired**
(A = `ENA`/`IN1`-`IN2` = left, B = `ENB`/`IN3`-`IN4` = right). Two corrections are
tempting here and they must not be stacked: flipping both dir signs to `-1` makes the robot
run nose-first but inverts the turn sign as well, and exchanging the two H-bridge groups
inverts the turn sign a second time — together they cancel and leave `linear.x` reversed
against the model, which is the one configuration that looks plausible on the bench and is
wrong in the arena.

The `cmd_vel` stream uses BEST_EFFORT / KEEP_LAST(1) QoS (`CMD_QOS` in `base_node.py`,
matched by `ra_embodied.qos.CMD_QOS`): reliable delivery of a 50 Hz sampled signal only
produced `Problem reserving CacheChange in reader` floods from Fast DDS. Both ends must
carry the same profile — a best-effort publisher does not match a reliable subscriber, so
**update the Pi before the PC** when rolling this out. `/estop` and `/wheel_cmd` stay
reliable.

Pi dependencies: `rclpy`, `geometry_msgs`, `std_msgs`, `RPi.GPIO`, `Adafruit_PCA9685`. No
Vicon, no numpy-heavy code. Without the GPIO modules the node runs with `FakeHardware`
(logs PWM values), which is what `test/test_plumbing.py` uses on the PC.

```bash
# on the Pi (same ROS_DOMAIN_ID as the PC)
ros2 run osoyoo_base base_node
ros2 param get /osoyoo_base left_dir_sign && ros2 param get /osoyoo_base right_dir_sign   # both must be 1
ros2 topic pub --once /osoyoo_4/cmd_vel geometry_msgs/Twist "{linear: {x: 0.15}}"   # tail-first, toward subject +x
ros2 topic pub --once /osoyoo_4/cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"   # must turn CCW
```
