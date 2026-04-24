# MyActuator RMD X-series Hardware

Author: [Tobit Flatscher](https://github.com/2b-t) (2024)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)



## Overview
This package holds the [**`ros2_control` integration**](https://control.ros.org/humble/index.html) for the [**MyActuator RMD-X actuator series**](https://www.myactuator.com/rmd-x) in the form of a [hardware component](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html). The hardware interface is based on the [C++ driver that I have written for these actuators](https://github.com/2b-t/myactuator_rmd).

For using it add the following lines to your URDF refering to the joint of interest `joint_name`:

```xml
<ros2_control name="${some_name}" type="actuator">
  <hardware>
    <plugin>myactuator_rmd_hardware/MyActuatorRmdHardwareInterface</plugin>
    <param name="ifname">${ifname}</param>
    <param name="actuator_id">${actuator_id}</param>
    <param name="torque_constant">${torque_constant}</param>
    <!-- Optional: Low-pass filters for velocity and effort (0 < alpha <= 1); defaults to no filter -->
    <param name="velocity_alpha">0.1</param>
    <param name="effort_alpha">0.1</param>
    <!-- Optional: Cycle time of the asynchronous thread in ms; defaults to 2ms (500Hz) -->
    <param name="cycle_time">10</param>
    <!-- Optional: Maximum velocity in dps for position commands; defaults to 720 -->
    <param name="max_velocity">240</param>
    <!-- Optional: CAN communication timeout in ms; 0 disables (default) -->
    <param name="timeout">100</param>
    <!-- Optional: Position planning acceleration/deceleration in dps/s; if omitted, motor keeps its current value -->
    <param name="position_acceleration">10000</param>
    <!-- Optional: Headroom multiplier applied to the per-cycle commanded velocity when sizing max_speed on position setpoints; <=0 or omitted disables dynamic scaling and max_velocity is sent verbatim -->
    <param name="velocity_headroom">1.2</param>
    <!-- Optional: Headroom multiplier applied to the position-error velocity; <=0 or omitted disables catch-up (max_speed is driven only by commanded velocity) -->
    <param name="catchup_velocity_headroom">0.5</param>
  </hardware>
  <joint name="${joint_name}">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

The `ifname` has to correspond to the name of the CAN interface as shown by `$ ifconfig` (e.g. `can0`) and the `actuator_id` to the ID of the actuator (e.g. `1`). The `torque_constant` is required for controlling the actuator over its effort interface and depends on the actuator type. Furthermore optional [low-pass filters](https://en.wikipedia.org/wiki/Low-pass_filter) (by means of the filter coefficient `alpha`) for the read velocity and effort can be activated. The correlation between the ratio of [sample](https://en.wikipedia.org/wiki/Sampling_(signal_processing)) (in our case the update rate of the hardware interface) and [cut-off frequency](https://en.wikipedia.org/wiki/Cutoff_frequency) is given by `sample_frequency/cutoff_frequency = (1-alpha)*2*pi/alpha`. For `alpha = 0.07` this ratio corresponds to `125`, meaning if the hardware interface is running at 1000 Hz any oscillations with a higher frequency than around 8 Hz will be filtered out.

| Without low pass filter (corresponds to `alpha = 1.0`)       | With low pass filter (`alpha = 0.07`)                        |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![Effort and velocity without low-pass filter](./media/without_low_pass_filter.png) | ![Effort and velocity with low-pass filter](./media/with_low_pass_filter.png) |

Similarly the cycle-time for the asynchronous thread interfacing the actuator through CAN can be specified. For examples refer to the `myactuator_rmd_description` package.

### Dynamic velocity scaling

By default, each call to `sendPositionAbsoluteSetpoint` sends `max_velocity` as the motor's `max_speed` argument. The motor's internal trapezoidal planner then decelerates to a stop at every intermediate waypoint, which produces jerky motion when streaming trajectory points.

The optional `velocity_headroom` and `catchup_velocity_headroom` parameters size `max_speed` dynamically from the per-cycle desired velocity, keeping the planner in its acceleration phase:

- `velocity_headroom` (double, default disabled): multiplier on the commanded trajectory velocity, estimated from consecutive position commands as `|pos_cmd - prev_pos_cmd| / cycle_time`. `1.2` = 20% headroom (a reasonable starting point). `0` or omitted disables this contribution entirely.
- `catchup_velocity_headroom` (double, default disabled): multiplier on the position-error velocity, `|pos_cmd - position_state| / cycle_time`. Allows the motor to catch up after lagging. `0` or omitted disables catch-up. When both are set, `max_speed` is the max of the two scaled values, clamped by `max_velocity`.

With both parameters omitted, `max_speed` equals `max_velocity` on every position setpoint (identical to upstream behavior). If `position_acceleration` is set to `0`, the motor bypasses its internal trapezoidal planner and `max_speed` acts purely as a velocity clamp for the internal PI tracker.

