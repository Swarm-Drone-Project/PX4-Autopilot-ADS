# Custom MC Rate Control Module

## Overview

`custom_mc_rate_control` is a minimal, direct torque-vectoring rate controller designed for aggressive flight maneuvers such as flips and acrobatic maneuvers.  It pairs with a custom attitude controller (e.g., `custom_mc_att_control`) to provide full control-stack customization.

## Features

- **Minimal overhead** – no yaw low-pass filtering or saturation logic, allowing raw torque output.
- **Direct PID** – uses the shared `RateControl` library so you can reuse existing `MC_*RATE_*` parameters.
- **Scheduled on gyro** – callback-triggered execution on `vehicle_angular_velocity` at ~1 kHz.
- **Passthrough thrust** – thrust setpoints from your attitude controller flow through unmodified, enabling negative Z-thrust for inverted maneuvers in simulation.

## Building

Enable the module in your board configuration (e.g., `antidrone.px4board`):

```plaintext
CONFIG_MODULES_CUSTOM_MC_RATE_CONTROL=y
```

Rebuild:

```bash
make px4_sitl_antidrone
```

## Usage

Start the module after (or instead of) the default `mc_rate_control`:

```bash
custom_mc_rate_control start
```

Use `status` to check runtime state:

```bash
custom_mc_rate_control status
```

Stop:

```bash
custom_mc_rate_control stop
```

## Input / Output Topics

| Direction | Topic                       | Purpose                                     |
|-----------|-----------------------------|---------------------------------------------|
| In        | `vehicle_rates_setpoint`    | Desired body rates (rad/s) and body thrust. |
| In        | `vehicle_angular_velocity`  | Measured gyro rates and derivative.         |
| In        | `vehicle_control_mode`      | Control mode flags (rate enable, etc.)      |
| Out       | `vehicle_torque_setpoint`   | Commanded normalized torque.                |
| Out       | `vehicle_thrust_setpoint`   | Commanded normalized thrust.                |
| Out       | `rate_ctrl_status`          | Integrator / PID telemetry.                 |

## Parameters

The module re-uses standard multicopter rate-loop parameters:

| Param           | Description                    |
|-----------------|--------------------------------|
| `MC_ROLLRATE_P` | Roll rate proportional gain    |
| `MC_ROLLRATE_I` | Roll rate integral gain        |
| `MC_ROLLRATE_D` | Roll rate derivative gain      |
| `MC_ROLLRATE_K` | Roll rate overall gain scaling |
| `MC_ROLLRATE_FF`| Roll rate feed-forward         |
| `MC_RR_INT_LIM` | Roll rate integrator limit     |
| …               | (similarly for pitch / yaw)    |

## Integration with Custom Attitude Controller

1. Your attitude controller publishes `vehicle_rates_setpoint` (plus thrust).
2. `custom_mc_rate_control` subscribes and outputs torque/thrust setpoints.
3. The control allocator (`control_allocator`) converts to motor outputs.

## Extending for Arbitrary Torque Injection

To command raw torques directly (bypassing PID), you can:

1. Modify `Run()` to check a flag or topic.
2. Override the PID output with arbitrary torques from an external topic.
3. Publish the modified `vehicle_torque_setpoint`.

Example skeleton in `Run()`:

```cpp
if (_direct_torque_mode) {
    direct_torque_setpoint_s cmd{};
    if (_direct_torque_sub.update(&cmd)) {
        torque_setpoint = Vector3f(cmd.torque);
    }
}
```
