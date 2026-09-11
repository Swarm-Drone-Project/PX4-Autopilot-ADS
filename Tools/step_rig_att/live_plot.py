#!/usr/bin/env python3
"""Live telemetry plot for step_rig_att runs.

Connects to a running SITL (or HITL) MAVLink stream and plots, in real time:
position, velocity + acceleration, attitude setpoint vs attitude, and rate
setpoint vs rate. This is a monitoring view - it is deliberately not the source
of the step-response numbers, because a plot fed by a lossy UDP stream at
whatever rate the link happens to deliver is the wrong instrument for measuring
a rise time. Use analyze_step_response.py on the .ulg for that.

Usage:
    ./live_plot.py                          # SITL instance 0
    ./live_plot.py --url udpin:0.0.0.0:14541  # SITL instance 1
    ./live_plot.py --window 20              # 20 s rolling window
"""

import argparse
import math
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pymavlink import mavutil

# Messages we need, and the rate to ask for. 50 Hz is comfortably finer than
# the transient being watched without swamping the link.
WANTED_MESSAGES = {
    "ATTITUDE": 30,
    "ATTITUDE_TARGET": 83,
    "LOCAL_POSITION_NED": 32,
    "HIGHRES_IMU": 105,
}
STREAM_RATE_HZ = 50


def quaternion_to_roll_pitch(q):
    """Roll and pitch in degrees from a (w, x, y, z) quaternion.

    Intrinsic Z-Y-X, matching PX4's Eulerf so the plotted setpoint is directly
    comparable to ATTITUDE's own roll/pitch.
    """
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    return math.degrees(roll), math.degrees(pitch)


class Telemetry:
    """Rolling buffers fed by a background MAVLink reader thread."""

    def __init__(self, window_s):
        self.window_s = window_s
        self.lock = threading.Lock()
        self.t0 = None
        self.series = {
            name: deque()
            for name in (
                "t_pos", "x", "y", "z",
                "t_vel", "vx", "vy", "vz",
                "t_acc", "ax", "ay", "az",
                "t_att", "roll", "pitch", "rollspeed", "pitchspeed", "yawspeed",
                "t_sp", "roll_sp", "pitch_sp", "rollrate_sp", "pitchrate_sp", "yawrate_sp",
            )
        }

    def _stamp(self, t_us):
        t = t_us * 1e-6
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def _trim(self, time_key, *value_keys):
        times = self.series[time_key]
        while times and (times[-1] - times[0]) > self.window_s:
            times.popleft()
            for k in value_keys:
                if self.series[k]:
                    self.series[k].popleft()

    def handle(self, msg):
        kind = msg.get_type()
        with self.lock:
            if kind == "ATTITUDE":
                t = self._stamp(msg.time_boot_ms * 1000)
                self.series["t_att"].append(t)
                self.series["roll"].append(math.degrees(msg.roll))
                self.series["pitch"].append(math.degrees(msg.pitch))
                self.series["rollspeed"].append(math.degrees(msg.rollspeed))
                self.series["pitchspeed"].append(math.degrees(msg.pitchspeed))
                self.series["yawspeed"].append(math.degrees(msg.yawspeed))
                self._trim("t_att", "roll", "pitch", "rollspeed", "pitchspeed", "yawspeed")

            elif kind == "ATTITUDE_TARGET":
                t = self._stamp(msg.time_boot_ms * 1000)
                roll_sp, pitch_sp = quaternion_to_roll_pitch(msg.q)
                self.series["t_sp"].append(t)
                self.series["roll_sp"].append(roll_sp)
                self.series["pitch_sp"].append(pitch_sp)
                self.series["rollrate_sp"].append(math.degrees(msg.body_roll_rate))
                self.series["pitchrate_sp"].append(math.degrees(msg.body_pitch_rate))
                self.series["yawrate_sp"].append(math.degrees(msg.body_yaw_rate))
                self._trim("t_sp", "roll_sp", "pitch_sp", "rollrate_sp", "pitchrate_sp", "yawrate_sp")

            elif kind == "LOCAL_POSITION_NED":
                t = self._stamp(msg.time_boot_ms * 1000)
                self.series["t_pos"].append(t)
                self.series["x"].append(msg.x)
                self.series["y"].append(msg.y)
                self.series["z"].append(msg.z)
                self._trim("t_pos", "x", "y", "z")

                self.series["t_vel"].append(t)
                self.series["vx"].append(msg.vx)
                self.series["vy"].append(msg.vy)
                self.series["vz"].append(msg.vz)
                self._trim("t_vel", "vx", "vy", "vz")

            elif kind == "HIGHRES_IMU":
                # Body-frame accelerometer rather than a derivative of the EKF
                # velocity: differentiating that amplifies estimator noise and
                # adds a filter lag, which on a 1-2 s transient is most of what
                # you would be looking at.
                t = self._stamp(msg.time_usec)
                self.series["t_acc"].append(t)
                self.series["ax"].append(msg.xacc)
                self.series["ay"].append(msg.yacc)
                self.series["az"].append(msg.zacc)
                self._trim("t_acc", "ax", "ay", "az")

    def snapshot(self):
        with self.lock:
            return {k: list(v) for k, v in self.series.items()}


def reader_loop(conn, telemetry):
    while True:
        msg = conn.recv_match(type=list(WANTED_MESSAGES), blocking=True)
        if msg is not None:
            telemetry.handle(msg)


def request_streams(conn):
    for name, msg_id in WANTED_MESSAGES.items():
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / STREAM_RATE_HZ), 0, 0, 0, 0, 0,
        )
        print(f"requested {name} at {STREAM_RATE_HZ} Hz")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="udpin:0.0.0.0:14540",
                        help="MAVLink endpoint (default: SITL instance 0)")
    parser.add_argument("--window", type=float, default=30.0,
                        help="rolling window in seconds (default: 30)")
    args = parser.parse_args()

    print(f"connecting to {args.url} ...")
    conn = mavutil.mavlink_connection(args.url)
    conn.wait_heartbeat()
    print(f"heartbeat from system {conn.target_system} component {conn.target_component}")
    request_streams(conn)

    telemetry = Telemetry(args.window)
    threading.Thread(target=reader_loop, args=(conn, telemetry), daemon=True).start()

    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    fig.suptitle("step_rig_att live telemetry")

    specs = [
        (axes[0], "Position (m, NED)", [
            ("t_pos", "x", "x"), ("t_pos", "y", "y"), ("t_pos", "z", "z (down)")]),
        (axes[1], "Velocity (m/s) and body accel (m/s^2)", [
            ("t_vel", "vx", "vx"), ("t_vel", "vy", "vy"), ("t_vel", "vz", "vz"),
            ("t_acc", "ax", "ax"), ("t_acc", "ay", "ay"), ("t_acc", "az", "az")]),
        (axes[2], "Attitude: setpoint vs actual (deg)", [
            ("t_sp", "roll_sp", "roll sp"), ("t_att", "roll", "roll"),
            ("t_sp", "pitch_sp", "pitch sp"), ("t_att", "pitch", "pitch")]),
        (axes[3], "Body rates: setpoint vs actual (deg/s)", [
            ("t_sp", "rollrate_sp", "roll rate sp"), ("t_att", "rollspeed", "roll rate"),
            ("t_sp", "pitchrate_sp", "pitch rate sp"), ("t_att", "pitchspeed", "pitch rate"),
            ("t_sp", "yawrate_sp", "yaw rate sp"), ("t_att", "yawspeed", "yaw rate")]),
    ]

    lines = []
    for ax, title, entries in specs:
        ax.set_title(title, fontsize=10)
        ax.grid(True, alpha=0.3)
        for t_key, v_key, label in entries:
            # Setpoint traces dashed so a setpoint/actual pair reads at a glance.
            style = "--" if "sp" in label else "-"
            (line,) = ax.plot([], [], style, linewidth=1.2, label=label)
            lines.append((line, t_key, v_key))
        ax.legend(loc="upper left", fontsize=7, ncol=3)
    axes[-1].set_xlabel("time since first sample (s)")

    def update(_frame):
        data = telemetry.snapshot()
        for line, t_key, v_key in lines:
            line.set_data(data[t_key], data[v_key])
        for ax, _title, _entries in specs:
            ax.relim()
            ax.autoscale_view()
        return [line for line, _, _ in lines]

    _anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
