#!/usr/bin/env python3
"""Flight-Review-style interactive HTML plots for a folder of PX4 .ulg files.

Scans a log directory, renders one interactive page per log (attitude, rates,
position, velocity and thrust, each against its setpoint, on a shared zoomable
time axis), and writes an index.html linking them all. Every page carries a
sidebar listing all logs, so you can move between runs without going back.

Step-response metrics (dead time, rise time, peak time, overshoot, settling
time) are computed for the commanded roll and pitch and printed at the top of
each page. They are computed on the FULL-RESOLUTION data, independently of the
downsampling applied to the plots.

Usage:
    ./generate_log_plots.py                 # render any logs not done yet
    ./generate_log_plots.py --watch         # keep watching for new .ulg files
    ./generate_log_plots.py --force         # re-render everything
    ./generate_log_plots.py --log-dir DIR --out-dir DIR
"""

import argparse
import html
import json
import os
import re
import shutil
import sys
import time
import traceback
from datetime import datetime

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pyulog import ULog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_step_response import analyze_axis  # noqa: E402  (same-directory tool)

DEFAULT_LOG_DIR = "/home/nikesh/Desktop/sitl log"
# Points per trace in the rendered page. The browser, not pyulog, is the
# bottleneck: a 150 MB log holds ~10^5 attitude samples per axis and plotting
# them raw makes the page crawl. Metrics never see this - they run on the full
# series - so this only ever costs visual detail when zoomed all the way in.
MAX_POINTS = 6000
WATCH_POLL_S = 5.0
# A file still being written must not be parsed. Two identical size readings
# this far apart mean the download has finished.
SIZE_SETTLE_S = 3.0

TOPICS = [
    "vehicle_attitude",
    "vehicle_attitude_setpoint",
    "vehicle_angular_velocity",
    "vehicle_rates_setpoint",
    "vehicle_local_position",
    "vehicle_local_position_setpoint",
    "vehicle_thrust_setpoint",
    "trajectory_setpoint",
]


def quaternion_to_euler_deg(q):
    """Roll, pitch, yaw in degrees from (w, x, y, z) columns, intrinsic Z-Y-X."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def downsample(t, y):
    if len(t) <= MAX_POINTS:
        return t, y
    stride = int(np.ceil(len(t) / MAX_POINTS))
    return t[::stride], y[::stride]


class LogData:
    """Everything one page needs, pulled out of a ULog."""

    def __init__(self, path):
        self.path = path
        self.ulog = ULog(path, TOPICS)
        self.t_ref = None
        self.series = {}
        self._load()

    def _dataset(self, name):
        try:
            return self.ulog.get_dataset(name)
        except (KeyError, IndexError):
            return None

    def _time(self, dataset):
        t = np.array(dataset.data["timestamp"], dtype=np.float64) * 1e-6
        if self.t_ref is None:
            self.t_ref = t[0]
        return t - self.t_ref

    def _load(self):
        att = self._dataset("vehicle_attitude")
        if att is not None:
            t = self._time(att)
            q = np.column_stack([np.array(att.data[f"q[{i}]"], dtype=np.float64) for i in range(4)])
            r, p, y = quaternion_to_euler_deg(q)
            self.series.update({"t_att": t, "roll": r, "pitch": p, "yaw": y})

        sp = self._dataset("vehicle_attitude_setpoint")
        if sp is not None:
            t = self._time(sp)
            q = np.column_stack([np.array(sp.data[f"q_d[{i}]"], dtype=np.float64) for i in range(4)])
            r, p, y = quaternion_to_euler_deg(q)
            self.series.update({"t_att_sp": t, "roll_sp": r, "pitch_sp": p, "yaw_sp": y})
            for i, axis in enumerate("xyz"):
                key = f"thrust_body[{i}]"
                if key in sp.data:
                    self.series[f"thrust_{axis}"] = np.array(sp.data[key], dtype=np.float64)
            self.series["t_thrust_sp"] = t

        rates = self._dataset("vehicle_angular_velocity")
        if rates is not None:
            t = self._time(rates)
            self.series["t_rate"] = t
            for i, axis in enumerate(("rollrate", "pitchrate", "yawrate")):
                key = f"xyz[{i}]"
                if key in rates.data:
                    self.series[axis] = np.degrees(np.array(rates.data[key], dtype=np.float64))

        rates_sp = self._dataset("vehicle_rates_setpoint")
        if rates_sp is not None:
            t = self._time(rates_sp)
            self.series["t_rate_sp"] = t
            for name, field in (("rollrate_sp", "roll"), ("pitchrate_sp", "pitch"), ("yawrate_sp", "yaw")):
                if field in rates_sp.data:
                    self.series[name] = np.degrees(np.array(rates_sp.data[field], dtype=np.float64))

        pos = self._dataset("vehicle_local_position")
        if pos is not None:
            t = self._time(pos)
            self.series["t_pos"] = t
            for field in ("x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az"):
                if field in pos.data:
                    self.series[field] = np.array(pos.data[field], dtype=np.float64)

        # Position/velocity setpoints live in different topics depending on the
        # build and the active mode; take whichever is present.
        for topic in ("vehicle_local_position_setpoint", "trajectory_setpoint"):
            sp_pos = self._dataset(topic)
            if sp_pos is None:
                continue
            t = self._time(sp_pos)
            self.series["t_pos_sp"] = t
            for i, axis in enumerate("xyz"):
                for src, dst in ((f"position[{i}]", f"{axis}_sp"), (f"velocity[{i}]", f"v{axis}_sp")):
                    if src in sp_pos.data:
                        self.series[dst] = np.array(sp_pos.data[src], dtype=np.float64)
                for src, dst in ((axis, f"{axis}_sp"), (f"v{axis}", f"v{axis}_sp")):
                    if src in sp_pos.data and dst not in self.series:
                        self.series[dst] = np.array(sp_pos.data[src], dtype=np.float64)
            break

        thrust = self._dataset("vehicle_thrust_setpoint")
        if thrust is not None and "thrust_z" not in self.series:
            t = self._time(thrust)
            self.series["t_thrust_sp"] = t
            for i, axis in enumerate("xyz"):
                key = f"xyz[{i}]"
                if key in thrust.data:
                    self.series[f"thrust_{axis}"] = np.array(thrust.data[key], dtype=np.float64)

    @property
    def duration_s(self):
        t = self.series.get("t_att")
        return float(t[-1] - t[0]) if t is not None and len(t) else 0.0

    def metrics(self):
        """Step-response metrics per axis, on full-resolution data.

        Reduced to plain scalars here: the raw arrays analyze_axis returns for
        plotting are megabytes per log, and these results get cached to disk so
        a re-run does not have to re-parse the ulog.
        """
        out = {}
        if not all(k in self.series for k in ("t_att_sp", "t_att")):
            return out
        for axis, sp_key, att_key in (("roll", "roll_sp", "roll"), ("pitch", "pitch_sp", "pitch")):
            if sp_key not in self.series or att_key not in self.series:
                continue
            try:
                result, reason = analyze_axis(
                    axis, self.series["t_att_sp"], self.series[sp_key],
                    self.series["t_att"], self.series[att_key],
                    settle_band_pct=2.0, window_s=10.0)
            except Exception as exc:  # a malformed/truncated log must not kill the page
                result, reason = None, f"{axis}: metric computation failed ({exc})"

            if result is None:
                out[axis] = {"ok": False, "reason": reason or "no step"}
            else:
                out[axis] = {"ok": True} | {
                    k: (None if result[k] is None else float(result[k]))
                    for k in ("t0", "from", "target", "amplitude", "delay", "rise",
                              "peak_time", "overshoot_pct", "settle", "final", "window")}
        return out


PLOT_ROWS = [
    ("Roll (deg)", [("t_att_sp", "roll_sp", "roll setpoint"), ("t_att", "roll", "roll")]),
    ("Pitch (deg)", [("t_att_sp", "pitch_sp", "pitch setpoint"), ("t_att", "pitch", "pitch")]),
    ("Yaw (deg)", [("t_att_sp", "yaw_sp", "yaw setpoint"), ("t_att", "yaw", "yaw")]),
    ("Roll rate (deg/s)", [("t_rate_sp", "rollrate_sp", "roll rate setpoint"),
                           ("t_rate", "rollrate", "roll rate")]),
    ("Pitch rate (deg/s)", [("t_rate_sp", "pitchrate_sp", "pitch rate setpoint"),
                            ("t_rate", "pitchrate", "pitch rate")]),
    ("Yaw rate (deg/s)", [("t_rate_sp", "yawrate_sp", "yaw rate setpoint"),
                          ("t_rate", "yawrate", "yaw rate")]),
    ("Position (m, NED)", [("t_pos", "x", "x"), ("t_pos", "y", "y"), ("t_pos", "z", "z (down)"),
                           ("t_pos_sp", "x_sp", "x setpoint"), ("t_pos_sp", "y_sp", "y setpoint"),
                           ("t_pos_sp", "z_sp", "z setpoint")]),
    ("Velocity (m/s)", [("t_pos", "vx", "vx"), ("t_pos", "vy", "vy"), ("t_pos", "vz", "vz"),
                        ("t_pos_sp", "vx_sp", "vx setpoint"), ("t_pos_sp", "vy_sp", "vy setpoint"),
                        ("t_pos_sp", "vz_sp", "vz setpoint")]),
    ("Acceleration (m/s^2)", [("t_pos", "ax", "ax"), ("t_pos", "ay", "ay"), ("t_pos", "az", "az")]),
    ("Thrust setpoint (normalized)", [("t_thrust_sp", "thrust_x", "thrust x"),
                                      ("t_thrust_sp", "thrust_y", "thrust y"),
                                      ("t_thrust_sp", "thrust_z", "thrust z")]),
]

PALETTE = ["#e4572e", "#2364aa", "#3da35d", "#9b59b6", "#e8a33d", "#17a2b8"]


def build_figure(log):
    rows = [(title, traces) for title, traces in PLOT_ROWS
            if any(t_key in log.series and v_key in log.series for t_key, v_key, _ in traces)]
    if not rows:
        return None

    fig = make_subplots(rows=len(rows), cols=1, shared_xaxes=True,
                        vertical_spacing=0.022, subplot_titles=[r[0] for r in rows])

    for row_idx, (_title, traces) in enumerate(rows, start=1):
        color_idx = 0
        # One legend per subplot rather than a single shared one. With ten rows
        # a shared legend runs to thirty-odd entries, none of which say which
        # plot they belong to - and clicking one then hides a trace in a plot
        # the reader is not looking at.
        legend_name = "legend" if row_idx == 1 else f"legend{row_idx}"
        for t_key, v_key, label in traces:
            if t_key not in log.series or v_key not in log.series:
                continue
            t, y = log.series[t_key], log.series[v_key]
            n = min(len(t), len(y))
            t, y = downsample(t[:n], y[:n])
            # Setpoints dashed, so a setpoint/actual pair reads at a glance even
            # when the two lie almost on top of each other.
            dashed = "setpoint" in label
            fig.add_trace(
                go.Scattergl(
                    x=t, y=y, name=label, mode="lines", legend=legend_name,
                    line=dict(width=1.4, dash="dot" if dashed else "solid",
                              color=PALETTE[color_idx % len(PALETTE)]),
                    hovertemplate=f"{label}: %{{y:.3f}}<br>t=%{{x:.3f}} s<extra></extra>",
                ),
                row=row_idx, col=1)
            color_idx += 1

    fig.update_layout(
        height=260 * len(rows), hovermode="x unified", showlegend=True,
        margin=dict(l=65, r=185, t=60, b=45), template="plotly_white",
        dragmode="zoom",
    )

    # Park each legend beside its own subplot, using the domain make_subplots
    # assigned to that row.
    for row_idx in range(1, len(rows) + 1):
        axis_key = "yaxis" if row_idx == 1 else f"yaxis{row_idx}"
        domain = fig.layout[axis_key].domain
        legend_name = "legend" if row_idx == 1 else f"legend{row_idx}"
        fig.update_layout(**{legend_name: dict(
            x=1.005, xanchor="left", y=domain[1], yanchor="top",
            font=dict(size=10), bgcolor="rgba(255,255,255,0.7)",
            borderwidth=0, itemsizing="constant")})
    fig.update_xaxes(title_text="time (s)", row=len(rows), col=1)
    fig.update_xaxes(showspikes=True, spikemode="across", spikethickness=1)
    for annotation in fig.layout.annotations:
        annotation.font.size = 13
    return fig


def metric_rows_html(metrics):
    if not metrics:
        return '<p class="muted">No attitude setpoint logged - step metrics unavailable.</p>'

    cells = []
    for axis in ("roll", "pitch"):
        result = metrics.get(axis)
        if result is None:
            continue
        if not result.get("ok"):
            cells.append(f'<tr><td class="axis">{axis}</td>'
                         f'<td colspan="7" class="muted">'
                         f'{html.escape(result.get("reason", "no step"))}</td></tr>')
            continue

        def f(v, nd=3):
            return "n/a" if v is None else f"{v:.{nd}f}"

        cells.append(
            f'<tr><td class="axis">{axis}</td>'
            f'<td>{result["from"]:.2f} &rarr; {result["target"]:.2f}&deg;</td>'
            f'<td>{result["t0"]:.2f} s</td>'
            f'<td>{f(result["delay"])} s</td>'
            f'<td class="hl">{f(result["rise"])} s</td>'
            f'<td class="hl">{f(result["peak_time"])} s</td>'
            f'<td>{result["overshoot_pct"]:.1f} %</td>'
            f'<td>{f(result["settle"]) + " s" if result["settle"] is not None else
                   "&gt; " + format(result.get("window") or 10.0, ".0f") + " s"}</td></tr>')

    if not cells:
        return '<p class="muted">No step detected on either axis.</p>'

    return f"""<table class="metrics">
<thead><tr><th>axis</th><th>step</th><th>at</th><th>dead time</th><th>rise time</th>
<th>peak time</th><th>overshoot</th><th>settling</th></tr></thead>
<tbody>{''.join(cells)}</tbody></table>
<p class="note">Rise time is 10%&rarr;90% of step amplitude; dead time is the step to 10%
(a large value usually means the airframe could not respond yet). Peak time is the first
peak past the target &mdash; blank when the response never overshoots. Settling uses a
&plusmn;2% band, measured over the first 10 s after the step. Computed on full-rate data,
not the downsampled traces plotted below.</p>"""


CSS = """
:root { --bg:#f6f7f9; --panel:#fff; --ink:#1b1f24; --muted:#6b7280; --line:#e3e6ea; --accent:#2364aa; }
* { box-sizing:border-box; }
body { margin:0; font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
       color:var(--ink); background:var(--bg); }
.layout { display:flex; min-height:100vh; }
.sidebar { width:280px; flex:0 0 280px; background:var(--panel); border-right:1px solid var(--line);
           padding:18px 0; position:sticky; top:0; height:100vh; overflow-y:auto; }
.sidebar h2 { font-size:12px; text-transform:uppercase; letter-spacing:.06em; color:var(--muted);
              margin:0 0 10px; padding:0 18px; }
.sidebar a { display:block; padding:9px 18px; color:var(--ink); text-decoration:none;
             border-left:3px solid transparent; font-size:13px; }
.sidebar a:hover { background:#f0f4f9; }
.sidebar a.active { border-left-color:var(--accent); background:#eef3fa; font-weight:600; }
.sidebar a .sub { display:block; color:var(--muted); font-size:11px; }
.main { flex:1; padding:26px 30px 60px; min-width:0; }
h1 { font-size:22px; margin:0 0 4px; }
.sub { color:var(--muted); font-size:13px; }
.card { background:var(--panel); border:1px solid var(--line); border-radius:8px;
        padding:18px 20px; margin:18px 0; }
table.metrics, table.index { width:100%; border-collapse:collapse; font-size:13px; }
table.metrics th, table.metrics td, table.index th, table.index td {
        text-align:left; padding:7px 10px; border-bottom:1px solid var(--line); }
table.metrics th, table.index th { color:var(--muted); font-weight:600; font-size:11px;
        text-transform:uppercase; letter-spacing:.04em; }
td.axis { font-weight:600; text-transform:capitalize; }
td.hl { font-weight:600; color:var(--accent); }
.muted { color:var(--muted); }
.note { color:var(--muted); font-size:12px; margin:10px 0 0; }
table.index tr:hover { background:#f7f9fc; }
table.index a { color:var(--accent); text-decoration:none; font-weight:600; }
.badge { display:inline-block; padding:2px 8px; border-radius:99px; font-size:11px;
         background:#eef3fa; color:var(--accent); }
.badge.none { background:#f0f1f3; color:var(--muted); }
"""


def sidebar_html(entries, current):
    links = []
    for e in entries:
        active = " active" if e["html_name"] == current else ""
        links.append(f'<a class="item{active}" href="{html.escape(e["html_name"])}">'
                     f'{html.escape(e["label"])}<span class="sub">{e["duration"]:.0f} s'
                     f'{" &middot; " + e["step_summary"] if e["step_summary"] else ""}</span></a>')
    index_active = " active" if current == "index.html" else ""
    return f"""<nav class="sidebar">
<h2>Logs</h2>
<a class="item{index_active}" href="index.html"><strong>&larr; All logs</strong></a>
{''.join(links)}
</nav>"""


def page_html(title, sidebar, body):
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(title)}</title><style>{CSS}</style></head>
<body><div class="layout">{sidebar}<main class="main">{body}</main></div></body></html>"""


def render_log_page(entry, entries, out_dir, frag_dir):
    """Assemble one page from its cached plot fragment.

    The figure is built once, when the log is first seen, and cached. Adding a
    new log rewrites every page's sidebar, and re-parsing a 150 MB ulog just to
    reprint a navigation list would make each new log cost minutes.
    """
    frag_path = os.path.join(frag_dir, entry["html_name"] + ".frag")
    try:
        with open(frag_path) as fh:
            plot_html = fh.read()
    except OSError:
        plot_html = '<p class="muted">Plot fragment missing - re-run with --force.</p>'

    body = f"""<h1>{html.escape(entry['label'])}</h1>
<p class="sub">{html.escape(entry['source_name'])} &middot; {entry['duration']:.1f} s &middot;
{entry['size_mb']:.1f} MB</p>
<div class="card"><h2 style="margin-top:0;font-size:15px">Step response &mdash; commanded roll and pitch</h2>
{metric_rows_html(entry['metrics'])}</div>
<div class="card">{plot_html}</div>"""

    out_path = os.path.join(out_dir, entry["html_name"])
    with open(out_path, "w") as fh:
        fh.write(page_html(entry["label"], sidebar_html(entries, entry["html_name"]), body))
    return out_path


def write_fragment(log, frag_path):
    fig = build_figure(log)
    if fig is None:
        frag = '<p class="muted">No plottable topics found in this log.</p>'
    else:
        frag = fig.to_html(full_html=False, include_plotlyjs="directory",
                           config={"displaylogo": False, "scrollZoom": True,
                                   "modeBarButtonsToRemove": ["lasso2d", "select2d"]})
    with open(frag_path, "w") as fh:
        fh.write(frag)


def render_index(entries, out_dir):
    rows = []
    for e in sorted(entries, key=lambda x: x["sort_key"], reverse=True):
        badge = (f'<span class="badge">{html.escape(e["step_summary"])}</span>'
                 if e["step_summary"] else '<span class="badge none">no step</span>')
        rise = e.get("rise_summary") or '<span class="muted">&ndash;</span>'
        rows.append(
            f'<tr><td><a href="{html.escape(e["html_name"])}">{html.escape(e["label"])}</a></td>'
            f'<td>{e["duration"]:.1f} s</td><td>{badge}</td><td>{rise}</td>'
            f'<td class="muted">{html.escape(e["source_name"])}</td></tr>')

    body = f"""<h1>PX4 SITL log plots</h1>
<p class="sub">{len(entries)} log{'s' if len(entries) != 1 else ''} &middot;
generated {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
<div class="card">
<table class="index">
<thead><tr><th>log</th><th>duration</th><th>commanded step</th>
<th>rise time (roll / pitch)</th><th>file</th></tr></thead>
<tbody>{''.join(rows) if rows else '<tr><td colspan="5" class="muted">No logs found.</td></tr>'}</tbody>
</table></div>"""

    out_path = os.path.join(out_dir, "index.html")
    with open(out_path, "w") as fh:
        fh.write(page_html("PX4 SITL log plots", sidebar_html(entries, "index.html"), body))
    return out_path


TIMESTAMP_RE = re.compile(r"(\d{4})-(\d{1,2})-(\d{1,2})-(\d{1,2})-(\d{1,2})-(\d{1,2})")


def log_identity(path):
    """Human label and output filename, from the log's own date/time."""
    base = os.path.basename(path)
    m = TIMESTAMP_RE.search(base)
    if m:
        y, mo, d, hh, mm, ss = (int(g) for g in m.groups())
        try:
            stamp = datetime(y, mo, d, hh, mm, ss)
        except ValueError:
            stamp = datetime.fromtimestamp(os.path.getmtime(path))
    else:
        stamp = datetime.fromtimestamp(os.path.getmtime(path))
    return stamp.strftime("%Y-%m-%d %H:%M:%S"), stamp.strftime("%Y-%m-%d_%H-%M-%S") + ".html", stamp


def summarize_metrics(metrics):
    """Short step description and rise times, for the index table."""
    steps, rises = [], []
    for axis in ("roll", "pitch"):
        result = metrics.get(axis)
        if result is None or not result.get("ok"):
            continue
        steps.append(f"{axis} {result['target']:.0f}°")
        rises.append(f"{axis[0]}: {result['rise']:.3f} s" if result["rise"] is not None
                     else f"{axis[0]}: n/a")
    return ", ".join(steps), " &middot; ".join(rises)


def wait_until_stable(path):
    """Do not parse a file that is still being written."""
    last = -1
    while True:
        try:
            size = os.path.getsize(path)
        except OSError:
            return False
        if size == last and size > 0:
            return True
        last = size
        time.sleep(SIZE_SETTLE_S)


def ensure_plotlyjs(out_dir):
    import plotly
    src = os.path.join(os.path.dirname(plotly.__file__), "package_data", "plotly.min.js")
    dst = os.path.join(out_dir, "plotly.min.js")
    if os.path.exists(src) and not os.path.exists(dst):
        shutil.copyfile(src, dst)


def build_entry(path, frag_dir):
    label, html_name, stamp = log_identity(path)
    log = LogData(path)
    metrics = log.metrics()
    write_fragment(log, os.path.join(frag_dir, html_name + ".frag"))
    step_summary, rise_summary = summarize_metrics(metrics)
    entry = {
        "path": path,
        "source_name": os.path.basename(path),
        "label": label,
        "html_name": html_name,
        "sort_key": stamp,
        "duration": log.duration_s,
        "size_mb": os.path.getsize(path) / 1e6,
        "metrics": metrics,
        "step_summary": step_summary,
        "rise_summary": rise_summary,
    }
    with open(os.path.join(frag_dir, html_name + ".json"), "w") as fh:
        json.dump({**entry, "sort_key": stamp.isoformat()}, fh)
    return entry


def load_cached_entry(path, frag_dir):
    """Reuse a previous run's parse if both cache files are newer than the log."""
    _label, html_name, _stamp = log_identity(path)
    meta_path = os.path.join(frag_dir, html_name + ".json")
    frag_path = os.path.join(frag_dir, html_name + ".frag")
    try:
        if min(os.path.getmtime(meta_path), os.path.getmtime(frag_path)) < os.path.getmtime(path):
            return None
        with open(meta_path) as fh:
            entry = json.load(fh)
    except (OSError, ValueError):
        return None
    entry["sort_key"] = datetime.fromisoformat(entry["sort_key"])
    entry["path"] = path
    return entry


def run_once(log_dir, out_dir, force, state):
    os.makedirs(out_dir, exist_ok=True)
    frag_dir = os.path.join(out_dir, ".cache")
    os.makedirs(frag_dir, exist_ok=True)
    ensure_plotlyjs(out_dir)

    paths = sorted(p for p in (os.path.join(log_dir, f) for f in os.listdir(log_dir))
                   if p.endswith(".ulg") and os.path.isfile(p))

    changed = False
    for path in paths:
        key = os.path.basename(path)
        if key in state and not force:
            continue

        if not force:
            cached = load_cached_entry(path, frag_dir)
            if cached is not None:
                state[key] = cached
                changed = True
                continue

        if not wait_until_stable(path):
            continue

        print(f"  parsing {key} ({os.path.getsize(path) / 1e6:.0f} MB) ...", flush=True)
        try:
            state[key] = build_entry(path, frag_dir)
        except Exception:
            print(f"  FAILED on {key}:", flush=True)
            traceback.print_exc()
            continue
        changed = True

    # Drop entries whose .ulg has gone away, so the index never links a 404.
    for key in [k for k in state if not os.path.exists(os.path.join(log_dir, k))]:
        del state[key]
        changed = True

    if not changed:
        return False

    # Every page is rewritten whenever the set of logs changes, because each
    # one carries the sidebar listing all of them. Cheap: the plot itself comes
    # from the cached fragment.
    entries = sorted(state.values(), key=lambda e: e["sort_key"], reverse=True)
    for entry in entries:
        try:
            render_log_page(entry, entries, out_dir, frag_dir)
        except Exception:
            print(f"  FAILED rendering page for {entry['source_name']}:", flush=True)
            traceback.print_exc()
    index_path = render_index(entries, out_dir)
    print(f"  index: {index_path}", flush=True)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log-dir", default=DEFAULT_LOG_DIR, help="folder holding the .ulg files")
    parser.add_argument("--out-dir", default=None,
                        help="output folder (default: <log-dir>/plots)")
    parser.add_argument("--force", action="store_true", help="re-render every log")
    parser.add_argument("--watch", action="store_true",
                        help="keep running and render new .ulg files as they appear")
    args = parser.parse_args()

    log_dir = os.path.abspath(os.path.expanduser(args.log_dir))
    out_dir = os.path.abspath(os.path.expanduser(args.out_dir)) if args.out_dir \
        else os.path.join(log_dir, "plots")

    if not os.path.isdir(log_dir):
        print(f"log directory not found: {log_dir}", file=sys.stderr)
        return 1

    print(f"log dir: {log_dir}\nout dir: {out_dir}")
    state = {}
    run_once(log_dir, out_dir, args.force, state)

    if not args.watch:
        return 0

    print(f"watching for new .ulg files (poll {WATCH_POLL_S:.0f}s) - Ctrl-C to stop", flush=True)
    try:
        while True:
            time.sleep(WATCH_POLL_S)
            if run_once(log_dir, out_dir, False, state):
                print(f"  updated {datetime.now().strftime('%H:%M:%S')}", flush=True)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
