#!/usr/bin/env python3
import argparse
import math
import sys
from collections import deque

import matplotlib.pyplot as plt
import serial

DEFAULT_LABELS = {
    "PLOT_G": "va_x[u],va_y[u],va_z[u],va_hat_x[u],va_hat_y[u],va_hat_z[u]",
    "PLOT_BIAS": "bias_hat_x[arb],bias_hat_y[arb],bias_hat_z[arb],i_corr_x[rad/s],i_corr_y[rad/s],i_corr_z[rad/s]",
    "PLOT_Q": "q_w[u],q_x[u],q_y[u],q_z[u],q_norm[u]",
    "PLOT_RPY": "roll[deg],pitch[deg],yaw[deg]",
    "PLOT_SENS": "accel_x[g],accel_y[g],accel_z[g],gyro_x[rad/s],gyro_y[rad/s],gyro_z[rad/s]",
    "PLOT_ERR": "err_angle_deg[deg],omega_mes_x[arb],omega_mes_y[arb],omega_mes_z[arb]",
    "PLOT": "err_angle_deg[deg],accel_mag[g],gyro_norm[rad/s],p_corr_norm[rad/s],i_corr_norm[rad/s],q_norm",
}

RAD_TO_DEG = 57.29577951308232


def parse_header(line):
    if not line.startswith("PLOT"):
        return None
    if "," not in line:
        return None
    head, rest = line.split(",", 1)
    if head == "PLOT_HEADER":
        prefix = "PLOT"
    elif head.startswith("PLOT_") and head.endswith("_HEADER"):
        prefix = head[: -len("_HEADER")]
    else:
        return None
    labels = rest.split(",") if rest else []
    return prefix, labels


def parse_values(line):
    if "," not in line:
        return None
    head, rest = line.split(",", 1)
    if head == "PLOT_HEADER" or head.endswith("_HEADER"):
        return None
    if not head.startswith("PLOT"):
        return None
    parts = rest.split(",")
    if not parts or parts == [""]:
        return None
    try:
        values = [float(p) for p in parts]
    except ValueError:
        return None
    return head, values


def split_label_unit(label):
    label = label.strip()
    for open_ch, close_ch in (("(", ")"), ("[", "]")):
        if close_ch in label:
            idx = label.rfind(open_ch)
            if idx != -1 and label.endswith(close_ch):
                name = label[:idx].strip()
                unit = label[idx + 1 : -1].strip()
                if name and unit:
                    return name, unit
    if ":" in label:
        name, unit = label.split(":", 1)
        name = name.strip()
        unit = unit.strip()
        if name and unit:
            return name, unit
    return label, ""


def build_labels_and_units(raw_labels, units_override):
    names = []
    units = []
    for lbl in raw_labels:
        name, unit = split_label_unit(lbl)
        names.append(name)
        units.append(unit)
    if units_override:
        override = [u.strip() for u in units_override.split(",")]
        if len(override) == len(names):
            units = override
    return names, units


class Stream:
    def __init__(self, prefix, maxlen):
        self.prefix = prefix
        self.maxlen = maxlen
        self.labels = []
        self.label_names = []
        self.label_units = []
        self.data = []
        self.lines = []
        self.ax = None
        self.value_text = None
        self.background = None
        self.last_values = None
        self.dirty = False

    def set_labels(self, labels, units_override=""):
        self.labels = labels
        self.label_names, self.label_units = build_labels_and_units(labels, units_override)
        self.data = [deque(maxlen=self.maxlen) for _ in self.label_names]
        self.last_values = None
        self.dirty = True
        self.background = None


class SerialLineBuffer:
    def __init__(self, ser):
        self.ser = ser
        self.buffer = ""

    def read_lines(self):
        count = self.ser.in_waiting
        chunk = self.ser.read(count if count > 0 else 1)
        if not chunk:
            return []
        self.buffer += chunk.decode(errors="ignore")
        lines = self.buffer.splitlines()
        if not self.buffer.endswith(("\n", "\r")) and lines:
            self.buffer = lines.pop()
        else:
            self.buffer = ""
        return [line.strip() for line in lines if line.strip()]


class PgStream:
    def __init__(self, prefix, maxlen):
        self.prefix = prefix
        self.maxlen = maxlen
        self.labels = []
        self.label_names = []
        self.label_units = []
        self.data = []
        self.plot = None
        self.curves = []
        self.value_text = None
        self.last_values = None
        self.dirty = False

    def set_labels(self, labels, units_override=""):
        self.labels = labels
        self.label_names, self.label_units = build_labels_and_units(labels, units_override)
        self.data = [deque(maxlen=self.maxlen) for _ in self.label_names]
        self.last_values = None
        self.dirty = True


def is_allowed(prefix, allowed_prefixes):
    if allowed_prefixes is None:
        return True
    return prefix in allowed_prefixes


def rebuild_layout(fig, streams, order):
    fig.clf()
    count = len(order)
    if count == 0:
        fig.canvas.draw()
        return
    cols = 2 if count > 1 else 1
    rows = (count + cols - 1) // cols
    axes = fig.subplots(rows, cols)
    if hasattr(axes, "flatten"):
        axes_list = list(axes.flatten())
    else:
        axes_list = [axes]
    for idx, prefix in enumserate(order):
        stream = streams[prefix]
        ax = axes_list[idx]
        stream.ax = ax
        ax.set_title(prefix)
        ax.set_xlabel("Sample")
        ax.set_ylabel("Value")
        x_max = stream.maxlen - 1 if stream.maxlen > 1 else 1
        ax.set_xlim(0, x_max)
        stream.lines = [ax.plot([], [], label=lbl)[0] for lbl in stream.label_names]
        for line in stream.lines:
            line.set_animated(True)
        if stream.label_names:
            ax.legend(loc="upper right")
        stream.value_text = ax.text(
            0.02,
            0.98,
            "",
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.7),
        )
        stream.value_text.set_animated(True)
        stream.background = None
        stream.dirty = True
    for ax in axes_list[count:]:
        ax.axis("off")
    fig.tight_layout()


def override_prefix(args):
    if args.prefixes:
        items = [p.strip() for p in args.prefixes.split(",") if p.strip()]
        if len(items) == 1:
            return items[0]
        return None
    if args.prefix:
        return args.prefix
    return "PLOT"


def labels_override_for(prefix, args, count):
    if args.labels and prefix == override_prefix(args):
        labels = [p.strip() for p in args.labels.split(",") if p.strip()]
        if len(labels) == count:
            return labels
    if prefix in DEFAULT_LABELS:
        labels = [p.strip() for p in DEFAULT_LABELS[prefix].split(",") if p.strip()]
        if len(labels) == count:
            return labels
    return [f"v{i}" for i in range(count)]


def units_override_for(prefix, args):
    if args.units and prefix == override_prefix(args):
        return args.units
    return ""


def compute_rpy_deg(values):
    if len(values) < 4:
        return None
    w, x, y, z = values[:4]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if sinp >= 1.0:
        pitch = math.pi / 2.0
    elif sinp <= -1.0:
        pitch = -math.pi / 2.0
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG]


def capture_backgrounds(fig, streams, order):
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(fig.bbox)
    for prefix in order:
        stream = streams[prefix]
        stream.background = background
    return background


def maybe_expand_ylim(stream, margin=0.05):
    if stream.ax is None or not stream.data:
        return False
    y_min = None
    y_max = None
    for series in stream.data:
        if not series:
            continue
        s_min = min(series)
        s_max = max(series)
        if y_min is None or s_min < y_min:
            y_min = s_min
        if y_max is None or s_max > y_max:
            y_max = s_max
    if y_min is None or y_max is None:
        return False
    if y_min == y_max:
        y_min -= 1.0
        y_max += 1.0
    cur_min, cur_max = stream.ax.get_ylim()
    if y_min < cur_min or y_max > cur_max:
        span = y_max - y_min
        pad = span * margin
        stream.ax.set_ylim(y_min - pad, y_max + pad)
        return True
    return False


def ensure_stream(prefix, values, args, streams, order):
    layout_changed = False
    stream = streams.get(prefix)
    if stream is None:
        stream = Stream(prefix, args.window)
        streams[prefix] = stream
        order.append(prefix)
        layout_changed = True
    if not stream.data or len(stream.data) != len(values):
        labels = labels_override_for(prefix, args, len(values))
        stream.set_labels(labels, units_override_for(prefix, args))
        layout_changed = True
    return stream, layout_changed


def update_stream_data(stream, values):
    for series, v in zip(stream.data, values):
        series.append(v)
    stream.last_values = values
    stream.dirty = True


def compute_stream_range(stream):
    y_min = None
    y_max = None
    for series in stream.data:
        if not series:
            continue
        s_min = min(series)
        s_max = max(series)
        if y_min is None or s_min < y_min:
            y_min = s_min
        if y_max is None or s_max > y_max:
            y_max = s_max
    if y_min is None or y_max is None:
        return None
    if y_min == y_max:
        y_min -= 1.0
        y_max += 1.0
    span = y_max - y_min
    pad = span * 0.05 if span > 0 else 1.0
    return y_min, y_max, pad


def run_pyqtgraph(args):
    try:
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore
    except Exception as exc:
        print(f"PyQtGraph backend not available: {exc}", file=sys.stderr)
        return 1

    if args.prefixes:
        allowed_prefixes = {p.strip() for p in args.prefixes.split(",") if p.strip()}
    elif args.prefix:
        allowed_prefixes = {args.prefix}
    else:
        allowed_prefixes = None

    ser = serial.Serial(args.port, args.baud, timeout=0)
    reader = SerialLineBuffer(ser)

    pg.setConfigOptions(antialias=True)
    app = pg.mkQApp("Serial Plotter")
    win = pg.GraphicsLayoutWidget(title="Serial Plotter")
    win.show()

    streams = {}
    order = []
    layout_dirty = False

    def rebuild_layout_pg():
        win.clear()
        count = len(order)
        cols = 2 if count > 1 else 1
        for idx, prefix in enumerate(order):
            stream = streams[prefix]
            row = idx // cols
            col = idx % cols
            plot = win.addPlot(row=row, col=col, title=prefix)
            plot.showGrid(x=True, y=True, alpha=0.2)
            x_max = stream.maxlen - 1 if stream.maxlen > 1 else 1
            plot.setXRange(0, x_max, padding=0)
            plot.enableAutoRange(x=False, y=False)
            plot.setClipToView(True)
            if stream.label_names:
                plot.addLegend(offset=(10, 10))
            stream.plot = plot
            stream.curves = []
            hues = max(1, len(stream.label_names))
            for i, name in enumerate(stream.label_names):
                pen = pg.mkPen(pg.intColor(i, hues=hues), width=1)
                curve = plot.plot(pen=pen, name=name)
                stream.curves.append(curve)
            stream.value_text = pg.TextItem(anchor=(0, 1))
            plot.addItem(stream.value_text)

    def push_values(prefix, values, updated):
        nonlocal layout_dirty
        if not is_allowed(prefix, allowed_prefixes):
            return
        stream = streams.get(prefix)
        if stream is None:
            stream = PgStream(prefix, args.window)
            streams[prefix] = stream
            order.append(prefix)
            layout_dirty = True
        if not stream.data or len(stream.data) != len(values):
            labels = labels_override_for(prefix, args, len(values))
            stream.set_labels(labels, units_override_for(prefix, args))
            layout_dirty = True
        update_stream_data(stream, values)
        updated.add(prefix)

    def update():
        nonlocal layout_dirty
        lines = reader.read_lines()
        if not lines:
            return
        updated = set()
        for line in lines:
            header = parse_header(line)
            if header:
                prefix, labels = header
                if not is_allowed(prefix, allowed_prefixes):
                    continue
                stream = streams.get(prefix)
                if stream is None:
                    stream = PgStream(prefix, args.window)
                    streams[prefix] = stream
                    order.append(prefix)
                stream.set_labels(labels, units_override_for(prefix, args))
                layout_dirty = True
                continue

            parsed = parse_values(line)
            if parsed is None:
                continue
            prefix, values = parsed
            if not is_allowed(prefix, allowed_prefixes):
                continue
            push_values(prefix, values, updated)

            if prefix == "PLOT_Q":
                rpy = compute_rpy_deg(values)
                if rpy is not None:
                    push_values("PLOT_RPY", rpy, updated)

        if layout_dirty:
            rebuild_layout_pg()
            updated.update(order)
            layout_dirty = False

        for prefix in updated:
            stream = streams.get(prefix)
            if stream is None or stream.plot is None or not stream.data:
                continue
            x = list(range(len(stream.data[0])))
            for series, curve in zip(stream.data, stream.curves):
                curve.setData(x, list(series))

            if stream.last_values is not None and stream.value_text is not None:
                lines_text = []
                for name, unit, v in zip(
                    stream.label_names, stream.label_units, stream.last_values
                ):
                    unit_txt = f" {unit}" if unit else ""
                    lines_text.append(f"{name}: {v:.6f}{unit_txt}")
                stream.value_text.setText("\n".join(lines_text))

            range_vals = compute_stream_range(stream)
            if range_vals is not None:
                y_min, y_max, pad = range_vals
                stream.plot.setYRange(y_min - pad, y_max + pad, padding=0)
                if stream.value_text is not None:
                    stream.value_text.setPos(0, y_max + pad)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    poll_ms = max(0, args.poll_ms)
    timer.start(poll_ms)
    if hasattr(app, "exec"):
        app.exec()
    else:
        app.exec_()
    return 0


def run_matplotlib(args):
    if args.prefixes:
        allowed_prefixes = {p.strip() for p in args.prefixes.split(",") if p.strip()}
    elif args.prefix:
        allowed_prefixes = {args.prefix}
    else:
        allowed_prefixes = None

    ser = serial.Serial(args.port, args.baud, timeout=1)
    plt.ion()
    fig = plt.figure()
    streams = {}
    order = []
    fig_background = None
    use_blit = getattr(fig.canvas, "supports_blit", False)

    while True:
        raw = ser.readline()
        if not raw:
            plt.pause(0.001)
            continue
        line = raw.decode(errors="ignore").strip()

        header = parse_header(line)
        if header:
            prefix, labels = header
            if not is_allowed(prefix, allowed_prefixes):
                continue
            stream = streams.get(prefix)
            if stream is None:
                stream = Stream(prefix, args.window)
                streams[prefix] = stream
                order.append(prefix)
            stream.set_labels(labels, units_override_for(prefix, args))
            rebuild_layout(fig, streams, order)
            fig_background = capture_backgrounds(fig, streams, order)
            continue

        parsed = parse_values(line)
        if parsed is None:
            continue
        prefix, values = parsed
        if not is_allowed(prefix, allowed_prefixes):
            continue

        layout_changed = False
        dirty_prefixes = set()

        def push_values(target_prefix, target_values):
            nonlocal layout_changed
            if not is_allowed(target_prefix, allowed_prefixes):
                return
            stream, changed = ensure_stream(
                target_prefix, target_values, args, streams, order
            )
            if changed:
                layout_changed = True
            update_stream_data(stream, target_values)
            dirty_prefixes.add(target_prefix)

        push_values(prefix, values)

        if prefix == "PLOT_Q":
            rpy = compute_rpy_deg(values)
            if rpy is not None:
                push_values("PLOT_RPY", rpy)

        if layout_changed:
            rebuild_layout(fig, streams, order)
            fig_background = capture_backgrounds(fig, streams, order)

        if not dirty_prefixes:
            continue

        needs_full_draw = False
        for prefix in order:
            stream = streams.get(prefix)
            if stream is None or stream.ax is None or not stream.data:
                continue
            if maybe_expand_ylim(stream):
                needs_full_draw = True

            x = list(range(len(stream.data[0])))
            for series, line_obj in zip(stream.data, stream.lines):
                line_obj.set_data(x, list(series))

            if stream.value_text is not None and stream.last_values is not None:
                lines_text = []
                for name, unit, v in zip(
                    stream.label_names, stream.label_units, stream.last_values
                ):
                    unit_txt = f" {unit}" if unit else ""
                    lines_text.append(f"{name}: {v:.6f}{unit_txt}")
                stream.value_text.set_text("\n".join(lines_text))

        if needs_full_draw or fig_background is None:
            fig_background = capture_backgrounds(fig, streams, order)

        if use_blit and fig_background is not None:
            fig.canvas.restore_region(fig_background)
            for prefix in order:
                stream = streams.get(prefix)
                if stream is None or stream.ax is None or not stream.data:
                    continue
                for line_obj in stream.lines:
                    stream.ax.draw_artist(line_obj)
                if stream.value_text is not None:
                    stream.ax.draw_artist(stream.value_text)
            fig.canvas.blit(fig.bbox)
        else:
            fig.canvas.draw()

        fig.canvas.flush_events()
        plt.pause(0.001)


def parse_args():
    parser = argparse.ArgumentParser(description="Live-plot CSV values over serial.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0).")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate.")
    parser.add_argument(
        "--backend",
        choices=["auto", "pg", "mpl"],
        default="auto",
        help="Plot backend: pyqtgraph (pg) or matplotlib (mpl).",
    )
    parser.add_argument(
        "--poll-ms",
        type=int,
        default=0,
        help="Polling interval for pyqtgraph backend (0 = as fast as possible).",
    )
    parser.add_argument(
        "--prefix",
        default="",
        help="Single prefix to parse (e.g. PLOT or PLOT_G).",
    )
    parser.add_argument(
        "--prefixes",
        default="",
        help="Comma-separated prefixes to parse.",
    )
    parser.add_argument("--window", type=int, default=500, help="Number of samples to keep.")
    parser.add_argument(
        "--labels",
        default="err_angle_deg[deg],accel_mag[g],gyro_norm[rad/s],p_corr_norm[rad/s],i_corr_norm[rad/s],q_norm",
        help="Comma-separated labels when no header is sent.",
    )
    parser.add_argument(
        "--units",
        default="",
        help="Comma-separated units matching labels (overrides units in labels).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.backend == "mpl":
        run_matplotlib(args)
        return
    if args.backend == "pg":
        if run_pyqtgraph(args) != 0:
            sys.exit(1)
        return
    if run_pyqtgraph(args) == 0:
        return
    print("Falling back to matplotlib backend.", file=sys.stderr)
    run_matplotlib(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
