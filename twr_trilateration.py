#!/usr/bin/env python3
"""
DS-TWR Trilateration Visualizer

Reads distance measurements from the tag over serial,
trilaterates the tag position from anchor distances,
and plots it live.

Usage:
  python3 twr_trilateration.py --live
  python3 twr_trilateration.py --live --port /dev/ttyACM1
"""

import serial
import re
import math
import argparse
import json
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from itertools import combinations
from statistics import median
from collections import deque
from datetime import datetime


PORT = "/dev/ttyACM0"
BAUD = 115200

# Anchor positions in meters (x, y)
# Change these to match your physical setup
ANCHORS = {
    2: (0.90, 0.60),
    7: (0.90, 0.00),
    10: (0.00, 0.60),
  
}

MIN_ANCHORS = 3          # need at least 3 for trilateration
MAX_DISTANCE = 30.0      # reject distances above this (meters)
MIN_DISTANCE = 0.05      # reject distances below this (meters)
MAX_RESIDUAL = 1.5        # max error per anchor to accept a solution (meters)
MAX_SPREAD = 3.0          # max spread between triplet solutions (meters)
TRAIL_LENGTH = 200        # how many past positions to show

# Regex to parse: [00:00:55.474,365] <inf> ds_twr: Anchor 6: 0.30 m  seq=0
PATTERN = re.compile(
    r"Anchor\s+(\d+)\s*:\s+(-?\d+\.?\d*)\s+m"
)

# ===================== TRILATERATION =====================

def trilaterate_3(a1, a2, a3, d1, d2, d3):
    """Solve 2D position from 3 anchor positions and distances."""
    x1, y1 = a1
    x2, y2 = a2
    x3, y3 = a3

    A = np.array([
        [2 * (x2 - x1), 2 * (y2 - y1)],
        [2 * (x3 - x1), 2 * (y3 - y1)],
    ])

    b = np.array([
        d1**2 - d2**2 + x2**2 + y2**2 - x1**2 - y1**2,
        d1**2 - d3**2 + x3**2 + y3**2 - x1**2 - y1**2,
    ])

    try:
        sol = np.linalg.solve(A, b)
        return float(sol[0]), float(sol[1])
    except np.linalg.LinAlgError:
        return None


def residual_ok(x, y, anchor_ids, distances):
    """Check that the solution is consistent with all used anchors."""
    for aid in anchor_ids:
        ax, ay = ANCHORS[aid]
        d_est = math.hypot(x - ax, y - ay)
        if abs(d_est - distances[aid]) > MAX_RESIDUAL:
            return False
    return True


def compute_position(distances):
    """
    Try all combinations of 3 anchors, trilaterate each,
    filter by residual, return median of good solutions.
    """
    available = [aid for aid in distances if aid in ANCHORS]

    if len(available) < MIN_ANCHORS:
        return None

    solutions = []

    for combo in combinations(available, 3):
        a1, a2, a3 = combo
        sol = trilaterate_3(
            ANCHORS[a1], ANCHORS[a2], ANCHORS[a3],
            distances[a1], distances[a2], distances[a3],
        )
        if sol is None:
            continue

        x, y = sol
        if residual_ok(x, y, combo, distances):
            solutions.append((x, y))

    if not solutions:
        return None

    xs = [s[0] for s in solutions]
    ys = [s[1] for s in solutions]

    if (max(xs) - min(xs)) > MAX_SPREAD or (max(ys) - min(ys)) > MAX_SPREAD:
        return None

    return median(xs), median(ys), len(solutions)


def load_anchors(path):
    """
    Load anchors from JSON object:
      {"1":[x,y], "2":[x,y], ...}
    """
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)

    anchors = {}
    for key, value in raw.items():
        aid = int(key)
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError(f"Anchor {key} must be [x, y]")
        anchors[aid] = (float(value[0]), float(value[1]))

    return anchors


def parse_anchor_entry(entry):
    """
    Parse one anchor entry in form: ID:X,Y
    Example: 7:1.2,0.4
    """
    try:
        anchor_id_str, coords_str = entry.split(":", 1)
        x_str, y_str = coords_str.split(",", 1)
        anchor_id = int(anchor_id_str.strip())
        x = float(x_str.strip())
        y = float(y_str.strip())
        return anchor_id, (x, y)
    except Exception as exc:
        raise ValueError(
            f"Invalid --anchor '{entry}'. Use ID:X,Y (example: 7:1.2,0.4)"
        ) from exc


# ===================== VISUALIZATION =====================

class LivePlot:
    def __init__(self):
        self.x_hist = deque(maxlen=TRAIL_LENGTH)
        self.y_hist = deque(maxlen=TRAIL_LENGTH)
        self.count = 0

        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self._setup_axes()

        # trail + current position
        self.trail_line, = self.ax.plot(
            [], [], "b-", alpha=0.3, linewidth=1.5, label="Trail"
        )
        self.pos_dot = self.ax.scatter(
            [], [], c="blue", s=120, zorder=5,
            marker="o", edgecolors="darkblue", linewidths=2,
            label="Tag",
        )

        # draw anchors
        ax_coords = list(ANCHORS.values())
        self.ax.scatter(
            [p[0] for p in ax_coords],
            [p[1] for p in ax_coords],
            c="red", s=250, marker="^", zorder=10,
            edgecolors="darkred", linewidths=2, label="Anchors",
        )
        for aid, (x, y) in ANCHORS.items():
            self.ax.annotate(
                f"A{aid}",
                (x, y),
                xytext=(8, 8),
                textcoords="offset points",
                fontsize=10,
                fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.9),
            )

        self.info_text = self.ax.text(
            0.02, 0.98, "",
            transform=self.ax.transAxes,
            verticalalignment="top",
            fontsize=10,
            bbox=dict(boxstyle="round", facecolor="lightgreen", alpha=0.9),
        )

        self.ax.legend(loc="upper right", fontsize=10)

    def _setup_axes(self):
        xs = [p[0] for p in ANCHORS.values()]
        ys = [p[1] for p in ANCHORS.values()]
        margin = 1.0
        self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
        self.ax.set_ylim(min(ys) - margin, max(ys) + margin)
        self.ax.set_xlabel("X (m)", fontsize=12, fontweight="bold")
        self.ax.set_ylabel("Y (m)", fontsize=12, fontweight="bold")
        self.ax.set_title("DS-TWR Live Position", fontsize=14, fontweight="bold")
        self.ax.grid(True, alpha=0.3, linestyle="--")
        self.ax.set_aspect("equal")

    def update(self, x, y, n_anchors, n_triplets):
        self.x_hist.append(x)
        self.y_hist.append(y)
        self.count += 1

        self.trail_line.set_data(list(self.x_hist), list(self.y_hist))
        self.pos_dot.set_offsets([[x, y]])

        self.info_text.set_text(
            f"Position: ({x:.2f}, {y:.2f}) m\n"
            f"Sample: {self.count}\n"
            f"Anchors: {n_anchors}  Triplets: {n_triplets}"
        )

        return self.trail_line, self.pos_dot, self.info_text


# ===================== SERIAL READER =====================

def run_live(port, baud):
    print("=" * 60)
    print("DS-TWR Trilateration - Live Mode")
    print("=" * 60)
    print(f"\nAnchors ({len(ANCHORS)}):")
    for aid, (x, y) in sorted(ANCHORS.items()):
        print(f"  A{aid}: ({x:.2f}, {y:.2f}) m")
    print(f"\nSerial: {port} @ {baud}")
    print("Close the plot window to stop.\n")

    ser = serial.Serial(port, baud, timeout=0.1)
    ser.reset_input_buffer()

    viz = LivePlot()
    distances = {}
    required_anchor_ids = set(ANCHORS.keys())
    ignored_anchor_ids = set()
    last_status_print = 0.0

    def update_frame(frame):
        nonlocal distances, ignored_anchor_ids, last_status_print

        now = time.monotonic()

        # read several lines per frame for responsiveness
        for _ in range(20):
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
            except Exception:
                continue

            m = PATTERN.search(line)
            if not m:
                continue

            anchor_id = int(m.group(1))
            dist = float(m.group(2))

            if anchor_id not in ANCHORS:
                if anchor_id not in ignored_anchor_ids:
                    ignored_anchor_ids.add(anchor_id)
                    print(f"Ignoring A{anchor_id}: no coordinate configured in ANCHORS")
                continue
            if not (MIN_DISTANCE <= dist <= MAX_DISTANCE):
                continue

            distances[anchor_id] = dist
            print(
                f"RANGE  A{anchor_id:>2}: {dist:5.2f} m  "
                f"(active {len(distances)}/{len(ANCHORS)})"
            )

            missing = sorted(required_anchor_ids - set(distances.keys()))
            if missing:
                if now - last_status_print >= 1.0:
                    last_status_print = now
                    missing_text = ", ".join(f"A{aid}" for aid in missing)
                    print(f"Waiting for anchor(s): {missing_text}")
                continue

            selected_distances = {aid: distances[aid] for aid in required_anchor_ids}
            result = compute_position(selected_distances)
            if result is not None:
                x, y, n_tri = result
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(
                    f"{viz.count+1:05d} | {ts} | "
                    f"X={x:6.2f} m  Y={y:6.2f} m | "
                    f"Anchors={len(selected_distances)}  Triplets={n_tri}"
                )
                return viz.update(x, y, len(selected_distances), n_tri)

            if now - last_status_print >= 1.0:
                last_status_print = now
                print(
                    "All configured anchors are present, but current geometry/filter "
                    "check did not produce a stable fix yet."
                )

        return viz.trail_line, viz.pos_dot, viz.info_text

    try:
        ani = FuncAnimation(
            viz.fig, update_frame,
            interval=50, blit=True, cache_frame_data=False,
        )
        plt.show()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
        print("Serial closed.")


# ===================== MAIN =====================

def main():
    global ANCHORS, MIN_ANCHORS, MAX_RESIDUAL, MAX_SPREAD

    parser = argparse.ArgumentParser(
        description="DS-TWR Trilateration Visualizer",
    )
    parser.add_argument("--live", action="store_true", help="Live from serial")
    parser.add_argument("--port", default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument("--baud", type=int, default=BAUD, help=f"Baud rate (default: {BAUD})")
    parser.add_argument(
        "--anchors-file",
        default=None,
        help='JSON with anchors, e.g. {"1":[0.0,0.0],"2":[1.0,0.0],"3":[0.0,1.0]}',
    )
    parser.add_argument(
        "--anchor",
        action="append",
        default=[],
        help="Add/override one anchor as ID:X,Y (repeatable)",
    )
    parser.add_argument(
        "--clear-default-anchors",
        action="store_true",
        help="Ignore built-in ANCHORS and use only --anchors-file/--anchor",
    )
    parser.add_argument(
        "--min-anchors",
        type=int,
        default=MIN_ANCHORS,
        help=f"Minimum anchors needed to solve position (default: {MIN_ANCHORS})",
    )
    parser.add_argument(
        "--max-residual",
        type=float,
        default=MAX_RESIDUAL,
        help=f"Max anchor residual in meters (default: {MAX_RESIDUAL})",
    )
    parser.add_argument(
        "--max-spread",
        type=float,
        default=MAX_SPREAD,
        help=f"Max spread between triplet solutions in meters (default: {MAX_SPREAD})",
    )

    args = parser.parse_args()

    if args.clear_default_anchors:
        ANCHORS = {}

    if args.anchors_file:
        ANCHORS = load_anchors(args.anchors_file)

    for entry in args.anchor:
        aid, coords = parse_anchor_entry(entry)
        ANCHORS[aid] = coords

    MIN_ANCHORS = max(1, args.min_anchors)
    MAX_RESIDUAL = max(0.01, args.max_residual)
    MAX_SPREAD = max(0.01, args.max_spread)

    if args.live:
        run_live(args.port, args.baud)
    else:
        parser.print_help()
        print("\nRun with --live to start.")


if __name__ == "__main__":
    main()
