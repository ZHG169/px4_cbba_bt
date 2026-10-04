#!/usr/bin/env python3
"""起飛懸停測試的紀錄與報告。

    hover_report.py record <輸出目錄> [--ns uav1 uav2] [--duration 秒]
        （gcs 容器，需要 ROS 2 與 px4_msgs）經由 mesh 訂閱各台的
        vehicle_local_position_v1 / vehicle_status_v1，寫成 <目錄>/samples.csv。
        events.csv（time,label）可以在測試過程中另外附加，用來標示各階段。

    hover_report.py plot <輸出目錄> [--altitude 5] [--tol 0.3] [--hover-sec 30] [--drift 0.5]
                                    [--settle 2]
        （只需要 matplotlib）畫高度與水平偏移，並依驗收標準寫 summary.csv。
        懸停視窗從「進入 ±tol 後再等 settle 秒」開始算，避免把爬升末段算進去；
        另外統計從懸停開始到紀錄結束的整段（含 mesh 干擾階段）。

圖上的文字使用英文，避免容器裡沒有中文字型時變成方框。
"""
import argparse
import csv
import math
import os
import sys
import time
from collections import defaultdict


# ---- record（ROS 2）-------------------------------------------------------------
def record(out_dir, namespaces, duration):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "samples.csv")
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["time", "ns", "x", "y", "z", "vz", "arming_state", "nav_state"])

    rclpy.init()
    node = rclpy.create_node("hover_report_recorder")
    status = {}
    last = defaultdict(float)
    count = defaultdict(int)

    def on_status(ns):
        def cb(msg):
            status[ns] = (msg.arming_state, msg.nav_state)
        return cb

    def on_position(ns):
        def cb(msg):
            now = time.time()
            if now - last[ns] < 0.1:   # 每台最多 10 Hz
                return
            last[ns] = now
            arm, nav = status.get(ns, (-1, -1))
            w.writerow([f"{now:.3f}", ns, f"{msg.x:.3f}", f"{msg.y:.3f}", f"{msg.z:.3f}",
                        f"{msg.vz:.3f}", arm, nav])
            count[ns] += 1
        return cb

    for ns in namespaces:
        node.create_subscription(VehicleStatus, f"/{ns}/fmu/out/vehicle_status_v1",
                                 on_status(ns), qos_profile_sensor_data)
        node.create_subscription(VehicleLocalPosition, f"/{ns}/fmu/out/vehicle_local_position_v1",
                                 on_position(ns), qos_profile_sensor_data)
    print(f"[hover_report] 紀錄 {', '.join(namespaces)} -> {path}", flush=True)
    end = time.time() + duration if duration > 0 else float("inf")
    next_flush = time.time() + 1.0
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() > next_flush:
                f.flush()
                next_flush = time.time() + 1.0
    except KeyboardInterrupt:
        pass
    f.close()
    print("[hover_report] 樣本數 " + ", ".join(f"{ns}={count[ns]}" for ns in namespaces))
    node.destroy_node()
    rclpy.shutdown()


# ---- plot（matplotlib）-----------------------------------------------------------
ARMED, OFFBOARD = 2, 14
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7"]
SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e1"


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def analyse(rows, altitude, tol, hover_sec, drift_limit, settle):
    """每台：起飛時間、到達時間、懸停 hover_sec 內的高度誤差與水平偏移。"""
    t0 = float(rows[0]["time"])
    ground_z = float(rows[0]["z"])
    ground_x, ground_y = float(rows[0]["x"]), float(rows[0]["y"])
    series = []
    engaged_at = reached_at = None
    for r in rows:
        t = float(r["time"])
        alt = ground_z - float(r["z"])
        drift = math.hypot(float(r["x"]) - ground_x, float(r["y"]) - ground_y)
        engaged = int(r["arming_state"]) == ARMED and int(r["nav_state"]) == OFFBOARD
        if engaged and engaged_at is None:
            engaged_at = t
        if engaged_at is not None and reached_at is None and abs(alt - altitude) < tol:
            reached_at = t
        series.append((t, alt, drift, engaged))
    res = dict(engaged_at=engaged_at, reached_at=reached_at, series=series, t0=t0)
    if reached_at is not None:
        begin = reached_at + settle
        res["hover_begin"] = begin
        window = [s for s in series if begin <= s[0] <= begin + hover_sec]
        whole = [s for s in series if s[0] >= begin]
        res["max_alt_err_all"] = max(abs(s[1] - altitude) for s in whole)
        res["max_drift_all"] = max(s[2] for s in whole)
        res["whole_span"] = whole[-1][0] - begin
        span = window[-1][0] - window[0][0] if window else 0.0
        errs = [abs(s[1] - altitude) for s in window]
        drifts = [s[2] for s in window]
        res.update(
            hover_span=span,
            max_alt_err=max(errs) if errs else float("nan"),
            mean_alt=sum(s[1] for s in window) / len(window) if window else float("nan"),
            max_drift=max(drifts) if drifts else float("nan"),
            stayed_offboard=all(s[3] for s in window),
        )
        res["pass"] = (span >= hover_sec * 0.95 and res["max_alt_err"] <= tol and
                       res["max_drift"] <= drift_limit and res["stayed_offboard"])
    else:
        res["pass"] = False
    return res


def plot(out_dir, altitude, tol, hover_sec, drift_limit, settle):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = read_csv(os.path.join(out_dir, "samples.csv"))
    by_ns = defaultdict(list)
    for r in rows:
        by_ns[r["ns"]].append(r)
    events_path = os.path.join(out_dir, "events.csv")
    events = read_csv(events_path) if os.path.exists(events_path) else []
    start = min(float(r["time"]) for r in rows)

    results = {ns: analyse(rs, altitude, tol, hover_sec, drift_limit, settle)
               for ns, rs in sorted(by_ns.items())}

    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID, "axes.grid": True, "grid.color": GRID,
        "axes.spines.top": False, "axes.spines.right": False, "font.size": 9.5,
        "axes.titleweight": "bold", "axes.titlelocation": "left", "legend.frameon": False,
    })
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.4), sharex=True,
                                   gridspec_kw=dict(hspace=0.22))
    for i, (ns, res) in enumerate(results.items()):
        c = COLORS[i % len(COLORS)]
        t = [s[0] - start for s in res["series"]]
        ax1.plot(t, [s[1] for s in res["series"]], color=c, lw=2, label=ns)
        ax2.plot(t, [s[2] for s in res["series"]], color=c, lw=2, label=ns)
        if res["reached_at"] is not None:
            hb = res["hover_begin"] - start
            ax1.axvspan(hb, hb + hover_sec, color=c, alpha=0.08, lw=0)
            ax1.annotate(f"{ns} hover window", (hb, altitude * 0.5), xytext=(4, 0),
                         textcoords="offset points", fontsize=8.5, color=c)
    ax1.axhspan(altitude - tol, altitude + tol, color=INK_2, alpha=0.08, lw=0)
    ax1.axhline(altitude, color=INK_2, lw=1, ls=(0, (4, 3)))
    ax1.set_ylabel("altitude above start (m)")
    ax1.set_title(f"Takeoff and hover — target {altitude:g} m (grey band = ±{tol:g} m, "
                  f"shaded = {hover_sec:g} s hover window after {settle:g} s settling)")
    ax2.axhline(drift_limit, color=INK_2, lw=1, ls=(0, (4, 3)))
    ax2.annotate(f"limit {drift_limit:g} m", (0, drift_limit), xytext=(2, 3),
                 textcoords="offset points", fontsize=8.5, color=INK_2)
    ax2.set_ylabel("horizontal drift from start (m)")
    ax2.set_xlabel("time since recording started (s)")
    for ax in (ax1, ax2):
        for e in events:
            et = float(e["time"]) - start
            ax.axvline(et, color=INK, lw=0.9, ls=(0, (1, 2)))
        ax.legend(loc="center right")
    for e in events:
        ax1.annotate(e["label"], (float(e["time"]) - start, ax1.get_ylim()[1]),
                     xytext=(3, -12), textcoords="offset points", fontsize=8, color=INK,
                     rotation=0)
    path = os.path.join(out_dir, "takeoff_hover.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"  wrote {path}")

    path = os.path.join(out_dir, "summary.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ns", "engaged_s", "reached_s", "climb_time_s", "hover_span_s", "mean_alt_m",
                    "max_alt_err_m", "max_drift_m", "stayed_offboard", "pass",
                    "whole_hover_s", "max_alt_err_whole_m", "max_drift_whole_m"])
        for ns, r in results.items():
            fmt = (lambda v: "" if v is None else f"{v - start:.2f}")
            climb = (f"{r['reached_at'] - r['engaged_at']:.2f}"
                     if r["reached_at"] and r["engaged_at"] else "")
            w.writerow([ns, fmt(r["engaged_at"]), fmt(r["reached_at"]), climb,
                        f"{r.get('hover_span', 0):.1f}", f"{r.get('mean_alt', float('nan')):.3f}",
                        f"{r.get('max_alt_err', float('nan')):.3f}",
                        f"{r.get('max_drift', float('nan')):.3f}",
                        int(r.get("stayed_offboard", False)), int(r["pass"]),
                        f"{r.get('whole_span', 0):.1f}",
                        f"{r.get('max_alt_err_all', float('nan')):.3f}",
                        f"{r.get('max_drift_all', float('nan')):.3f}"])
            print(f"  {ns}: climb {climb or '-'} s, hover {r.get('hover_span', 0):.1f} s, "
                  f"max alt err {r.get('max_alt_err', float('nan')):.3f} m, "
                  f"max drift {r.get('max_drift', float('nan')):.3f} m -> "
                  f"{'PASS' if r['pass'] else 'FAIL'}  | whole hover {r.get('whole_span', 0):.0f} s: "
                  f"max alt err {r.get('max_alt_err_all', float('nan')):.3f} m, "
                  f"max drift {r.get('max_drift_all', float('nan')):.3f} m")
    print(f"  wrote {path}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("out_dir")
    r.add_argument("--ns", nargs="+", default=["uav1", "uav2"])
    r.add_argument("--duration", type=float, default=0.0, help="秒；0 表示直到 Ctrl+C")
    g = sub.add_parser("plot")
    g.add_argument("out_dir")
    g.add_argument("--altitude", type=float, default=5.0)
    g.add_argument("--tol", type=float, default=0.3)
    g.add_argument("--hover-sec", type=float, default=30.0)
    g.add_argument("--drift", type=float, default=0.5)
    g.add_argument("--settle", type=float, default=2.0, help="進入 ±tol 後等多久才開始算懸停")
    args = p.parse_args(sys.argv[1:] if len(sys.argv) > 1 else ["-h"])
    if args.cmd == "record":
        record(args.out_dir, args.ns, args.duration)
    else:
        plot(args.out_dir, args.altitude, args.tol, args.hover_sec, args.drift, args.settle)


if __name__ == "__main__":
    main()
