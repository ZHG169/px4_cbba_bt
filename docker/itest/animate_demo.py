#!/usr/bin/env python3
"""把 itest demo（或 uav）記下的事件畫成俯視動畫：無人機飛去自己拿到的任務、隨機火警出現、丟出地面任務。

  python3 docker/itest/animate_demo.py <itest 輸出目錄> --frames <目錄>   每一格存成 PNG（主機用 ffmpeg 合成 MP4；
                                                                         itest.py demo 會自動做）
  python3 docker/itest/animate_demo.py <itest 輸出目錄>                    直接存成 demo.gif（沒有 ffmpeg 時）

要 matplotlib（主機沒有時在 sim 容器跑）。只用 ros_bridge 記下的話題（PX4 位置、current_task、task_result、new_task），
不看節點的 log；有 demo_plan.json（itest demo 寫的）時另外畫出全部任務點、火警起火的時刻。

播放速度：事件（建立、指派、完成、起火、丟出地面任務）前後 --window 秒用 --slow 倍速，其他時間 --speedup 倍速；
起火、丟出地面任務時停 --freeze 秒，結尾停 3 秒。
左邊：俯視地圖。綠色方塊 = 固定任務；火點起火時才出現（紅色方塊＋光圈，在某台的航線上）；每台的航跡（實線）、
目前要去的任務（虛線）；任務的圓圈未分配是白的、有人拿著時填那台的顏色、完成後變灰；地面處置任務是紅色方塊。
右邊：事件時間軸。樣式沿用 cbba_core/scripts/plot_results.py；圖上的文字用英文（容器裡沒有中文字型）。
"""
import argparse
import bisect
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "ros2_ws", "src", "cbba_core", "scripts"))
from plot_results import AGENT_COLORS, FIRE, GRID, INK, INK_2, MUTED, SURFACE  # noqa: E402

GROUND_TYPE = 2
PATROL = "#3f9b3f"        # 固定巡檢點（綠色平台）
DOG = "#e07000"           # 假的 V60（2026-10-13）


def load(out_dir):
    ev = [json.loads(line) for line in open(os.path.join(out_dir, "events.jsonl"), encoding="utf-8") if line.strip()]
    pos, assigned, done, tasks, dog, closed, battery = {}, {}, {}, {}, [], {}, {}
    for e in ev:
        k, t = e["kind"], e["t"]
        if k == "battery":
            battery.setdefault(e["uav"], []).append((t, e["pct"]))
        elif k == "dog":
            dog.append((t, e["x"], e["y"]))
        elif k == "closed" and e["msg"].get("status") == 1:
            closed.setdefault(e["msg"]["task_id"], t)
        elif k == "pos":
            pos.setdefault(e["uav"], []).append((t, e["x"], e["y"]))
        elif k == "assigned":
            assigned.setdefault(e["uav"], []).append((t, e["msg"].get("task_id", 0)))
        elif k == "result" and e["msg"].get("success"):
            done.setdefault(e["msg"]["task_id"], (t, e["uav"]))
        elif k == "new_task":
            m = e["msg"]
            if m["task_id"] not in tasks:
                p = m.get("position", {})
                tasks[m["task_id"]] = dict(t=t, x=p.get("x", 0.0), y=p.get("y", 0.0), type=m.get("type", 1),
                                           by=e["uav"])
    return pos, assigned, done, tasks, dog, closed, battery


def at(series, t, default=0):
    i = bisect.bisect_right([s[0] for s in series], t) - 1
    return series[i][1] if i >= 0 else default


def frame_times(t_start, t_end, keys, freezes, fps, fast, slow, window, freeze):
    """每一格對應的事件時刻：事件附近慢放、其他快轉；freezes 的時刻停住"""
    times, t = [], t_start
    keys = sorted(keys)
    pending = sorted(freezes)
    while t <= t_end:
        while pending and pending[0] <= t:
            times += [pending.pop(0)] * int(freeze * fps)
        times.append(t)
        i = bisect.bisect_left(keys, t - window)
        near = i < len(keys) and keys[i] <= t + window
        t += (slow if near else fast) / fps
    return times + [t_end] * (3 * fps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--frames", help="每一格存成這個目錄的 PNG（之後用 ffmpeg 合成）；沒給時存 demo.gif")
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--speedup", type=float, default=3.0, help="沒有事件時的倍速")
    ap.add_argument("--slow", type=float, default=1.0, help="事件附近的倍速")
    ap.add_argument("--window", type=float, default=2.0)
    ap.add_argument("--freeze", type=float, default=1.5)
    args = ap.parse_args()

    pos, assigned, done, tasks, dog, closed, battery = load(args.out_dir)
    if not tasks or not pos:
        sys.exit("events.jsonl 裡沒有任務或位置（要用 itest.py demo 或 uav 的輸出）")
    plan_path = os.path.join(args.out_dir, "demo_plan.json")
    plan = json.load(open(plan_path, encoding="utf-8")) if os.path.exists(plan_path) else None
    points = plan["points"] if plan else {}
    fires = [f for f in plan["fires"] if "spawned" in f] if plan else []

    ids = sorted(pos)
    colors = {v: AGENT_COLORS[i % len(AGENT_COLORS)] for i, v in enumerate(ids)}
    air = [tid for tid in sorted(tasks, key=lambda k: tasks[k]["t"]) if tasks[tid]["type"] != GROUND_TYPE]
    ground = sorted((tid for tid in tasks if tasks[tid]["type"] == GROUND_TYPE), key=lambda k: tasks[k]["t"])

    def point_of(x, y):
        best = min(points.items(), key=lambda kv: math.hypot(kv[1]["x"] - x, kv[1]["y"] - y), default=None)
        return best[0] if best and math.hypot(best[1]["x"] - x, best[1]["y"] - y) < 1.5 else None

    def fire_of(x, y):
        return next((f["name"] for f in fires if math.hypot(f["x"] - x, f["y"] - y) <= 2.5), None)

    names = {}
    for i, tid in enumerate(air):
        x, y = tasks[tid]["x"], tasks[tid]["y"]
        p, f = point_of(x, y), fire_of(x, y)
        names[tid] = f"{f} fire" if f else (f"{p} task" if p else f"T{i + 1}")

    t_start = min(tasks[t]["t"] for t in air) - 2.0
    t_end = max([v[0] for v in done.values()] + [tasks[g]["t"] for g in ground] + list(closed.values()) +
                [t_start + 10]) + 3.0
    # 做完之後飛回起飛點（2026-10-13）：畫到位置不再變（最多再 40 s）
    last_move = max((p[0] for v in pos.values() for a, p in zip(v, v[1:])
                     if math.hypot(p[1] - a[1], p[2] - a[2]) > 0.05), default=t_end)
    t_end = max(t_end, min(last_move + 2.0, t_end + 40.0))

    # 事件時間軸
    events = []      # 2026-10-14：不顯示「uavN created …」（建立者是測試指定的，看不出東西）
    for f in fires:
        where = (f"at patrol point {f['point']}" if "point" in f else f"beside a route ({f['side']})" if "side" in f
                 else f"at ({f['x']:g}, {f['y']:g})" if f.get("custom") else f"on uav{f.get('on_route_of', '?')}'s route")
        events.append((f["spawned"], f"!! {f['name']} fire {where}", FIRE))
    for v in ids:
        last = 0
        for t, tid in assigned.get(v, []):
            if tid and tid != last and tid in names:
                events.append((t, f"uav{v} <- {names[tid]}", colors[v]))
            last = tid
    for tid, (t, v) in done.items():
        if tid in names:
            events.append((t, f"uav{v} done {names[tid]}", colors.get(v, INK)))
    point_mode = bool(plan and plan.get("mode") == "point")
    for g in ground:
        f = fire_of(tasks[g]["x"], tasks[g]["y"]) or "fire"
        how = ("spots {} -> ground task, hovers" if point_mode else
               "spots {} en route -> ground task, diverts and hovers" if plan and plan.get("mode") in ("side", "custom")
               else "spots {} en route -> ground task")
        events.append((tasks[g]["t"], f"uav{tasks[g]['by']} " + how.format(f), FIRE))
        if g in closed:
            events.append((closed[g], "dog done -> ground task closed", DOG))
    events.sort()

    fig = plt.figure(figsize=(12.8, 7.2), facecolor=SURFACE)        # 1280×720（dpi 100）
    gs = fig.add_gridspec(1, 2, width_ratios=[1.75, 1], wspace=0.06)
    ax = fig.add_subplot(gs[0])
    # 右邊（2026-10-14）：事件時間軸、電量折線圖（有電量資料時）、圖例
    right = gs[1].subgridspec(3, 1, height_ratios=[5.0, 3.0, 1.4] if battery else [8.0, 0.01, 1.4], hspace=0.35)
    axl = fig.add_subplot(right[0])
    axl.axis("off")
    axb = fig.add_subplot(right[1])
    axg = fig.add_subplot(right[2])
    axg.axis("off")

    xs = [tasks[t]["x"] for t in tasks] + [p[1] for v in ids for p in pos[v]] + [p["x"] for p in points.values()] + \
        [p[1] for p in dog]
    ys = [tasks[t]["y"] for t in tasks] + [p[2] for v in ids for p in pos[v]] + [p["y"] for p in points.values()] + \
        [p[2] for p in dog]
    pad = 5.0
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect("equal")
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRID, linewidth=0.8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.set_xlabel("x east (m)", color=INK_2)
    ax.set_ylabel("y north (m)", color=INK_2)
    world = plan["world"] if plan else "Gazebo"
    sub = f", seed {plan['seed']}" if plan else ""
    ax.set_title(f"CBBA on PX4 SITL ({world}): {len(ids)} UAVs, {len(air)} tasks{sub}",
                 loc="left", color=INK, fontsize=12, fontweight="bold")

    # 全部點（一開始就畫，和 Gazebo 場景一樣）：綠色平台 G = 固定任務、灰色方塊 R = 可能起火的地方
    for name, p in points.items():
        color = PATROL if name.startswith("G") else "#9a9994"
        ax.scatter([p["x"]], [p["y"]], s=420, marker="s", color=color, alpha=0.35, linewidths=0, zorder=1)
        ax.annotate(name, (p["x"], p["y"]), xytext=(-10, -17), textcoords="offset points",
                    fontsize=9, color=color, fontweight="bold", zorder=2)
    # 火點：起火的時刻才出現（紅色方塊＋光圈）
    fire_art = {}
    for f in fires:
        block = ax.scatter([f["x"]], [f["y"]], s=300, marker="s", color=FIRE, alpha=0.8, linewidths=0, zorder=3,
                           visible=False)
        halo = ax.scatter([f["x"]], [f["y"]], s=1100, facecolors=[(0.78, 0.09, 0.11, 0.15)], edgecolors=FIRE,
                          linewidths=2, zorder=3, visible=False)
        lbl = ax.annotate("FIRE (tag 101)", (f["x"], f["y"]), xytext=(12, -22), textcoords="offset points",
                          fontsize=9, color=FIRE, fontweight="bold", zorder=6, visible=False)
        fire_art[f["name"]] = (f["spawned"], halo, lbl, block)

    task_art, task_lbl = {}, {}
    for tid in air:
        x, y = tasks[tid]["x"], tasks[tid]["y"]
        task_art[tid] = ax.scatter([x], [y], s=200, marker="o", facecolors=SURFACE, edgecolors=MUTED,
                                   linewidths=1.8, zorder=4, visible=False)
        task_lbl[tid] = ax.annotate("", (x, y), xytext=(10, 8), textcoords="offset points",
                                    fontsize=9, color=INK, zorder=5, visible=False)
    ground_art = {}
    for g in ground:
        x, y = tasks[g]["x"], tasks[g]["y"]
        sq = ax.scatter([x], [y], s=170, marker="s", color=FIRE, edgecolors=SURFACE, linewidths=1.5,
                        zorder=6, visible=False)
        lbl = ax.annotate("ground task -> dog", (x, y), xytext=(12, -36), textcoords="offset points",
                          fontsize=9, color=FIRE, fontweight="bold", zorder=6, visible=False)
        ground_art[g] = (sq, lbl)

    trails, drones, links = {}, {}, {}
    for v in ids:
        trails[v], = ax.plot([], [], color=colors[v], lw=1.8, alpha=0.85, zorder=2)
        links[v], = ax.plot([], [], color=colors[v], lw=1.2, ls="--", zorder=2)
        drones[v] = ax.scatter([], [], s=230, marker="^", color=colors[v], edgecolors=INK, linewidths=1.0, zorder=7)
    dog_trail, = ax.plot([], [], color=DOG, lw=1.8, alpha=0.85, zorder=2)
    dog_art = ax.scatter([], [], s=200, marker="D", color=DOG, edgecolors=INK, linewidths=1.0, zorder=7)
    clock = ax.text(0.99, 0.02, "", transform=ax.transAxes, ha="right", va="bottom", fontsize=11,
                    color=INK, bbox=dict(facecolor=SURFACE, edgecolor=GRID, pad=3))
    # 電量（PX4 battery_status，2026-10-14）：右邊的橫向長條圖，每台一條、長度 = 目前電量，隨時間縮短。
    # 橫軸從資料的最低值附近開始（任務期間只掉十幾 %，從 0 開始畫的話幾乎看不出縮短）
    bat_bar, bat_val = {}, {}
    if battery:
        lo = max(0.0, math.floor(min(p[1] for v in battery.values() for p in v)) - 3.0)
        axb.set_xlim(lo, 100.0)
        axb.set_ylim(-0.6, len(ids) - 0.4)
        axb.invert_yaxis()
        axb.set_yticks(range(len(ids)))
        axb.set_yticklabels([f"uav{v}" for v in ids], fontsize=9, color=INK)
        axb.set_facecolor(SURFACE)
        axb.grid(axis="x", color=GRID, linewidth=0.8)
        for side in ("top", "right"):
            axb.spines[side].set_visible(False)
        axb.tick_params(axis="x", labelsize=8, colors=INK_2)
        axb.set_title(f"Battery (%)   axis from {lo:.0f}", loc="left", fontsize=11, fontweight="bold", color=INK)
        for i, v in enumerate(ids):
            axb.barh(i, 100.0 - lo, left=lo, height=0.6, color=GRID, zorder=1)          # 底色 = 滿電
            bat_bar[v] = axb.barh(i, 0.0, left=lo, height=0.6, color=colors[v], zorder=2)[0]
            bat_val[v] = axb.text(lo, i, "", va="center", ha="left", fontsize=9, family="monospace",
                                  color=INK, fontweight="bold", zorder=3)
    else:
        axb.axis("off")

    axl.text(0.0, 1.0, "Events", fontsize=12, fontweight="bold", color=INK, va="top", transform=axl.transAxes)
    n_lines = 11 if battery else 15
    step = 0.9 / n_lines
    lines = [axl.text(0.0, 0.92 - i * step, "", fontsize=9, va="top", transform=axl.transAxes,
                      family="monospace") for i in range(n_lines)]
    handles = [Line2D([], [], marker="^", ls="", markersize=11, color=colors[v], markeredgecolor=INK,
                      label=f"uav{v}") for v in ids]
    if points:
        handles += [Line2D([], [], marker="s", ls="", markersize=11, color=PATROL, alpha=0.5, label="fixed task (G)")]
    if dog:
        handles += [Line2D([], [], marker="D", ls="", markersize=9, color=DOG, markeredgecolor=INK, label="dog (fake V60)")]
    handles += [Line2D([], [], marker="o", ls="", markersize=13, markerfacecolor=(0.78, 0.09, 0.11, 0.15),
                       markeredgecolor=FIRE, label="fire (tag 101)"),
                Line2D([], [], marker="s", ls="", markersize=10, color=FIRE, label="ground task (dog)")]
    axg.legend(handles=handles, loc="center left", fontsize=9, frameon=False, ncol=2, bbox_to_anchor=(0.0, 0.5))

    keys = [e[0] for e in events]
    freezes = [f["spawned"] for f in fires] + [tasks[g]["t"] for g in ground] + list(closed.values())
    times = frame_times(t_start, t_end, keys, freezes, args.fps, args.speedup, args.slow, args.window, args.freeze)

    def draw(t, rate):
        for v in ids:
            pts = [p for p in pos[v] if t_start - 1 <= p[0] <= t]
            if not pts:
                continue
            trails[v].set_data([p[1] for p in pts], [p[2] for p in pts])
            drones[v].set_offsets([[pts[-1][1], pts[-1][2]]])
            cur = at(assigned.get(v, []), t)
            if cur in tasks and not (cur in done and done[cur][0] <= t):
                links[v].set_data([pts[-1][1], tasks[cur]["x"]], [pts[-1][2], tasks[cur]["y"]])
            else:
                links[v].set_data([], [])
        dp = [p for p in dog if t_start - 1 <= p[0] <= t]
        if dp:
            dog_trail.set_data([p[1] for p in dp], [p[2] for p in dp])
            dog_art.set_offsets([[dp[-1][1], dp[-1][2]]])
        for name, (ts, halo, lbl, block) in fire_art.items():
            if ts <= t:
                halo.set_visible(True)
                lbl.set_visible(True)
                block.set_visible(True)
                halo.set_sizes([900 + 350 * math.sin((t - ts) * 3.0)])
        for tid in air:
            if tasks[tid]["t"] > t:
                continue
            task_art[tid].set_visible(True)
            task_lbl[tid].set_visible(True)
            holder = next((v for v in ids if at(assigned.get(v, []), t) == tid), None)
            if tid in done and done[tid][0] <= t:
                task_art[tid].set_facecolors([GRID])
                task_art[tid].set_edgecolors([MUTED])
                task_lbl[tid].set_text("done")
                task_lbl[tid].set_color(MUTED)
            elif holder is not None:
                task_art[tid].set_facecolors([colors[holder]])
                task_art[tid].set_edgecolors([INK])
                task_lbl[tid].set_text(f"-> uav{holder}")
                task_lbl[tid].set_color(INK)
            else:
                task_lbl[tid].set_text("waiting")
                task_lbl[tid].set_color(MUTED)
        for g, (sq, lbl) in ground_art.items():
            if tasks[g]["t"] <= t:
                sq.set_visible(True)
                lbl.set_visible(True)
                if g in closed and closed[g] <= t:
                    lbl.set_text("ground task done")
        past = [e for e in events if e[0] <= t][-n_lines:]
        for i, ln in enumerate(lines):
            if i < len(past):
                et, text, color = past[i]
                ln.set_text(f"{et - t_start:6.1f} s  {text}")
                ln.set_color(color)
            else:
                ln.set_text("")
        for v in bat_bar:
            pct = at(battery.get(v, []), t, None)
            x0 = axb.get_xlim()[0]
            bat_bar[v].set_width(0.0 if pct is None else max(0.0, pct - x0))
            bat_val[v].set_text("" if pct is None else f" {pct:5.1f}%")
            bat_val[v].set_x(x0 if pct is None else max(x0, pct - 9.0 * (100.0 - x0) / 100.0))
        clock.set_text(f"t = {t - t_start:5.1f} s   ({rate})")

    if args.frames:
        os.makedirs(args.frames, exist_ok=True)
        for i, t in enumerate(times):
            prev = times[i - 1] if i else t
            rate = "paused" if t == prev and i else f"x{(t - prev) * args.fps:.0f}"
            draw(t, rate)
            fig.savefig(os.path.join(args.frames, f"{i:05d}.png"), dpi=100, facecolor=SURFACE)
        with open(os.path.join(args.frames, "fps.txt"), "w") as f:
            f.write(str(args.fps))
        print(f"{len(times)} 格 → {args.frames}（{len(times) / args.fps:.0f} s 影片）")
    else:
        from matplotlib.animation import FuncAnimation, PillowWriter
        anim = FuncAnimation(fig, lambda i: draw(times[i], ""), frames=len(times), blit=False)
        anim.save(os.path.join(args.out_dir, "demo.gif"), writer=PillowWriter(fps=args.fps), dpi=70)
    draw(t_end, "end")
    fig.savefig(os.path.join(args.out_dir, "demo_final.png"), dpi=100, facecolor=SURFACE)


if __name__ == "__main__":
    main()
