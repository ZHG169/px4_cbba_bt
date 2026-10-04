#!/usr/bin/env python3
"""把 cbba_sim 的結果做成 GIF 動畫（matplotlib + Pillow）。

用法：
    animate_results.py mission <run 輸出目錄> <場景檔> [--fps N] [--speedup K]
        依 assignment.csv 讓每台載具沿路徑移動並返航，右側顯示電量下降。
        場景檔裡 tag=fire 的任務以紅色光圈標示，完成前光圈會閃動，完成後變灰。
        最後 5 秒顯示「誰去火災」：每台對每個火點的成本、誰成本最低、誰得標、
        沒得標的原因，以及 CBBA 合作（協商）的結果。需要 run 輸出的 bids.csv。
        輸出 <目錄>/mission.gif

    animate_results.py weights <輸出 GIF> <run 目錄 1> <run 目錄 2> ...
        每個目錄是用不同 --battery-weight 跑的 run，一個權重一格，
        顯示分配如何隨電池權重改變。

圖上的文字使用英文，避免容器裡沒有中文字型時變成方框。
"""
import argparse
import math
import os
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_results import (  # noqa: E402
    AGENT_MARKERS, FIRE, GRID, INK, INK_2, MUTED, SURFACE, TASK_MARKERS,
    agent_color_map, agent_name, cooperation_summary, draw_fire_halo, fire_decisions,
    fire_legend_handle, read_csv, read_task_tags,
)


def read_summary(out_dir):
    path = os.path.join(out_dir, "summary.csv")
    if not os.path.exists(path):
        return {}
    return {r["key"]: float(r["value"]) for r in read_csv(path)}


def read_scenario(path):
    """回傳 {agent_id: (energy_per_meter, hover_energy_per_sec)}。"""
    params = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cols = [c.strip() for c in line.split(",")]
            if cols[0] == "agent":
                params[int(cols[1])] = (float(cols[8]), float(cols[9]))
    return params


def load_run(out_dir):
    agents = read_csv(os.path.join(out_dir, "agents.csv"))
    tasks = read_csv(os.path.join(out_dir, "tasks.csv"))
    legs = defaultdict(list)
    for row in read_csv(os.path.join(out_dir, "assignment.csv")):
        legs[int(row["agent_id"])].append(row)
    for rows in legs.values():
        rows.sort(key=lambda r: int(r["order"]))
    return agents, tasks, legs


def build_track(agent, legs, epm, hover):
    """每台載具的分段軌跡：[(t0, t1, p0, p1, e0, e1)]，e 為累計耗能。

    去程的時間直接用 assignment.csv 的到達/完成時間，返航用最後一段的實際速度。
    """
    home = (float(agent["x"]), float(agent["y"]))
    pos, t, energy, speed = home, 0.0, 0.0, None
    segs = []
    for leg in legs:
        nxt = (float(leg["x"]), float(leg["y"]))
        arrival, completion = float(leg["arrival"]), float(leg["completion"])
        dist = math.dist(pos, nxt)
        e_travel = energy + dist * epm
        segs.append((t, arrival, pos, nxt, energy, e_travel))
        if arrival > t:
            speed = dist / (arrival - t) if dist > 0 else speed
        e_done = e_travel + (completion - arrival) * hover
        segs.append((arrival, completion, nxt, nxt, e_travel, e_done))
        pos, t, energy = nxt, completion, e_done
    if segs:
        dist = math.dist(pos, home)
        dt = dist / speed if speed else 0.0
        segs.append((t, t + dt, pos, home, energy, energy + dist * epm))
    return segs


def state_at(segs, home, now):
    if not segs:
        return home, 0.0
    for t0, t1, p0, p1, e0, e1 in segs:
        if now <= t1:
            f = 0.0 if t1 <= t0 else max(0.0, (now - t0) / (t1 - t0))
            return ((p0[0] + (p1[0] - p0[0]) * f, p0[1] + (p1[1] - p0[1]) * f),
                    e0 + (e1 - e0) * f)
    return segs[-1][3], segs[-1][5]


def draw_static_map(ax, agents, tasks, legs, colors, faded_paths=True):
    ax.set_aspect("equal", adjustable="box")
    for a in agents:
        aid = int(a["agent_id"])
        pts = [(float(a["x"]), float(a["y"]))] + [(float(l["x"]), float(l["y"]))
                                                  for l in legs.get(aid, [])]
        if len(pts) > 1:
            pts.append(pts[0])
            xs, ys = zip(*pts)
            ax.plot(xs, ys, color=colors[aid], lw=1.0, ls=(0, (2, 3)),
                    alpha=0.45 if faded_paths else 0.9, zorder=1)
        ax.scatter([pts[0][0]], [pts[0][1]], marker="s", s=60, color="none",
                   edgecolors=colors[aid], linewidths=1.2, zorder=2)
    xs = [float(t["x"]) for t in tasks] + [float(a["x"]) for a in agents]
    ys = [float(t["y"]) for t in tasks] + [float(a["y"]) for a in agents]
    pad = 0.15 * max(max(xs) - min(xs), max(ys) - min(ys), 10)
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


# ---- 1. 任務執行動畫 ----------------------------------------------------------
def animate_mission(out_dir, scenario, fps, speedup):
    agents, tasks, legs = load_run(out_dir)
    summary = read_summary(out_dir)
    params = read_scenario(scenario)
    ids = [int(a["agent_id"]) for a in agents]
    colors = agent_color_map(ids)
    homes = {int(a["agent_id"]): (float(a["x"]), float(a["y"])) for a in agents}
    tracks = {int(a["agent_id"]): build_track(a, legs.get(int(a["agent_id"]), []),
                                              *params[int(a["agent_id"])]) for a in agents}
    completion = {int(r["task_id"]): float(r["completion"])
                  for rows in legs.values() for r in rows}
    t_end = max((s[-1][1] for s in tracks.values() if s), default=1.0)

    fig = plt.figure(figsize=(11, 6.2))
    gs = fig.add_gridspec(1, 2, width_ratios=[2.1, 1], wspace=0.25)
    ax = fig.add_subplot(gs[0])
    axb = fig.add_subplot(gs[1])
    draw_static_map(ax, agents, tasks, legs, colors)
    tags = read_task_tags(scenario)

    task_art, halos = {}, {}
    for t in tasks:
        tid, winner = int(t["task_id"]), int(t["winner"])
        x, y = float(t["x"]), float(t["y"])
        fire = tags.get(tid) == "fire"
        if fire:
            halos[tid] = draw_fire_halo(ax, x, y)
        face = colors.get(winner, SURFACE)
        art = ax.scatter([x], [y], marker=TASK_MARKERS.get(t["type"], "o"), s=150,
                         color=face, edgecolors=MUTED if not winner else SURFACE,
                         linewidths=1.8, zorder=4)
        label = ("FIRE " if fire else "") + f"T{tid}" + ("" if winner else " (none)")
        ax.annotate(label, (x, y), xytext=(10, 8), textcoords="offset points", fontsize=8,
                    color=FIRE if fire else (INK if winner else MUTED), zorder=5,
                    fontweight="bold" if fire else "normal")
        task_art[tid] = art
    if halos:
        ax.legend(handles=[fire_legend_handle()], loc="lower right", fontsize=8)

    trails, markers = {}, {}
    for a in agents:
        aid = int(a["agent_id"])
        trails[aid], = ax.plot([], [], color=colors[aid], lw=2.2, zorder=3)
        markers[aid] = ax.scatter([homes[aid][0]], [homes[aid][1]],
                                  marker=AGENT_MARKERS.get(a["type"], "^"), s=220,
                                  color=colors[aid], edgecolors=INK, linewidths=1.1, zorder=6)
    clock = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top", fontsize=11,
                    fontweight="bold", color=INK,
                    bbox=dict(facecolor=SURFACE, edgecolor=GRID, pad=3))
    state = (f"converged {summary['convergence_time']:.2f} s"
             if summary.get("converged") else "NOT converged")
    ax.set_title(f"Mission replay — loss {summary.get('loss', 0) * 100:.0f}%, {state}, "
                 f"battery weight {summary.get('battery_weight', 1):g}")

    # 電量長條圖
    names = [agent_name(a["type"], int(a["agent_id"])) for a in agents]
    start = [float(a["battery"]) for a in agents]
    reserve = [float(a["safety_reserve"]) for a in agents]
    ypos = list(range(len(agents)))[::-1]
    axb.barh(ypos, start, color=GRID, height=0.55, zorder=1)
    bars = axb.barh(ypos, start, color=[colors[i] for i in ids], height=0.55, zorder=2)
    for y, r in zip(ypos, reserve):
        axb.plot([r, r], [y - 0.38, y + 0.38], color=INK, lw=1.4, zorder=3)
    axb.set_yticks(ypos, names)
    axb.set_xlim(0, max(start) * 1.25)
    axb.set_xlabel("battery (black tick = safety reserve)")
    axb.set_title("Battery")
    axb.grid(axis="y", visible=False)
    vals = [axb.text(s + max(start) * 0.02, y, "", va="center", fontsize=9, color=INK_2)
            for s, y in zip(start, ypos)]

    mission_frames = max(2, int(math.ceil(t_end / speedup * fps))) + fps  # 結尾停 1 秒
    decision_frames = 5 * fps if halos else 0
    n_frames = mission_frames + decision_frames
    decision_art = []

    def show_decision():
        """最後的決策畫面：地圖上把得標者到火點的實際路線標紅，右側列出每台的成本與原因。"""
        decisions = fire_decisions(out_dir, tags)
        for d in decisions:
            if d["winner"] is None:
                continue
            rows = legs.get(d["winner"], [])
            pts = [homes[d["winner"]]]
            for r in rows:
                pts.append((float(r["x"]), float(r["y"])))
                if int(r["task_id"]) == d["task_id"]:
                    break
            decision_art.extend(ax.plot([p[0] for p in pts], [p[1] for p in pts], color=FIRE,
                                        lw=3.0, ls=(0, (5, 3)), zorder=7))
        axb.clear()
        axb.axis("off")
        axb.set_title("Who goes to the fire?", color=FIRE)
        y = 1.0
        for d in decisions:
            names = {r["agent_id"]: r["name"] for r in d["rows"]}
            axb.text(0.0, y, f"FIRE T{d['task_id']}  ->  {names.get(d['winner'], 'nobody')}",
                     transform=axb.transAxes, va="top", fontsize=10.5, fontweight="bold",
                     color=FIRE)
            y -= 0.065
            for r in sorted(d["rows"], key=lambda r: (r["cost"] is None, r["cost"] or 0)):
                cost = "  --" if r["cost"] is None else f"{r['cost']:5.1f}"
                low = "  lowest cost" if r["agent_id"] == d["lowest"] else ""
                axb.text(0.03, y, f"{r['name']:<5} cost {cost}{low}", transform=axb.transAxes,
                         va="top", fontsize=9, family="monospace", color=colors[r["agent_id"]],
                         fontweight="bold")
                y -= 0.048
                axb.text(0.10, y, ("-> " if r["win"] else "") + r["reason"],
                         transform=axb.transAxes, va="top", fontsize=8.5,
                         color=FIRE if r["win"] else INK_2,
                         fontweight="bold" if r["win"] else "normal")
                y -= 0.055
            y -= 0.02
        head, _ = cooperation_summary(out_dir, decisions)
        axb.text(0.0, max(y, 0.0), "Cooperation (CBBA consensus):\n" + head.replace(", ", "\n")
                 + "\ncost = only this task, lower is better\nred dashed = winner's route",
                 transform=axb.transAxes, va="top", fontsize=8.5, color=INK_2)

    def update(frame):
        if frame >= mission_frames:
            if not decision_art:
                show_decision()
            return []
        now = min(t_end, frame * speedup / fps)
        for a, bar, txt, s0 in zip(agents, bars, vals, start):
            aid = int(a["agent_id"])
            (x, y), used = state_at(tracks[aid], homes[aid], now)
            markers[aid].set_offsets([[x, y]])
            pts = [homes[aid]]
            for t0, t1, p0, p1, _, _ in tracks[aid]:
                if now >= t1:
                    pts.append(p1)
                elif now > t0:
                    pts.append((x, y))
                    break
            trails[aid].set_data([p[0] for p in pts], [p[1] for p in pts])
            bar.set_width(s0 - used)
            txt.set_text(f"{s0 - used:.1f}")
        for tid, art in task_art.items():
            if tid in completion and now >= completion[tid]:
                art.set_edgecolors(INK)
                art.set_alpha(0.35)
        for tid, halo in halos.items():
            if tid in completion and now >= completion[tid]:
                halo.set_sizes([520])                     # 已處理：光圈變灰
                halo.set_edgecolors([MUTED])
                halo.set_facecolors([(0.5, 0.5, 0.5, 0.10)])
            else:
                halo.set_sizes([520 + 260 * (0.5 + 0.5 * math.sin(frame * 0.9))])
        clock.set_text(f"t = {now:5.1f} s")
        return []

    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    path = os.path.join(out_dir, "mission.gif")
    anim.save(path, writer=PillowWriter(fps=fps), dpi=90)
    plt.close(fig)
    print(f"  wrote {path}")


# ---- 2. 電池權重逐格比較 -------------------------------------------------------
def animate_weights(gif_path, run_dirs, hold_sec):
    runs = []
    for d in run_dirs:
        agents, tasks, legs = load_run(d)
        runs.append((read_summary(d), agents, tasks, legs))
    ids = [int(a["agent_id"]) for a in runs[0][1]]
    colors = agent_color_map(ids)

    fig = plt.figure(figsize=(10.5, 5.6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.6, 1], wspace=0.28)

    def update(i):
        fig.clf()
        ax = fig.add_subplot(gs[0])
        axb = fig.add_subplot(gs[1])
        summary, agents, tasks, legs = runs[i]
        draw_static_map(ax, agents, tasks, legs, colors, faded_paths=False)
        for a in agents:
            aid = int(a["agent_id"])
            x, y = float(a["x"]), float(a["y"])
            for leg in legs.get(aid, []):
                nx, ny = float(leg["x"]), float(leg["y"])
                ax.annotate("", xy=(nx, ny), xytext=(x, y), zorder=2,
                            arrowprops=dict(arrowstyle="-|>", color=colors[aid], lw=2.2,
                                            shrinkA=8, shrinkB=8, mutation_scale=13))
                x, y = nx, ny
            ax.scatter([float(a["x"])], [float(a["y"])], marker=AGENT_MARKERS.get(a["type"], "^"),
                       s=240, color=colors[aid], edgecolors=INK, linewidths=1.1, zorder=6)
            ax.annotate(f"{agent_name(a['type'], aid)}\nbattery {float(a['battery']):.0f}",
                        (float(a["x"]), float(a["y"])), xytext=(0, -16),
                        textcoords="offset points", ha="center", va="top", fontsize=8.5,
                        fontweight="bold")
        for t in tasks:
            winner = int(t["winner"])
            ax.scatter([float(t["x"])], [float(t["y"])], marker=TASK_MARKERS.get(t["type"], "o"),
                       s=160, color=colors.get(winner, SURFACE),
                       edgecolors=SURFACE if winner else MUTED, linewidths=1.8, zorder=4)
            ax.annotate(f"T{t['task_id']}", (float(t["x"]), float(t["y"])), xytext=(8, 6),
                        textcoords="offset points", fontsize=8.5)
        ax.set_title(f"Battery weight = {summary.get('battery_weight', 0):g}", fontsize=13)

        names = [agent_name(a["type"], int(a["agent_id"])) for a in agents]
        start = [float(a["battery"]) for a in agents]
        after = [float(a["battery_after"]) for a in agents]
        ntask = [int(a["num_tasks"]) for a in agents]
        ypos = list(range(len(agents)))[::-1]
        axb.barh(ypos, start, color=GRID, height=0.55)
        axb.barh(ypos, after, color=[colors[int(a["agent_id"])] for a in agents], height=0.55)
        for y, a, s, n in zip(ypos, agents, start, ntask):
            r = float(a["safety_reserve"])
            axb.plot([r, r], [y - 0.38, y + 0.38], color=INK, lw=1.4)
            axb.text(s + max(start) * 0.02, y, f"{float(a['battery_after']):.1f} left\n{n} task(s)",
                     va="center", fontsize=9, color=INK_2)
        axb.set_yticks(ypos, names)
        axb.set_xlim(0, max(start) * 1.45)
        axb.set_xlabel("battery after returning home (grey = start)")
        axb.set_title("Battery after mission")
        axb.grid(axis="y", visible=False)
        return []

    anim = FuncAnimation(fig, update, frames=len(runs), blit=False)
    anim.save(gif_path, writer=PillowWriter(fps=1.0 / hold_sec), dpi=90)
    plt.close(fig)
    print(f"  wrote {gif_path}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("mission")
    m.add_argument("out_dir")
    m.add_argument("scenario")
    m.add_argument("--fps", type=int, default=10)
    m.add_argument("--speedup", type=float, default=8.0, help="模擬秒數 / 影片秒數")
    w = sub.add_parser("weights")
    w.add_argument("gif")
    w.add_argument("run_dirs", nargs="+")
    w.add_argument("--hold", type=float, default=1.5, help="每格停留秒數")
    args = p.parse_args()
    if args.cmd == "mission":
        animate_mission(args.out_dir, args.scenario, args.fps, args.speedup)
    else:
        animate_weights(args.gif, args.run_dirs, args.hold)


if __name__ == "__main__":
    main()
