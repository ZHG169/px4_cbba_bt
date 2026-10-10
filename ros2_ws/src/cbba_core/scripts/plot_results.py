#!/usr/bin/env python3
"""把 cbba_sim 輸出的 CSV 畫成圖（matplotlib）。

用法：
    plot_results.py <輸出目錄> [--scenario 場景檔] [--show]

目錄裡有哪些 CSV 就畫哪些圖，存成同一個目錄下的 PNG：
    tasks.csv + agents.csv + assignment.csv  ->  allocation.png  任務分配地圖
                                                （加 --scenario 場景檔時，tag=fire 的任務以紅色標示）
    timeline.csv                             ->  timeline.png    協商過程
    sweep.csv                                ->  sweep.png       各丟包率的收斂統計
    weights.csv                              ->  weights.png     電池權重的影響
    bids.csv（加 --scenario 且有火點）         ->  fire_decision.png  誰去火點、誰成本最低、為什麼

圖上的文字使用英文，避免容器裡沒有中文字型時變成方框。
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

import matplotlib

if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

# ---- 樣式 -------------------------------------------------------------------
# 載具顏色固定依編號排序指定，四色之間已檢查過色盲可辨識；
# 載具另外用形狀和文字標示，不只靠顏色區分。
AGENT_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7"]
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#8a8984"
GRID = "#e6e5e1"
TASK_MARKERS = {"AIR_RECON": "o", "GROUND_INTERVENTION": "s", "PATROL": "D"}
TASK_LABELS = {"AIR_RECON": "air recon", "GROUND_INTERVENTION": "ground intervention",
               "PATROL": "patrol"}
AGENT_MARKERS = {"UAV": "^", "UGV": "P"}
# 火點：紅色光圈＋紅字 FIRE（不只靠顏色，和橘色的 UAV2 也分得開）
FIRE = "#c8161d"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "axes.titlecolor": INK,
    "xtick.color": INK_2, "ytick.color": INK_2, "text.color": INK,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.labelsize": 9.5, "xtick.labelsize": 9, "ytick.labelsize": 9,
    "legend.fontsize": 9, "legend.frameon": False, "font.size": 9.5,
    "lines.linewidth": 2.0, "lines.markersize": 7,
})


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def read_task_tags(scenario):
    """從場景檔讀 task 的選用第 10 欄 tag，回傳 {task_id: tag}。沒給場景檔時為空。"""
    tags = {}
    if not scenario:
        return tags
    with open(scenario) as f:
        for line in f:
            cols = [c.strip() for c in line.strip().split(",")]
            if cols[0] == "task" and len(cols) > 9 and cols[9]:
                tags[int(cols[1])] = cols[9].lower()
    return tags


def draw_fire_halo(ax, x, y, size=520):
    """在火點任務後面畫紅色光圈（半透明填色、實心外框），回傳 artist（動畫可調整）。"""
    return ax.scatter([x], [y], marker="o", s=size, facecolors=[(0.78, 0.09, 0.11, 0.18)],
                      edgecolors=FIRE, linewidths=2.2, zorder=3)


def fire_legend_handle():
    return Line2D([], [], marker="o", color="none", markerfacecolor=(0.78, 0.09, 0.11, 0.18),
                  markeredgecolor=FIRE, markeredgewidth=2.0, markersize=15,
                  label="fire point (red halo)")


def fire_decisions(out_dir, tags):
    """整理每個火點的決策：每台載具的成本、出價、是否得標與原因。

    回傳 [{task_id, winner, lowest, rows:[{agent_id, name, cost, score, win, reason}]}]，
    cost 用「只做這一個任務」的成本（無法完成時為 None），方便直接比較誰離火點最近。
    """
    path = os.path.join(out_dir, "bids.csv")
    fires = sorted(t for t, tag in tags.items() if tag == "fire")
    if not fires or not os.path.exists(path):
        return []
    agents = {int(a["agent_id"]): a for a in read_csv(os.path.join(out_dir, "agents.csv"))}
    paths = defaultdict(list)
    for r in sorted(read_csv(os.path.join(out_dir, "assignment.csv")),
                    key=lambda r: int(r["order"])):
        paths[int(r["agent_id"])].append(int(r["task_id"]))
    bids = defaultdict(list)
    for r in read_csv(path):
        bids[int(r["task_id"])].append(r)

    out = []
    for tid in fires:
        rows, winner = [], None
        for r in bids.get(tid, []):
            aid = int(r["agent_id"])
            a = agents[aid]
            cost = float(r["alone_cost"]) if r["alone_feasible"] == "1" else None
            win = r["winner"] == "1"
            others = [t for t in paths[aid] if t != tid]
            if win:
                winner = aid
                reason = "WINNER"
            elif r["compatible"] != "1":
                reason = "cannot do this task type"
            elif cost is None:
                reason = "not enough battery to fly there and back"
            elif r["feasible"] != "1":
                busy = ", ".join(f"{'FIRE ' if tags.get(t) == 'fire' else ''}T{t}" for t in others)
                reason = f"battery already used for {busy}" if busy else "not enough battery"
            else:
                reason = "lower bid than the winner"
            rows.append(dict(agent_id=aid, name=agent_name(a["type"], aid), cost=cost,
                             score=float(r["alone_score"]), bid=float(r["score"]),
                             win=win, reason=reason))
        feasible = [r for r in rows if r["cost"] is not None]
        lowest = min(feasible, key=lambda r: r["cost"])["agent_id"] if feasible else None
        out.append(dict(task_id=tid, winner=winner, lowest=lowest, rows=rows))
    return out


def cooperation_summary(out_dir, decisions):
    """一句話說明 CBBA 合作的結果（協商時間、廣播數、火點分工）。"""
    summary_path = os.path.join(out_dir, "summary.csv")
    summary = ({r["key"]: float(r["value"]) for r in read_csv(summary_path)}
               if os.path.exists(summary_path) else {})
    names = {r["agent_id"]: r["name"] for d in decisions for r in d["rows"]}
    split = ", ".join(f"FIRE T{d['task_id']} -> {names.get(d['winner'], 'nobody')}"
                      for d in decisions)
    head = ""
    if summary:
        head = (f"{int(summary['broadcasts'])} broadcasts, agreed in "
                f"{summary['convergence_time']:.2f} s, {int(summary['duplicates'])} duplicates")
    return head, split


def agent_color_map(agent_ids):
    ids = sorted(set(agent_ids))
    return {a: AGENT_COLORS[i % len(AGENT_COLORS)] for i, a in enumerate(ids)}


def agent_name(agent_type, agent_id):
    return f"{agent_type}{agent_id}"


def save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"  wrote {path}")


# ---- 1. 任務分配地圖 ----------------------------------------------------------
def plot_allocation(out_dir, scenario=None):
    agents = read_csv(os.path.join(out_dir, "agents.csv"))
    tasks = read_csv(os.path.join(out_dir, "tasks.csv"))
    assignment = read_csv(os.path.join(out_dir, "assignment.csv"))
    summary = {}
    summary_path = os.path.join(out_dir, "summary.csv")
    if os.path.exists(summary_path):
        summary = {r["key"]: float(r["value"]) for r in read_csv(summary_path)}
    tags = read_task_tags(scenario)

    colors = agent_color_map(int(a["agent_id"]) for a in agents)
    legs = defaultdict(list)
    for row in assignment:
        legs[int(row["agent_id"])].append(row)
    for rows in legs.values():
        rows.sort(key=lambda r: int(r["order"]))

    fig, ax = plt.subplots(figsize=(9.5, 7.2))
    ax.set_aspect("equal", adjustable="datalim")

    # 路徑：實線為去程，細虛線為返航
    for a in agents:
        aid = int(a["agent_id"])
        x, y = float(a["x"]), float(a["y"])
        home = (x, y)
        for leg in legs.get(aid, []):
            nx, ny = float(leg["x"]), float(leg["y"])
            ax.annotate("", xy=(nx, ny), xytext=(x, y), zorder=2,
                        arrowprops=dict(arrowstyle="-|>", color=colors[aid], lw=2.0,
                                        shrinkA=7, shrinkB=8, mutation_scale=12))
            x, y = nx, ny
        if legs.get(aid):
            ax.plot([x, home[0]], [y, home[1]], color=colors[aid], lw=1.0, ls=(0, (2, 3)),
                    alpha=0.7, zorder=1)

    # 任務：形狀表示類型，填色表示得標的載具，空心表示無人認領
    arrival = {int(r["task_id"]): r for r in assignment}
    for t in tasks:
        tid, winner = int(t["task_id"]), int(t["winner"])
        x, y = float(t["x"]), float(t["y"])
        face = colors.get(winner, SURFACE)
        edge = SURFACE if winner else MUTED
        fire = tags.get(tid) == "fire"
        if fire:
            draw_fire_halo(ax, x, y)
        ax.scatter([x], [y], marker=TASK_MARKERS.get(t["type"], "o"), s=150, color=face,
                   edgecolors=edge, linewidths=1.8 if not winner else 2.0, zorder=4)
        label = f"FIRE T{tid}" if fire else f"T{tid}"
        if tid in arrival:
            leg = arrival[tid]
            label += f"  #{leg['order']} · {float(leg['arrival']):.0f}s"
            if leg["overdue"] == "1":
                label += " (late)"
        else:
            label += "  unassigned"
        ax.annotate(label, (x, y), xytext=(9, 8), textcoords="offset points", fontsize=8.5,
                    color=FIRE if fire else (INK if tid in arrival else MUTED), zorder=5,
                    fontweight="bold" if fire else "normal",
                    bbox=dict(facecolor=SURFACE, edgecolor="none", pad=1.2, alpha=0.85))

    # 載具：標示電量（出發 -> 做完並返航後）
    for a in agents:
        aid = int(a["agent_id"])
        x, y = float(a["x"]), float(a["y"])
        ax.scatter([x], [y], marker=AGENT_MARKERS.get(a["type"], "^"), s=240, color=colors[aid],
                   edgecolors=INK, linewidths=1.2, zorder=6)
        text = (f"{agent_name(a['type'], aid)}\nbattery {float(a['battery']):.0f} → "
                f"{float(a['battery_after']):.0f}")
        ax.annotate(text, (x, y), xytext=(0, -14), textcoords="offset points", ha="center",
                    va="top", fontsize=8.5, color=INK, fontweight="bold", zorder=7,
                    bbox=dict(facecolor=SURFACE, edgecolor="none", pad=1.2, alpha=0.85))

    handles = [Line2D([], [], marker=AGENT_MARKERS.get(a["type"], "^"), color=colors[int(a["agent_id"])],
                      markeredgecolor=INK, markersize=10, lw=2,
                      label=agent_name(a["type"], int(a["agent_id"]))) for a in agents]
    for ttype in sorted({t["type"] for t in tasks}):
        handles.append(Line2D([], [], marker=TASK_MARKERS.get(ttype, "o"), color="none",
                              markerfacecolor=MUTED, markeredgecolor=SURFACE, markersize=9,
                              label=f"task: {TASK_LABELS.get(ttype, ttype)}"))
    if "fire" in tags.values():
        handles.append(fire_legend_handle())
    handles.append(Line2D([], [], marker="o", color="none", markerfacecolor=SURFACE,
                          markeredgecolor=MUTED, markersize=9, label="unassigned task"))
    handles.append(Line2D([], [], color=MUTED, lw=1.0, ls=(0, (2, 3)), label="return to home"))
    ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1.01, 0.5))

    title = "Task allocation"
    if summary:
        state = (f"converged in {summary['convergence_time']:.2f} s"
                 if summary.get("converged") else "NOT converged")
        title += (f" — packet loss {summary['loss'] * 100:.0f}%, {state}, "
                  f"battery weight {summary['battery_weight']:g}")
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.margins(x=0.12, y=0.2)
    fig.text(0.125, 0.0, "Labels: task id, order in the vehicle's path, arrival time. "
             "Battery: at start → after finishing the path and returning home.",
             fontsize=8.5, color=INK_2)
    save(fig, out_dir, "allocation.png")
    return fig


# ---- 2. 協商過程 --------------------------------------------------------------
def plot_timeline(out_dir):
    rows = read_csv(os.path.join(out_dir, "timeline.csv"))
    t = [float(r["time"]) for r in rows]
    dis = [int(r["disagreements"]) for r in rows]
    bro = [int(r["broadcasts"]) for r in rows]
    summary_path = os.path.join(out_dir, "summary.csv")
    summary = ({r["key"]: float(r["value"]) for r in read_csv(summary_path)}
               if os.path.exists(summary_path) else {})

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5.2), sharex=True,
                                   gridspec_kw=dict(hspace=0.25))
    ax1.step(t, dis, where="post", color=AGENT_COLORS[0])
    ax1.fill_between(t, dis, step="post", color=AGENT_COLORS[0], alpha=0.12, linewidth=0)
    ax1.set_ylabel("tasks in disagreement")
    ax1.set_title("Negotiation: tasks on which the vehicles disagree about the winner")
    ax1.set_ylim(bottom=0)
    ax1.yaxis.get_major_locator().set_params(integer=True)

    ax2.step(t, bro, where="post", color=AGENT_COLORS[0])
    ax2.set_ylabel("broadcasts (cumulative)")
    ax2.set_xlabel("time since the tasks were announced (s)")
    ax2.set_title("Messages sent by all vehicles")
    ax2.set_ylim(bottom=0)

    if summary.get("converged"):
        ct = summary["convergence_time"]
        for ax in (ax1, ax2):
            ax.axvline(ct, color=INK_2, lw=1.0, ls=(0, (4, 3)))
        ax1.annotate(f"converged at {ct:.2f} s", (ct, ax1.get_ylim()[1]), xytext=(6, -4),
                     textcoords="offset points", va="top", color=INK_2, fontsize=9)
    save(fig, out_dir, "timeline.png")
    return fig


# ---- 3. 各丟包率的收斂統計 ------------------------------------------------------
def percentile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    return values[min(len(values) - 1, int(round(q * (len(values) - 1))))]


def plot_sweep(out_dir):
    rows = read_csv(os.path.join(out_dir, "sweep.csv"))
    by_loss = defaultdict(list)
    for r in rows:
        by_loss[float(r["loss"])].append(r)
    losses = sorted(by_loss)
    xs = list(range(len(losses)))
    labels = [f"{l * 100:.0f}%" for l in losses]

    rate, med, lo, hi, kbps = [], [], [], [], []
    for l in losses:
        trials = by_loss[l]
        ok = [float(r["time"]) for r in trials if r["converged"] == "1"]
        rate.append(100.0 * len(ok) / len(trials))
        med.append(percentile(ok, 0.5))
        lo.append(percentile(ok, 0.05))
        hi.append(percentile(ok, 0.95))
        kbps.append(sum(float(r["bytes"]) / max(float(r["elapsed"]), 1e-9) for r in trials)
                    / len(trials) / 1000.0)
    n = len(by_loss[losses[0]])

    fig, axes = plt.subplots(1, 3, figsize=(13, 3.9), gridspec_kw=dict(wspace=0.36))
    c = AGENT_COLORS[0]

    ax = axes[0]
    ax.plot(xs, rate, marker="o", color=c)
    ax.axhline(95, color=INK_2, lw=1.0, ls=(0, (4, 3)))
    ax.annotate("required: 95% at 30% loss", (xs[0], 95), xytext=(0, -12),
                textcoords="offset points", color=INK_2, fontsize=8.5)
    ax.set_ylim(0, 112)
    ax.set_ylabel("converged within 5 s (%)")
    ax.set_title("Convergence rate")
    for x, v in zip(xs, rate):
        ax.annotate(f"{v:.0f}%", (x, v), xytext=(0, 7), textcoords="offset points",
                    ha="center", fontsize=8.5, color=INK)

    ax = axes[1]
    err = [[m - a for m, a in zip(med, lo)], [b - m for m, b in zip(med, hi)]]
    ax.errorbar(xs, med, yerr=err, fmt="o", color=c, ecolor=c, elinewidth=1.5, capsize=4)
    ax.set_ylim(bottom=0)
    ax.set_ylabel("convergence time (s)")
    ax.set_title("Convergence time (median, p5–p95)")
    for x, v in zip(xs, med):
        ax.annotate(f"{v:.2f}", (x, v), xytext=(8, 0), textcoords="offset points",
                    va="center", fontsize=8.5, color=INK)

    ax = axes[2]
    ax.plot(xs, kbps, marker="o", color=c)
    ax.set_ylim(bottom=0)
    ax.set_ylabel("traffic, all vehicles (kB/s)")
    ax.set_title("Traffic sent (estimated)")

    for ax in axes:
        ax.set_xticks(xs)
        ax.set_xticklabels(labels)
        ax.set_xlabel("packet loss")
        ax.set_xlim(-0.4, len(xs) - 0.6)
    fig.suptitle(f"CBBA convergence under packet loss — {n} trials per level, delay 50±30 ms",
                 x=0.125, ha="left", fontsize=12, fontweight="bold", y=1.02)
    save(fig, out_dir, "sweep.png")
    return fig


# ---- 4. 電池權重的影響 ---------------------------------------------------------
def plot_weights(out_dir):
    rows = read_csv(os.path.join(out_dir, "weights.csv"))
    weights = sorted({float(r["weight"]) for r in rows})
    xs = list(range(len(weights)))
    labels = [f"{w:g}" for w in weights]
    agents = sorted({(int(r["agent_id"]), r["type"]) for r in rows})
    colors = agent_color_map(a for a, _ in agents)
    cell = {(float(r["weight"]), int(r["agent_id"])): r for r in rows}

    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.0), gridspec_kw=dict(wspace=0.32))

    ax = axes[0]
    for aid, atype in agents:
        ys = [float(cell[(w, aid)]["battery_after"]) for w in weights]
        ax.plot(xs, ys, marker=AGENT_MARKERS.get(atype, "^"), color=colors[aid],
                label=agent_name(atype, aid))
        ax.annotate(agent_name(atype, aid), (xs[-1], ys[-1]), xytext=(7, 0),
                    textcoords="offset points", va="center", fontsize=8.5, color=INK)
    reserve = float(rows[0]["safety_reserve"])
    ax.axhline(reserve, color=INK_2, lw=1.0, ls=(0, (4, 3)))
    ax.annotate("safety reserve", (xs[-1], reserve), xytext=(0, -12), textcoords="offset points",
                ha="right", color=INK_2, fontsize=8.5)
    ax.set_ylim(bottom=0)
    ax.set_ylabel("battery after returning home")
    ax.set_title("Remaining battery per vehicle")

    ax = axes[1]
    for aid, atype in agents:
        ys = [int(cell[(w, aid)]["num_tasks"]) for w in weights]
        ax.plot(xs, ys, marker=AGENT_MARKERS.get(atype, "^"), color=colors[aid],
                label=agent_name(atype, aid))
    ax.set_ylim(bottom=-0.15)
    ax.yaxis.get_major_locator().set_params(integer=True)
    ax.set_ylabel("tasks in the vehicle's path")
    ax.set_title("Tasks taken per vehicle")
    handles, names = ax.get_legend_handles_labels()
    fig.legend(handles, names, loc="upper right", bbox_to_anchor=(0.9, 1.06), ncol=len(names))

    ax = axes[2]
    mean_arrival = []
    for w in weights:
        total, count = 0.0, 0
        for aid, _ in agents:
            r = cell[(w, aid)]
            total += float(r["mean_arrival"]) * int(r["num_tasks"])
            count += int(r["num_tasks"])
        mean_arrival.append(total / count if count else float("nan"))
    ax.plot(xs, mean_arrival, marker="o", color=INK_2)
    ax.set_ylim(bottom=0)
    ax.set_ylabel("mean arrival time (s)")
    ax.set_title("Response time, all assigned tasks")

    for ax in axes:
        ax.set_xticks(xs)
        ax.set_xticklabels(labels)
        ax.set_xlabel("battery weight")
        ax.set_xlim(-0.4, len(xs) - 0.2)
    fig.suptitle("Effect of the battery weight on the allocation", x=0.125, ha="left",
                 fontsize=12, fontweight="bold", y=1.02)
    save(fig, out_dir, "weights.png")
    return fig


# ---- 5. 火點決策 --------------------------------------------------------------
def plot_fire_decision(out_dir, scenario):
    tags = read_task_tags(scenario)
    decisions = fire_decisions(out_dir, tags)
    if not decisions:
        return None
    ids = sorted({r["agent_id"] for d in decisions for r in d["rows"]})
    colors = agent_color_map(ids)
    fig, axes = plt.subplots(1, len(decisions), figsize=(6.2 * len(decisions), 3.9),
                             squeeze=False)
    for ax, d in zip(axes[0], decisions):
        rows = d["rows"]
        ys = list(range(len(rows)))[::-1]
        top = max([r["cost"] for r in rows if r["cost"] is not None] + [1.0])
        for y, r in zip(ys, rows):
            if r["cost"] is None:
                ax.text(top * 0.02, y + 0.12, "X", va="center", fontsize=10, color=MUTED,
                        fontweight="bold")
            else:
                ax.barh(y, r["cost"], height=0.55, color=colors[r["agent_id"]],
                        edgecolor=FIRE if r["win"] else "none", linewidth=2.5,
                        alpha=1.0 if r["win"] else 0.55)
                tag = "  lowest cost" if r["agent_id"] == d["lowest"] else ""
                ax.text(r["cost"] + top * 0.02, y + 0.12, f"{r['cost']:.1f}{tag}", va="center",
                        fontsize=9, color=INK, fontweight="bold" if tag else "normal")
            ax.text(top * 0.02 if r["cost"] is None else r["cost"] + top * 0.02, y - 0.2,
                    r["reason"], va="center", fontsize=8.5,
                    color=FIRE if r["win"] else INK_2, fontweight="bold" if r["win"] else "normal")
        ax.set_yticks(ys, [r["name"] for r in rows])
        ax.set_xlim(0, top * 2.3)
        ax.set_xlabel("cost if this vehicle did only this task (lower is better)")
        ax.set_title(f"FIRE T{d['task_id']}", color=FIRE)
        ax.grid(axis="y", visible=False)
    head, split = cooperation_summary(out_dir, decisions)
    fig.suptitle("Who goes to the fire?", x=0.01, ha="left", fontweight="bold", color=FIRE)
    fig.text(0.01, -0.04, f"Cooperation (CBBA consensus): {head}.   Result: {split}",
             fontsize=9, color=INK_2)
    fig.tight_layout()
    save(fig, out_dir, "fire_decision.png")
    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot cbba_sim results")
    parser.add_argument("out_dir", help="cbba_sim 的輸出目錄")
    parser.add_argument("--scenario", help="場景檔，用來讀任務的 tag（例如 fire）")
    parser.add_argument("--show", action="store_true", help="畫完後開啟視窗顯示")
    args = parser.parse_args()

    if not os.path.isdir(args.out_dir):
        sys.exit(f"找不到目錄：{args.out_dir}")

    def has(*names):
        return all(os.path.exists(os.path.join(args.out_dir, n)) for n in names)

    made = 0
    if has("agents.csv", "tasks.csv", "assignment.csv"):
        plot_allocation(args.out_dir, args.scenario)
        made += 1
    if has("timeline.csv"):
        plot_timeline(args.out_dir)
        made += 1
    if has("sweep.csv"):
        plot_sweep(args.out_dir)
        made += 1
    if has("weights.csv"):
        plot_weights(args.out_dir)
        made += 1
    if has("bids.csv", "agents.csv", "assignment.csv") and args.scenario:
        if plot_fire_decision(args.out_dir, args.scenario):
            made += 1
    if made == 0:
        sys.exit(f"{args.out_dir} 裡沒有 cbba_sim 的輸出，請先執行 cbba_sim")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
