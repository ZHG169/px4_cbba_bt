#!/usr/bin/env python3
"""100% 斷網的時間軸圖（2026-11-09）：itest demo 的 blackout 情境 → 同目錄的 blackout_timeline.png。

  python3 docker/itest/plot_blackout.py docker/itest/out/<時間>_demo

每台一列（uav1～N、dog1～N），橫條是那段時間手上拿著的 CBBA 任務（藍 = 空中任務、橘 = 火警／地面任務），
灰底是 100% 斷網的時段，✓ 是那台回報完成、× 是被取消（任務從手上撤掉、自己沒回報完成）。
時間以斷網開始為 0。另外標出：鄰居判定失聯（狗的 cbba_node log「持有者 … 失聯」）、失聯接手（「失聯接手」）、恢復連線。
資料：events.jsonl（無人機的 assigned／result、狗的 dog_assigned）、demo_plan.json（blackout、任務名稱）、
dog_cbba_<ns>.log（狗回報成功、失聯、接手的時刻）。要 matplotlib（主機沒有時在 sim 容器跑）。
"""
import json
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

# 和 plot_loss.py 同一組（dataviz 的參考調色盤，已驗證色盲可分辨）
AIR, GROUND = "#2a78d6", "#eb6834"
SURFACE, INK, MUTED, GRID, SHADE = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df", "#d9d8d4"
GROUND_TYPE = 2
LOG_LINE = re.compile(r"\[(\d+\.\d+)\] \[[^\]]+\]: (.*)")


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    out_dir = sys.argv[1]
    plan = json.load(open(os.path.join(out_dir, "demo_plan.json"), encoding="utf-8"))
    b = plan.get("blackout")
    if not b:
        sys.exit("demo_plan.json 沒有 blackout（不是斷網情境的輸出）")
    cut, back = b["t0"], b["t1"]
    events = [json.loads(l) for l in open(os.path.join(out_dir, "events.jsonl"), encoding="utf-8") if l.strip()]
    dogs = sorted(plan.get("dog") or [], key=lambda d: d["id"])
    dog_name = {d["id"]: f"dog{i + 1}" for i, d in enumerate(dogs)}
    uavs = sorted({e["uav"] for e in events if e["kind"] == "assigned" and isinstance(e.get("uav"), int)})
    rows = [f"uav{v}" for v in uavs] + [dog_name[d["id"]] for d in dogs]

    # 任務名稱與種類：固定／插入任務照 demo_plan 的點名；地面任務照起火順序叫 F1、F2…
    name = {p["task_id"]: p["point"] for p in plan.get("patrol", []) if "task_id" in p}
    ground = sorted((e for e in events if e["kind"] == "new_task" and e["msg"].get("type") == GROUND_TYPE),
                    key=lambda e: e["t"])
    fires = sorted((f for f in plan.get("fires", []) if f.get("spawned")), key=lambda f: f["spawned"])
    for e in ground:
        p = e["msg"]["position"]
        k = next((i for i, f in enumerate(fires) if abs(f["x"] - p["x"]) < 1.0 and abs(f["y"] - p["y"]) < 1.0), None)
        name.setdefault(e["msg"]["task_id"], f"F{k + 1}" if k is not None else "F")
    is_ground = {e["msg"]["task_id"] for e in ground}

    # 每台手上的任務（current_task 的變化）→ 區段
    changes = {r: [] for r in rows}
    for e in events:
        if e["kind"] == "assigned" and f"uav{e['uav']}" in changes:
            changes[f"uav{e['uav']}"].append((e["t"], e["msg"].get("task_id", 0)))
        elif e["kind"] == "dog_assigned" and e.get("dog") in dog_name:
            changes[dog_name[e["dog"]]].append((e["t"], e["msg"].get("task_id", 0)))
    t_end = max(e["t"] for e in events)

    # 回報完成：無人機看 task_result；狗看自己 cbba_node 的 log
    done = {r: [] for r in rows}
    for e in events:
        if e["kind"] == "result" and e["msg"].get("success") is True and f"uav{e['uav']}" in done:
            done[f"uav{e['uav']}"].append((e["t"], e["msg"].get("task_id")))
    marks = []        # (t, row, 文字)
    for d in dogs:
        path = os.path.join(out_dir, f"dog_cbba_{d['ns']}.log")
        if not os.path.exists(path):
            continue
        who = dog_name[d["id"]]
        lost_seen = takeover_seen = False
        for line in open(path, encoding="utf-8", errors="replace"):
            m = LOG_LINE.search(line)
            if not m:
                continue
            t, text = float(m.group(1)), m.group(2)
            ok = re.search(r"task_result ([0-9A-F]{8})：success 1", text)
            if ok:
                done[who].append((t, int(ok.group(1), 16)))
            if cut <= t <= back and "失聯：先鎖著" in text and not lost_seen:
                lost_seen = True
                marks.append((t, who, f"lost detected +{t - cut:.1f} s"))
            if cut <= t <= back and text.startswith("失聯接手") and not takeover_seen:
                takeover_seen = True
                marks.append((t, who, f"takeover +{t - cut:.1f} s"))

    t0 = cut - 12.0
    t1 = min(t_end, back + 30.0)
    fig, ax = plt.subplots(figsize=(13, 0.62 * len(rows) + 2.2), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    ax.axvspan(0, back - cut, color=SHADE, zorder=0)
    ax.text((back - cut) / 2, len(rows) - 0.35, f"100% blackout ({back - cut:.0f} s)", ha="center", va="bottom",
            color=MUTED, fontsize=10)
    for i, r in enumerate(rows):
        y = len(rows) - 1 - i
        seq = sorted(changes[r]) + [(t_end, 0)]
        for (ta, task), (tb, _) in zip(seq, seq[1:]):
            if not task or tb < t0 or ta > t1:
                continue
            a, z = max(ta, t0) - cut, min(tb, t1) - cut
            color = GROUND if task in is_ground else AIR
            ax.barh(y, z - a, left=a, height=0.5, color=color, edgecolor=SURFACE, linewidth=2, zorder=2)
            if z - a > 2.5:
                ax.text(a + 0.4, y, name.get(task, f"{task:08X}"), va="center", ha="left", color="white",
                        fontsize=9, fontweight="bold", zorder=3)
            finished = any(task == tk and ta - 0.5 <= tt <= tb + 0.5 for tt, tk in done[r])
            if tb < t_end and tb <= t1 and not finished:
                ax.plot([tb - cut], [y], marker="x", markersize=10, color=INK, markeredgewidth=2, zorder=4)
        for tt, tk in done[r]:
            if t0 <= tt <= t1:
                ax.plot([tt - cut], [y], marker="$✓$", markersize=13, color=INK, zorder=4)
    for t, r, text in marks:
        y = len(rows) - 1 - rows.index(r)
        ax.annotate(text, (t - cut, y + 0.25), xytext=(0, 10), textcoords="offset points", ha="center",
                    color=MUTED, fontsize=8, arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
    for x, text in ((0, "blackout"), (back - cut, "restored")):
        ax.axvline(x, color=MUTED, linewidth=1, linestyle="--", zorder=1)

    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(list(reversed(rows)))
    ax.set_xlim(t0 - cut, t1 - cut)
    ax.set_ylim(-0.7, len(rows) + 0.2)
    ax.set_xlabel("time since blackout started (s)", color=MUTED)
    ax.set_title("Task execution under 100% network blackout", color=INK, fontsize=12, loc="left")
    ax.grid(axis="x", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.legend(handles=[Patch(color=AIR, label="air task (held)"), Patch(color=GROUND, label="fire / ground task (held)"),
                       Line2D([], [], marker="$✓$", color=INK, linestyle="", markersize=11, label="reported done"),
                       Line2D([], [], marker="x", color=INK, linestyle="", markersize=9, markeredgewidth=2,
                              label="cancelled"),
                       Patch(color=SHADE, label="100% blackout")],
              frameon=False, labelcolor=INK, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=5, fontsize=9)
    fig.tight_layout()
    out = os.path.join(out_dir, "blackout_timeline.png")
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(out)


if __name__ == "__main__":
    main()
