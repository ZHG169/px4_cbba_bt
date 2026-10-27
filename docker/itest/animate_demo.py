#!/usr/bin/env python3
"""把 itest demo（或 uav）記下的事件畫成俯視動畫：無人機飛去自己拿到的任務、隨機火警出現、丟出地面任務。

  python3 docker/itest/animate_demo.py <itest 輸出目錄> --frames <目錄>   每一格存成 PNG（主機用 ffmpeg 合成 MP4；
                                                                         itest.py demo 會自動做）
  python3 docker/itest/animate_demo.py <itest 輸出目錄>                    直接存成 demo.gif（沒有 ffmpeg 時）

要 matplotlib（主機沒有時在 sim 容器跑）。只用 ros_bridge 記下的話題（PX4 位置、current_task、task_result、new_task），
不看節點的 log；有 demo_plan.json（itest demo 寫的）時另外畫出全部任務點、火警起火的時刻。

播放速度：預設全程 1 倍速（2026-10-19 使用者：忽快忽慢沒有意義，原本沒有事件時 3 倍速快轉）；
起火、丟出地面任務、地面任務結束時停 --freeze 秒，結尾停 3 秒。要快轉時給 --speedup（沒有事件時的倍速），
事件前後 --window 秒維持 --slow 倍速。
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
from matplotlib.path import Path  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "ros2_ws", "src", "cbba_core", "scripts"))
from plot_results import AGENT_COLORS, FIRE, GRID, INK, INK_2, MUTED, SURFACE  # noqa: E402

GROUND_TYPE = 2
PATROL = "#3f9b3f"        # 固定巡檢點（綠色平台）
DOG = "#e07000"           # 假的 V60（2026-10-13）
DOG_SHADES = ["#e07000", "#9c4f00", "#f4a259"]   # 多隻狗（2026-10-20）：同色系、深淺不同


def quad_marker():
    """四旋翼的圖示（2026-10-20，使用者：2D 動畫用無人機的圖案、標編號）：機身＋兩支交叉的機臂＋四個旋翼圈"""
    parts = [Path.unit_rectangle().transformed(Affine2D().translate(-0.5, -0.5).scale(0.42))]
    for deg in (45, -45):
        parts.append(Path.unit_rectangle().transformed(Affine2D().translate(-0.5, -0.5).scale(1.5, 0.14).rotate_deg(deg)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(Path.circle((0.53 * sx, 0.53 * sy), 0.3))
    return Path.make_compound_path(*parts)


def dog_marker():
    """機器狗（四足、側面朝右）的圖示：一個不自我重疊的多邊形"""
    v = [(-0.55, 0.30), (0.30, 0.30), (0.30, 0.50), (0.62, 0.50), (0.62, 0.32), (0.78, 0.32), (0.78, 0.18),
         (0.42, 0.18), (0.42, -0.45), (0.30, -0.45), (0.30, -0.05), (0.12, -0.05), (0.12, -0.45), (0.00, -0.45),
         (0.00, -0.05), (-0.26, -0.05), (-0.26, -0.45), (-0.38, -0.45), (-0.38, -0.05), (-0.48, -0.05),
         (-0.48, -0.45), (-0.60, -0.45), (-0.60, 0.20), (-0.80, 0.42), (-0.86, 0.36), (-0.62, 0.30)]
    return Path(v + [v[0]], [Path.MOVETO] + [Path.LINETO] * (len(v) - 1) + [Path.CLOSEPOLY])


def flag_marker():
    """任務點的旗子（2026-10-20，使用者：原本是圓點）：旗桿底端在原點（＝任務點），三角旗朝右上"""
    pole = Path.unit_rectangle().transformed(Affine2D().translate(-0.5, 0.0).scale(0.08, 1.0))
    pennant = Path([(0.04, 1.0), (0.72, 0.8), (0.04, 0.6), (0.04, 1.0)],
                   [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY])
    return Path.make_compound_path(pole, pennant)


QUAD = quad_marker()
DOG_ICON = dog_marker()
FLAG = flag_marker()


ARROW_SPACING = 4.0   # 航跡上每隔幾公尺畫一個方向箭頭（2026-10-20，使用者：路徑加箭頭）
ICON_M = 1.6          # 無人機圖示的寬度（公尺，照地圖比例；2026-10-24）。小場景時最大到原本的大小
MIN_ICON_PX = 9.0     # 圖示最小幾個 px（太小看不見）


def arrow_spots(xs, ys, spacing=ARROW_SPACING, step=0.3):
    """航跡上每隔 spacing 公尺的位置與前進方向 [(x, y, ux, uy)]。
    先把位移不到 step 的點略過（懸停時的幾公分晃動不算路程，否則會在任務點畫出亂指的箭頭）。"""
    if not xs:
        return []
    kept = [(xs[0], ys[0])]
    for x, y in zip(xs[1:], ys[1:]):
        if math.hypot(x - kept[-1][0], y - kept[-1][1]) >= step:
            kept.append((x, y))
    out, walked, nxt = [], 0.0, spacing
    for (x0, y0), (x1, y1) in zip(kept, kept[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        while walked + seg >= nxt:
            r = (nxt - walked) / seg
            out.append((x0 + r * (x1 - x0), y0 + r * (y1 - y0), (x1 - x0) / seg, (y1 - y0) / seg))
            nxt += spacing
        walked += seg
    return out


def load(out_dir):
    ev = [json.loads(line) for line in open(os.path.join(out_dir, "events.jsonl"), encoding="utf-8") if line.strip()]
    pos, assigned, done, tasks, dog, closed, battery, dog_assigned = {}, {}, {}, {}, {}, {}, {}, {}
    for e in ev:
        k, t = e["kind"], e["t"]
        if k == "battery":
            battery.setdefault(e["uav"], []).append((t, e["pct"]))
        elif k == "dog":   # 2026-10-20 起帶機號（多隻狗）；舊的紀錄沒有，當成 50
            dog.setdefault(e.get("dog", 50), []).append((t, e["x"], e["y"]))
        elif k == "dog_assigned":
            dog_assigned.setdefault(e["dog"], []).append((t, e["msg"].get("task_id", 0)))
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
    return pos, assigned, done, tasks, dog, closed, battery, dog_assigned


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
    ap.add_argument("--speedup", type=float, default=1.0, help="沒有事件時的倍速（預設 1 = 全程同速）")
    ap.add_argument("--slow", type=float, default=1.0, help="事件附近的倍速")
    ap.add_argument("--window", type=float, default=2.0)
    ap.add_argument("--freeze", type=float, default=1.5)
    args = ap.parse_args()

    pos, assigned, done, tasks, dog, closed, battery, dog_assigned = load(args.out_dir)
    # 狗的名字照機號排序叫 dog1、dog2、dog3（2026-10-24 使用者；和 uav1～uav3 一致，原本用機號 dog50～dog52）
    dog_name = {d: f"dog{k + 1}" for k, d in enumerate(sorted(dog))}

    def dname(d):
        return dog_name.get(d, f"dog{d}")
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
        # 任務點就在火點上（tf、fire_mode point）才算火點的任務：0.5 m 內。原本 2.5 m，G1 旁 2 m 的 R4 起火時
        # G1 被叫成「F_R4 fire」，事件列表看不出 G1 完成（2026-10-20）
        return next((f["name"] for f in fires if math.hypot(f["x"] - x, f["y"] - y) <= 0.5), None)

    names = {}
    for i, tid in enumerate(air):
        x, y = tasks[tid]["x"], tasks[tid]["y"]
        p, f = point_of(x, y), fire_of(x, y)
        names[tid] = (f"{p} task (fire)" if p and f else f"{p} task" if p else f"{f} fire" if f else f"T{i + 1}")

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
    for g in ground:
        # 地面任務的位置是偵測到的火點（誤差可到 2 m），用寬一點的範圍找是哪個火
        f = next((ff["name"] for ff in fires if math.hypot(ff["x"] - tasks[g]["x"], ff["y"] - tasks[g]["y"]) <= 2.5),
                 "fire")
        # 發現的那台當時在做什麼（2026-10-20：原本照模式猜，在巡檢點懸停時看到的也寫成 en route）：
        # 手上有空中任務而且在它上空 1 m 內 → 在那個點看到；手上有任務但還沒到 → 路上；手上沒任務 → 待命時
        v, tg = tasks[g]["by"], tasks[g]["t"]
        held = at(assigned.get(v, []), tg)
        here = [p for p in pos.get(v, []) if p[0] <= tg]
        if held in tasks and held in names and here and \
           math.hypot(here[-1][1] - tasks[held]["x"], here[-1][2] - tasks[held]["y"]) <= 1.0:
            how = f"sees {f} at {names[held]} -> ground task"
        elif held in tasks and held in names:
            how = f"sees {f} en route to {names[held]} -> ground task"
        else:
            how = f"sees {f} while idle -> ground task"
        events.append((tg, f"uav{v} " + how, FIRE))
        for d in sorted(dog_assigned):   # 哪隻狗得標（多隻狗時看得出競標結果，2026-10-20）
            first = next((tt for tt, tid in dog_assigned[d] if tid == g), None)
            if first is not None:
                events.append((first, f"{dname(d)} <- ground task ({f})", DOG))
        if g in closed:
            who = next((d for d in sorted(dog_assigned) if any(tid == g for _, tid in dog_assigned[d])), None)
            events.append((closed[g], f"{dname(who)} done -> ground task closed" if who else "dog done -> ground task closed",
                           DOG))
    # 人工指派（insert.py 的 m 指令，2026-10-20）：地面站／平板指定某台去某點、期間不參與 CBBA
    manual = [m for m in (plan.get("manual", []) if plan else []) if m.get("ok")]
    # 2026-10-24 使用者：用 M1、M2… 表示，不直接寫座標（舊的紀錄沒有 label 時照序號補）
    for k, m in enumerate(manual):
        m.setdefault("label", f"M{k + 1}")
        events.append((m["t"], f"{m['target']} manual " + ("clear" if m["clear"] else
                                                          f"<- {m['label']} (leaves CBBA)"), INK))
    events.sort()

    fig = plt.figure(figsize=(12.8, 7.2), facecolor=SURFACE)        # 1280×720（dpi 100）
    # 地圖佔比 2026-10-24 從 1.75 : 1 加大到 2.3 : 1（yard 100 × 80 m，地圖小時三台擠在一起）
    gs = fig.add_gridspec(1, 2, width_ratios=[2.3, 1], wspace=0.05)
    ax = fig.add_subplot(gs[0])
    # 右邊（2026-10-14）：事件時間軸、電量折線圖（有電量資料時）、圖例
    right = gs[1].subgridspec(3, 1, height_ratios=[5.0, 3.0, 1.4] if battery else [8.0, 0.01, 1.4], hspace=0.35)
    axl = fig.add_subplot(right[0])
    axl.axis("off")
    axb = fig.add_subplot(right[1])
    axg = fig.add_subplot(right[2])
    axg.axis("off")

    xs = [tasks[t]["x"] for t in tasks] + [p[1] for v in ids for p in pos[v]] + [p["x"] for p in points.values()] + \
        [p[1] for d in dog.values() for p in d]
    ys = [tasks[t]["y"] for t in tasks] + [p[2] for v in ids for p in pos[v]] + [p["y"] for p in points.values()] + \
        [p[2] for d in dog.values() for p in d]
    pad = 5.0
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect("equal")
    # 圖示大小照地圖比例（2026-10-24 使用者：yard 的 2D 圖無人機圖示太大、三台擠在一起）：原本固定的螢幕大小
    # （無人機約 25 px），yard 每公尺約 7 px，一個圖示佔 3.5 m，起降板上隔 2 m 的三台疊在一起。改成無人機約
    # ICON_M 公尺寬（最小 MIN_ICON_PX），其他圖示一起縮放；demo_site 這種小場景照舊（不放大）
    fig.canvas.draw()
    ppm = ax.get_window_extent().width / (ax.get_xlim()[1] - ax.get_xlim()[0])     # 每公尺幾個 px
    base_px = math.sqrt(320) * 100.0 / 72.0                                       # s = 320 的無人機圖示（px）
    k = min(1.0, max(MIN_ICON_PX, ICON_M * ppm) / base_px)
    ms = k * k                                                                    # scatter 的 s 是面積
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
        ax.scatter([p["x"]], [p["y"]], s=420 * ms, marker="s", color=color, alpha=0.35, linewidths=0, zorder=1)
        ax.annotate(name, (p["x"], p["y"]), xytext=(-10, -17), textcoords="offset points",
                    fontsize=9, color=color, fontweight="bold", zorder=2)
    # 火點：起火的時刻才出現（紅色方塊＋光圈）
    fire_art = {}
    for f in fires:
        block = ax.scatter([f["x"]], [f["y"]], s=300 * ms, marker="s", color=FIRE, alpha=0.8, linewidths=0, zorder=3,
                           visible=False)
        halo = ax.scatter([f["x"]], [f["y"]], s=1100, facecolors=[(0.78, 0.09, 0.11, 0.15)], edgecolors=FIRE,
                          linewidths=2, zorder=3, visible=False)
        lbl = ax.annotate("FIRE (tag 101)", (f["x"], f["y"]), xytext=(12, -22), textcoords="offset points",
                          fontsize=9, color=FIRE, fontweight="bold", zorder=6, visible=False)
        fire_art[f["name"]] = (f["spawned"], halo, lbl, block)
    manual_art = []
    for m in manual:
        if m["clear"]:
            continue
        mk = ax.scatter([m["x"]], [m["y"]], s=260 * ms, marker="X", color=INK, linewidths=0, zorder=5, visible=False)
        ml = ax.annotate(m["label"], (m["x"], m["y"]), xytext=(8, 6), textcoords="offset points",
                         fontsize=9, color=INK, fontweight="bold", zorder=6, visible=False)
        manual_art.append((m["t"], mk, ml))

    task_art, task_lbl = {}, {}
    for tid in air:
        x, y = tasks[tid]["x"], tasks[tid]["y"]
        # 旗子（旗桿底端在任務點）：等待中白底、有人執行時填那台的顏色、完成變灰
        task_art[tid] = ax.scatter([x], [y], s=700 * ms, marker=FLAG, facecolors=SURFACE, edgecolors=MUTED,
                                   linewidths=1.2, zorder=4, visible=False)
        task_lbl[tid] = ax.annotate("", (x, y), xytext=(16, 14), textcoords="offset points",
                                    fontsize=9, color=INK, zorder=5, visible=False)
    ground_art = {}
    for g in ground:
        x, y = tasks[g]["x"], tasks[g]["y"]
        sq = ax.scatter([x], [y], s=170 * ms, marker="s", color=FIRE, edgecolors=SURFACE, linewidths=1.5,
                        zorder=6, visible=False)
        lbl = ax.annotate("ground task -> dog", (x, y), xytext=(12, -36), textcoords="offset points",
                          fontsize=9, color=FIRE, fontweight="bold", zorder=6, visible=False)
        ground_art[g] = (sq, lbl)

    trails, drones, links = {}, {}, {}
    for v in ids:
        trails[v], = ax.plot([], [], color=colors[v], lw=1.8, alpha=0.85, zorder=2)
        links[v], = ax.plot([], [], color=colors[v], lw=1.2, ls="--", zorder=2)
        # 圖示不上色（2026-10-20 使用者），各台靠航跡和編號的顏色區分
        drones[v] = ax.scatter([], [], s=320 * ms, marker=QUAD, color=INK_2, edgecolors=INK, linewidths=0.6, zorder=7)
    tags = {v: ax.annotate(f"uav{v}", (0, 0), xytext=(0, 10), textcoords="offset points", ha="center", fontsize=9,
                           fontweight="bold", color=colors[v], zorder=8, visible=False) for v in ids}
    dog_color = {d: DOG_SHADES[i % len(DOG_SHADES)] for i, d in enumerate(sorted(dog))}
    dog_trail, dog_art, dog_tag = {}, {}, {}
    for d in sorted(dog):
        dog_trail[d], = ax.plot([], [], color=dog_color[d], lw=1.8, alpha=0.85, zorder=2)
        dog_art[d] = ax.scatter([], [], s=460 * ms, marker=DOG_ICON, color=dog_color[d], edgecolors=INK, linewidths=0.6,
                                zorder=7)
        dog_tag[d] = ax.annotate(dname(d), (0, 0), xytext=(0, -16), textcoords="offset points", ha="center",
                                 fontsize=9, fontweight="bold", color=dog_color[d], zorder=8, visible=False)
    clock = ax.text(0.99, 0.02, "", transform=ax.transAxes, ha="right", va="bottom", fontsize=11,
                    color=INK, bbox=dict(facecolor=SURFACE, edgecolor=GRID, pad=3))
    # 電量（PX4 battery_status，2026-10-14）：右邊的橫向長條圖，每台一條、長度 = 目前電量，隨時間縮短。
    # 一般的 0～100 % 刻度（2026-10-19 使用者：原本從最低電量附近開始畫，「axis from 86」看不懂）；實際值看長條旁的數字
    bat_bar, bat_val = {}, {}
    if battery:
        lo = 0.0
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
        axb.set_title("Battery (%)", loc="left", fontsize=11, fontweight="bold", color=INK)
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
    handles = [Line2D([], [], marker=QUAD, ls="-", lw=2, markersize=14, color=colors[v], markerfacecolor=INK_2,
                      markeredgecolor=INK, markeredgewidth=0.6, label=f"uav{v}") for v in ids]
    if points:
        handles += [Line2D([], [], marker="s", ls="", markersize=11, color=PATROL, alpha=0.5, label="fixed task (G)")]
    handles += [Line2D([], [], marker=FLAG, ls="", markersize=16, markerfacecolor=SURFACE, markeredgecolor=INK,
                       markeredgewidth=1.0, label="UAV task (flag)")]
    handles += [Line2D([], [], marker=DOG_ICON, ls="", markersize=14, color=dog_color[d], markeredgecolor=INK,
                       markeredgewidth=0.6, label=f"{dname(d)} (fake V60)") for d in sorted(dog)]
    handles += [Line2D([], [], marker="o", ls="", markersize=13, markerfacecolor=(0.78, 0.09, 0.11, 0.15),
                       markeredgecolor=FIRE, label="fire (tag 101)"),
                Line2D([], [], marker="s", ls="", markersize=10, color=FIRE, label="ground task (dog)")]
    if manual_art:
        handles += [Line2D([], [], marker="X", ls="", markersize=10, color=INK, label="manual assignment (Mx)")]
    axg.legend(handles=handles, loc="center left", fontsize=9, frameon=False, ncol=2, bbox_to_anchor=(0.0, 0.5))

    keys = [e[0] for e in events]
    freezes = [f["spawned"] for f in fires] + [tasks[g]["t"] for g in ground] + list(closed.values())
    times = frame_times(t_start, t_end, keys, freezes, args.fps, args.speedup, args.slow, args.window, args.freeze)

    # 方向箭頭：每個載具一串 annotate，路程變長才新增（時間只往前，不用每一格重建）
    arrow_art = {}

    def draw_arrows(key, xs, ys, color):
        spots = arrow_spots(xs, ys, spacing=ARROW_SPACING / k)     # 地圖縮小時箭頭也拉開（yard 約 9 m 一個）
        arts = arrow_art.setdefault(key, [])
        while len(arts) < len(spots):
            arts.append(ax.annotate("", (0, 0), xytext=(0, 0), zorder=3, arrowprops=dict(
                arrowstyle="-|>,head_length=0.7,head_width=0.35", color=color, lw=1.2, shrinkA=0, shrinkB=0)))
        for i, a in enumerate(arts):
            if i < len(spots):
                x, y, ux, uy = spots[i]
                a.xy = (x + 0.35 * ux, y + 0.35 * uy)
                a.set_position((x - 0.35 * ux, y - 0.35 * uy))
                a.set_visible(True)
            else:
                a.set_visible(False)

    def draw(t, rate):
        for v in ids:
            pts = [p for p in pos[v] if t_start - 1 <= p[0] <= t]
            if not pts:
                continue
            trails[v].set_data([p[1] for p in pts], [p[2] for p in pts])
            draw_arrows(("uav", v), [p[1] for p in pts], [p[2] for p in pts], colors[v])
            drones[v].set_offsets([[pts[-1][1], pts[-1][2]]])
            tags[v].xy = (pts[-1][1], pts[-1][2])
            tags[v].set_visible(True)
            cur = at(assigned.get(v, []), t)
            if cur in tasks and not (cur in done and done[cur][0] <= t):
                links[v].set_data([pts[-1][1], tasks[cur]["x"]], [pts[-1][2], tasks[cur]["y"]])
            else:
                links[v].set_data([], [])
        for d, track in dog.items():
            dp = [p for p in track if p[0] <= t]
            if dp:
                moved = [p for p in dp if p[0] >= t_start - 1]
                dog_trail[d].set_data([p[1] for p in moved], [p[2] for p in moved])
                draw_arrows(("dog", d), [p[1] for p in moved], [p[2] for p in moved], dog_color[d])
                dog_art[d].set_offsets([[dp[-1][1], dp[-1][2]]])
                dog_tag[d].xy = (dp[-1][1], dp[-1][2])
                dog_tag[d].set_visible(True)
        for ts, mk, ml in manual_art:
            mk.set_visible(ts <= t)
            ml.set_visible(ts <= t)
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
                who = next((d for d in sorted(dog_assigned) if at(dog_assigned[d], t) == g), None)
                if g in closed and closed[g] <= t:
                    lbl.set_text("ground task done")
                elif who is not None:
                    lbl.set_text(f"ground task -> {dname(who)}")
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
