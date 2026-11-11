#!/usr/bin/env python3
"""把 itest demo（或 uav）記下的事件畫成俯視動畫：無人機飛去自己拿到的任務、隨機火警出現、丟出地面任務。

  python3 docker/itest/animate_demo.py <itest 輸出目錄> --frames <目錄>   每一格存成 PNG（主機用 ffmpeg 合成 MP4；
                                                                         itest.py demo 會自動做）
  python3 docker/itest/animate_demo.py <itest 輸出目錄>                    直接存成 demo.gif（沒有 ffmpeg 時）

要 matplotlib（主機沒有時在 sim 容器跑）。只用 ros_bridge 記下的話題（PX4 位置、current_task、task_result、new_task），
不看節點的 log；有 demo_plan.json（itest demo 寫的）時另外畫出全部任務點、火警起火的時刻。

播放速度：預設全程 1 倍速（2026-10-19 使用者：忽快忽慢沒有意義，原本沒有事件時 3 倍速快轉）；
中途不停頓（2026-11-07 使用者：起火、丟出地面任務、地面任務結束時原本停 1.5 秒，影響觀看），只在結尾停 3 秒。
要在事件時停住再給 --freeze 秒；要快轉時給 --speedup（沒有事件時的倍速），事件前後 --window 秒維持 --slow 倍速。
畫面（2026-11-06 使用者的「CBBA 動畫顯示需求」）：
  左：載具用相同圖示＋名字（UAV1～3 有機頭、朝移動方向；dog1～3），播放中不畫歷史路徑、不畫起飛區、不寫「任務 → 誰」；
      任務全部用旗子，顏色同 Gazebo 的地面標記（黃 T＝UAV 一般任務、綠 G＝固定任務、紅 F＝火災／地面處置、藍 M＝人工指派），
      旁邊只寫編號；狗的日常巡檢點是紫色小旗子（A1～、B1～、C1～；做完變灰，下一圈再出發去它時變回紫色）；做完的旗子變淺灰（2026-11-07 使用者：讓觀看者知道做完了；Gazebo 那邊由 itest 刪掉標記）
  右：時間、Live Agent Status（Agent／Task／Status：Idle、En Route、Executing、Completed、Returning、
      Unavailable，每一格是當下的狀態）、Battery (%)（同一個顏色）、旗子的圖例
  最後一格（demo_final.png）：加上 UAV 的完整航跡（UAV1 實線、UAV2 虛線、UAV3 點線，航跡上標名字）、Makespan、Final Battery (%)
  不顯示任務數量（使用者 2026-11-06）；狗巡邏時顯示巡檢點（dog1 A1～、dog2 B1～、dog3 C1～，2026-11-07，demo_plan.json 的 dog[].patrol；
  到點 ptz_inspect 時 Executing），地面任務 F、人工指派 M 優先
  --at 秒：只畫那一格（demo_at.png，看版面用）；--final-only：只重畫最後一格
"""
import argparse
import bisect
import json
import math
import os
import re
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
sys.path.insert(0, HERE)
import dog_patrol  # noqa: E402

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


def quad_nose_marker():
    """有機頭的四旋翼（2026-11-06：朝目前移動方向轉）：四旋翼＋前方（+y）一個小三角形"""
    nose = Path([(-0.22, 0.82), (0.22, 0.82), (0.0, 1.15), (-0.22, 0.82)],
                [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY])
    return Path.make_compound_path(quad_marker(), nose)


QUAD_NOSE = quad_nose_marker()
DOG_ICON = dog_marker()
FLAG = flag_marker()


FINAL_ARROW_M = 22.0  # 最後一格 UAV 航跡上的方向箭頭間隔（公尺）
ARROW_SPACING = 10.0  # 航跡上每隔幾公尺畫一個方向箭頭（2026-10-20，使用者：路徑加箭頭；2026-11-06 4 → 10 m：太密）
DOG_TRAIL_SEC = 30.0  # 狗的軌跡只畫最近幾秒（2026-11-06：巡邏來回好幾趟，全畫會疊成粗線）
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
            pos.setdefault(e["uav"], []).append((t, e["x"], e["y"], e.get("z", 0.0)))
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
    ap.add_argument("--freeze", type=float, default=0.0, help="起火、地面任務出現與結束時停住幾秒（預設 0 = 不停）")
    ap.add_argument("--final-only", action="store_true", help="只畫最後一格 demo_final.png（不做動畫）")
    ap.add_argument("--at", type=float, default=None, help="只畫開始後這一秒的一格（demo_at.png，看版面用）")
    ap.add_argument("--status", action="store_true", help="只印 Live Agent Status 的變化（文字，檢查時段用，不畫圖）")
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
    ev_path = os.path.join(args.out_dir, "events.jsonl")
    events_raw = [json.loads(l) for l in open(ev_path, encoding="utf-8") if l.strip()] if os.path.exists(ev_path) else []
    points = plan["points"] if plan else {}
    fires = [f for f in plan["fires"] if "spawned" in f] if plan else []
    # 火的名字照起火順序 F1、F2…（和 gazebo_labeled.mp4 一樣；事件列表、地圖都用這個，2026-11-06）
    fire_label = {f["name"]: f"F{i + 1}" for i, f in enumerate(sorted(fires, key=lambda f: f["spawned"]))}

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

    plan_name = {p["task_id"]: p["point"] for p in (plan or {}).get("patrol", []) if "task_id" in p}
    names = {}
    for i, tid in enumerate(air):
        if tid in plan_name:            # 情境檔的名字（G1、T3…，2026-11-06：地圖上看得出是哪個任務）
            names[tid] = plan_name[tid]
            continue
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
        f = next((fire_label[ff["name"]] for ff in fires
                  if math.hypot(ff["x"] - tasks[g]["x"], ff["y"] - tasks[g]["y"]) <= 2.5), "fire")
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
        if isinstance(v, str):     # 狗的模擬偵測（itest dog_watch，2026-11-05）：new_task 的 uav 是 "dogN"
            how = f"sees {f} -> ground task"
        events.append((tg, (v if isinstance(v, str) else f"uav{v}") + " " + how, FIRE))
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

    # =========================================================================================================
    # 畫面（2026-11-06 使用者的「CBBA 動畫顯示需求」）：
    #   左：載具（相同圖示＋名字，不用顏色分）、任務（全部用旗子，顏色分種類：灰 T、綠 G、紅 F、黑 M，旁邊只寫編號）
    #       播放中不畫歷史路徑、不畫起飛區、不寫「任務 → 誰」；最後一格才畫 UAV 的完整航跡（實線／虛線／點線）
    #   右：Live Agent Status（Agent／Task／Status，每一格是當下的狀態）、Battery (%)（同一個顏色）、時間與完成數；
    #       最後一格：Tasks Completed、Makespan、Final Battery (%)
    # =========================================================================================================
    # 旗子顏色和 Gazebo 的地面標記一樣（models/task_marker_g／m／t／f、label_gazebo.py，2026-11-07 使用者）
    FLAG_COLOR = {"T": "#f2bf0d", "G": "#1ab233", "F": "#d91a14", "M": "#2666e6", "P": "#8e44ad"}  # P：狗的巡檢點
    FLAG_DONE, FLAG_DONE_EDGE, FLAG_DONE_TEXT = "#d4d4d4", "#a0a0a0", "#9a9a9a"     # 做完的旗子（淺灰，2026-11-07）
    VEHICLE = "#3a3a3a"
    BAR = "#5b6f86"
    LINESTYLE = ["-", "--", ":"]
    home = {v: (pos[v][0][1], pos[v][0][2]) for v in ids}          # 起飛點＝第一筆位置（只用來判斷返航，不畫）

    # ---- 任務（旗子）：種類、編號、位置、出現與完成的時刻 ----
    flags = []
    for tid in air:
        nm = names[tid]
        kind = "G" if nm.startswith("G") else "T"
        flags.append(dict(kind=kind, name=nm, x=tasks[tid]["x"], y=tasks[tid]["y"], t=tasks[tid]["t"],
                          done=done[tid][0] if tid in done else None))
    ground_name = {}
    for g in ground:
        f = next((fire_label[ff["name"]] for ff in fires
                  if math.hypot(ff["x"] - tasks[g]["x"], ff["y"] - tasks[g]["y"]) <= 2.5), f"F{len(ground_name) + 1}")
        ground_name[g] = f
        flags.append(dict(kind="F", name=f, x=tasks[g]["x"], y=tasks[g]["y"], t=tasks[g]["t"], done=closed.get(g)))

    def pos_at(series, t):
        """series 的 (t, x, y, …) 在 t 時的位置（線性內插）；還沒有資料回傳 None"""
        ts = [q[0] for q in series]
        i = bisect.bisect_right(ts, t)
        if i == 0:
            return None
        if i >= len(series):
            return series[-1][1:3]
        (t0, x0, y0), (t1, x1, y1) = series[i - 1][:3], series[i][:3]
        a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return x0 + (x1 - x0) * a, y0 + (y1 - y0) * a

    # 人工指派 M：從指派到「到點、停滿 duration 秒後離開 1.5 m」為止
    manual_jobs = []
    for m in manual:
        if m.get("clear"):
            continue
        v = int(str(m["target"]).replace("uav", "")) if str(m["target"]).startswith("uav") else None
        # 完成時刻用 BT 的實際紀錄（itest 記的 finished_t，「平板任務 N 完成」）；舊的輸出沒有才用距離猜
        # （到點、停滿 duration 後離開 1.5 m——火在 M 旁時移到火點上空會被誤判成做完，2026-11-07）
        end = m.get("finished_t")
        label = str(m.get("label", ""))
        log = os.path.join(args.out_dir, f"stack_uav{v}.log")
        if end is None and v is not None and label[1:].isdigit() and os.path.exists(log):
            # 舊的輸出沒有 finished_t：直接讀 BT 的 log（「[牆上時間] [節點]: 平板任務 N 完成」）
            for line in open(log, encoding="utf-8", errors="replace"):
                mm = re.search(r"\[(\d+\.\d+)\] \[[^\]]+\]: .*平板任務 " + label[1:] + " 完成", line)
                if mm and float(mm.group(1)) >= m["t"]:
                    end = float(mm.group(1))
                    break
        if end is None and v is not None:
            arrive = next((q[0] for q in pos.get(v, []) if q[0] >= m["t"] and
                           math.hypot(q[1] - m["x"], q[2] - m["y"]) < 1.0), None)
            if arrive is not None:
                end = next((q[0] for q in pos[v] if q[0] >= arrive + m.get("duration", 0.0) and
                            math.hypot(q[1] - m["x"], q[2] - m["y"]) > 1.5), None)
        tg = str(m["target"])
        dog_id = int(tg[3:]) if tg.startswith("dog") and tg[3:].isdigit() else None    # 指派給狗的 M（2026-11-07）
        manual_jobs.append(dict(uav=v, dog=dog_id, name=m["label"], x=m["x"], y=m["y"], t=m["t"], end=end))
        flags.append(dict(kind="M", name=m["label"], x=m["x"], y=m["y"], t=m["t"], done=end))
    # 狗的日常巡檢點（2026-11-07 使用者：圖上也標狗的任務）：紫色小旗子、一開始就在
    # 做完（雲台檢查完、往下一點走）變灰，下一圈再出發去它時變回紫色（使用者 2026-11-07；dog_patrol.py 讀狗 BT 的 log）
    for u in (plan.get("dog") or []) if plan else []:
        pts = u.get("patrol") or []
        log = os.path.join(args.out_dir, f"dog_bt_{u['ns']}.log")
        ivs = dog_patrol.done_intervals(dog_patrol.parse(open(log, encoding="utf-8", errors="replace")),
                                        [q["name"] for q in pts]) if os.path.exists(log) else {}
        for q in pts:
            flags.append(dict(kind="P", name=q["name"], x=q["x"], y=q["y"], t=float("-inf"), done=None,
                              done_ivs=ivs.get(q["name"], [])))

    task_xy = {tid: (tasks[tid]["x"], tasks[tid]["y"]) for tid in tasks}

    # 火警監控（BT 自己的流程，不是分派的任務；2026-11-07 使用者：要切換，才知道它去做什麼）：
    # 發現的那台從回報到地面任務結束（最多 monitor_timeout 120 s），先 En Route 到火點、到了上空（2 m 內）Executing；
    # 別台在這段時間停在火點上空（2 m 內、速度 < 0.5 m/s：AirRecon 看到同一個火改成監控）也算監控
    MONITOR_R, MONITOR_MAX = 2.0, 120.0
    watches = []
    for f in fires:
        g = next((gg for gg, nm in ground_name.items() if nm == fire_label.get(f["name"])), None)
        t0_f = f.get("reported_t") or (tasks[g]["t"] if g else None)
        if t0_f is None:
            continue
        end_f = closed.get(g) if g in closed else t0_f + MONITOR_MAX
        by = f.get("reported_by")
        arrive = next((q[0] for q in pos.get(by, []) if q[0] >= t0_f and
                       math.hypot(q[1] - f["x"], q[2] - f["y"]) < MONITOR_R), None) if by in pos else None
        watches.append(dict(name=fire_label[f["name"]], x=f["x"], y=f["y"], t0=t0_f, t1=min(end_f, t0_f + MONITOR_MAX),
                            by=by, arrive=arrive))

    def speed_at(series, t):
        p = [q for q in series if t - 1.0 <= q[0] <= t]
        if len(p) < 2 or p[-1][0] - p[0][0] < 0.3:
            return 0.0
        return math.hypot(p[-1][1] - p[0][1], p[-1][2] - p[0][2]) / (p[-1][0] - p[0][0])

    def watching(v, t, x, y):
        """這台現在在監控哪個火：(火的編號, Status) 或 None"""
        for w in watches:
            if not w["t0"] <= t <= w["t1"]:
                continue
            near = math.hypot(x - w["x"], y - w["y"]) < MONITOR_R
            if near and (w["by"] == v or speed_at(pos[v], t) < 0.5):
                return w["name"], "Executing"
            if w["by"] == v and not near and (w["arrive"] is None or t < w["arrive"]):
                return w["name"], "En Route"
        return None

    # 狀態只有三種（2026-11-07 使用者：讓觀眾知道每台現在的任務做完了沒）：En Route（前往）、Executing（執行中）、
    # Done（最後一個任務做完，直到接到下一個）；還沒有任務時 Task、Status 都是 —
    def uav_status(v, t):
        """(Task, Status)：每一格只看當下"""
        p = [q for q in pos[v] if q[0] <= t]
        if not p or p[-1][3] < 1.0:
            return "—", "—"
        x, y = p[-1][1], p[-1][2]
        w = watching(v, t, x, y)
        if w:
            return w
        for job in manual_jobs:
            if job["uav"] == v and job["t"] <= t and (job["end"] is None or t < job["end"]):
                near = math.hypot(x - job["x"], y - job["y"]) < 1.0
                return job["name"], "Executing" if near else "En Route"
        cur = at(assigned.get(v, []), t)
        if cur in tasks and not (cur in done and done[cur][0] <= t):
            tx, ty = task_xy[cur]
            return names.get(cur, "?"), "Executing" if math.hypot(x - tx, y - ty) < 1.0 else "En Route"
        finished = [(tt, names.get(tid, "?")) for tid, (tt, vv) in done.items() if vv == v and tt <= t] + \
            [(job["end"], job["name"]) for job in manual_jobs if job["uav"] == v and job["end"] and job["end"] <= t]
        if finished:
            return max(finished)[1], "Done"
        return "—", "—"

    # 狗的日常巡檢（2026-11-07 使用者：狗也要有任務變化）：itest 的 demo_plan.json 的 dog[].patrol（dog1 A1～、dog2 B1～、
    # dog3 C1～，到點後 ptz_inspect 看最近的設備；每隻狗各自一條巡檢線）。現在在哪個巡檢點照伯宇 BT 的 log
    # （dog_bt_<ns>.log 的「去 B2」→ En Route、「✓ 抵達 B2」→ Executing）：被 CBBA 任務打斷後他會接回被打斷的那個點，
    # 原本用位置推算（到點再離開就換下一個）會跳到下一個點，和實際不同步（2026-11-07）
    patrol_of = {u["id"]: u.get("patrol") or [] for u in (plan.get("dog") or [])} if plan else {}

    def dog_busy(d, t):
        """(Task, 目標座標, 半徑) 或 None：CBBA 地面任務 ＞ 人工指派 M"""
        cur = at(dog_assigned.get(d, []), t)
        if cur in ground_name and not (cur in closed and closed[cur] <= t):
            return ground_name[cur], task_xy[cur], 2.5
        for job in manual_jobs:
            if job["dog"] == d and job["t"] <= t and (job["end"] is None or t < job["end"]):
                return job["name"], (job["x"], job["y"]), 2.5
        return None

    patrol_line = {}       # 狗 → [(t, 巡檢點名稱或 None（在做別的）, Status)]
    for u in (plan.get("dog") or []) if plan else []:
        names_p = {q["name"] for q in patrol_of.get(u["id"], [])}
        log = os.path.join(args.out_dir, f"dog_bt_{u['ns']}.log")
        if not names_p or not os.path.exists(log):
            continue
        line = []
        for row in open(log, encoding="utf-8", errors="replace"):
            mm = re.search(r"\[(\d+\.\d+)\] .*(去|✓ 抵達) (\S+)", row)
            if not mm:
                continue
            wp = mm.group(3) if mm.group(3) in names_p else None
            line.append((float(mm.group(1)), wp, "En Route" if mm.group(2) == "去" else "Executing"))
        patrol_line[u["id"]] = line

    def dog_status(d, t):
        p = [q for q in dog[d] if q[0] <= t]
        if not p:
            return "—", "—"
        x, y = p[-1][1], p[-1][2]
        busy = dog_busy(d, t)
        if busy:
            name, (tx, ty), r = busy
            return name, "Executing" if math.hypot(x - tx, y - ty) < r else "En Route"
        finished = [(closed[g], ground_name[g], task_xy[g]) for g in ground_name if g in closed and closed[g] <= t and
                    any(tid == g for _, tid in dog_assigned.get(d, []))] + \
            [(job["end"], job["name"], (job["x"], job["y"])) for job in manual_jobs
             if job["dog"] == d and job["end"] and job["end"] <= t]
        last = max(finished) if finished else None
        line = patrol_line.get(d)
        i = bisect.bisect_right([q[0] for q in line], t) - 1 if line else -1
        if i >= 0 and line[i][1] and not (last and last[0] > line[i][0]):
            # 巡邏中（BT 最後一筆是巡檢點）；剛做完的任務比這筆新時先顯示 Done，等 BT 出發去下一個巡檢點
            return line[i][1], line[i][2]
        if last:
            return last[1], "Done"
        return "—", "—"

    def heading(series, t):
        """目前移動方向（rad）：最近 1 s 內位移 > 0.2 m 時；否則沿用上一次（懸停時不亂轉）"""
        p = [q for q in series if t - 1.0 <= q[0] <= t]
        if len(p) >= 2 and math.hypot(p[-1][1] - p[0][1], p[-1][2] - p[0][2]) > 0.2:
            return math.atan2(p[-1][2] - p[0][2], p[-1][1] - p[0][1])
        return None

    finish_times = [f["done"] for f in flags if f["done"]]
    makespan = (max(finish_times) - t_start) if finish_times else None

    # ---- 版面（2026-11-06 使用者給的樣式圖：深藍標題列、右邊三張圓角卡片；Tasks、Makespan 不放）----
    from matplotlib.patches import FancyBboxPatch, Rectangle
    NAVY, CARD_EDGE = "#16386b", "#9fb2cc"
    UAV_COLOR = ["#1f6fe5", "#f07a1c", "#22b04b", "#7a4fc9"]
    ucolor = {v: UAV_COLOR[i % len(UAV_COLOR)] for i, v in enumerate(ids)}
    W_IN, H_IN = 14.4, 7.2                                           # 1440×720（dpi 100）
    fig = plt.figure(figsize=(W_IN, H_IN), facecolor="white")

    def card(x0, y0, x1, y1, fc="white", ec=CARD_EDGE, lw=1.2, r=0.012):
        fig.patches.append(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle=f"round,pad=0,rounding_size={r}",
                                          transform=fig.transFigure, facecolor=fc, edgecolor=ec, lw=lw, zorder=0))

    # 標題列
    # 標題列只在最後一格（2026-11-07 使用者：demo 影片不要標題，地圖用整個高度）
    n_before = len(fig.patches)
    card(0.015, 0.875, 0.58, 0.985, fc=NAVY, ec=NAVY)
    world = plan["world"] if plan else "Gazebo"
    sub = f" | Seed {plan['seed']}" if plan else ""
    banner = fig.patches[n_before:] + [
        fig.text(0.2975, 0.948, "CBBA Multi-UAV Task Allocation Result", ha="center", va="center", color="white",
                 fontsize=19, fontweight="bold"),
        fig.text(0.2975, 0.898, f"PX4 SITL ({world}) | {len(ids)} UAVs{sub}", ha="center", va="center",
                 color="#dfe8f5", fontsize=12)]
    MAP_FINAL, MAP_VIDEO = [0.055, 0.075, 0.525, 0.78], [0.055, 0.075, 0.525, 0.9]
    ax = fig.add_axes(MAP_VIDEO)

    xs = [f["x"] for f in flags] + [q[1] for v in ids for q in pos[v]] + [q[1] for d in dog.values() for q in d]
    ys = [f["y"] for f in flags] + [q[2] for v in ids for q in pos[v]] + [q[2] for d in dog.values() for q in d]
    pad = 5.0
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_aspect("equal")
    fig.canvas.draw()
    ppm = ax.get_window_extent().width / (ax.get_xlim()[1] - ax.get_xlim()[0])     # 每公尺幾個 px
    base_px = math.sqrt(320) * 100.0 / 72.0
    k = min(1.0, max(MIN_ICON_PX, ICON_M * ppm) / base_px)
    ms = k * k
    ax.set_facecolor("white")
    ax.grid(color="#e6e9ee", linewidth=0.8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.tick_params(labelsize=11)
    ax.set_xlabel("x east (m)", fontsize=12, color=INK)
    ax.set_ylabel("y north (m)", fontsize=12, color=INK)
    clock = ax.text(0.97, 0.04, "", transform=ax.transAxes, ha="right", va="bottom", fontsize=12, color=INK,
                    bbox=dict(boxstyle="round,pad=0.4", facecolor="#f4f5f7", edgecolor="#c7ccd4"), zorder=9)

    # 旗子：出現的時刻才顯示；旁邊只寫編號
    flag_art = []
    for f in flags:
        mk = ax.scatter([f["x"]], [f["y"]], s=600 if f["kind"] == "P" else 1000, marker=FLAG, color=FLAG_COLOR[f["kind"]], edgecolors="#222222",
                        linewidths=0.6, zorder=4, visible=False)
        lb = ax.annotate(f["name"], (f["x"], f["y"]), xytext=(12, -12), textcoords="offset points", fontsize=11,
                         fontweight="bold", color=INK, zorder=5, visible=False)
        flag_art.append((f, mk, lb))

    # 載具：播放中同一個圖示（無人機有機頭，朝移動方向轉）＋名字；最後一格只畫航跡
    uav_art, uav_tag, uav_head = {}, {}, {}
    for v in ids:
        uav_art[v] = ax.scatter([], [], s=130, marker=QUAD, color=VEHICLE, edgecolors=INK,
                                linewidths=0.4, zorder=7)
        uav_tag[v] = ax.annotate(f"UAV{v}", (0, 0), xytext=(0, 12), textcoords="offset points", ha="center",
                                 fontsize=10, fontweight="bold", color=INK, zorder=8, visible=False)
        uav_head[v] = math.pi / 2
    # 狗的偵測範圍（2026-11-09 使用者：影片看不出狗「看得到」火）：demo.dog_detect.radius（10 m）的圓，跟著狗移動；
    # 平常淡色虛線，發現火的前後（持續 dwell 秒開始、回報後 DETECT_HOLD 秒）那隻狗的圓變紅、加粗，標「dogN sees Fk」
    det = (plan or {}).get("dog_detect") or {}
    if not det:
        try:
            det = json.load(open(os.path.join(HERE, "config.json"), encoding="utf-8"))["demo"].get("dog_detect") or {}
        except (OSError, KeyError, ValueError):
            det = {}
    det_r, det_dwell, DETECT_HOLD = float(det.get("radius", 0.0)), float(det.get("dwell", 1.0)), 4.0
    dog_reports = {}                 # 狗機號 → [(回報時刻, 火的編號)]
    for e in events_raw:
        if e.get("kind") == "new_task" and e.get("source") == "dog_detect" and e.get("dog") is not None:
            pxy = e["msg"]["position"]
            # 狗回報的位置就是火的位置（dog_watch 用火的座標發布），1 m 內認是哪個火
            f = next((ff["name"] for ff in fires if math.hypot(ff["x"] - pxy["x"], ff["y"] - pxy["y"]) <= 1.0), None)
            dog_reports.setdefault(e["dog"], []).append((e["t"], fire_label.get(f, "fire") if f else "fire"))
    det_art, det_txt = {}, {}
    from matplotlib.patches import Circle
    for d in sorted(dog):
        if det_r > 0:
            det_art[d] = Circle((0, 0), det_r, fill=False, edgecolor="#7a3e12", linewidth=1.0, linestyle="--",
                                alpha=0.45, zorder=3, visible=False)
            ax.add_patch(det_art[d])
            det_txt[d] = ax.annotate("", (0, 0), xytext=(0, 0), textcoords="offset points", ha="center",
                                     fontsize=10, fontweight="bold", color="#d0021b", zorder=9, visible=False)
    dog_art, dog_tag = {}, {}
    for d in sorted(dog):
        dog_art[d] = ax.scatter([], [], s=max(460 * ms, 300), marker=DOG_ICON, color="#7a3e12", linewidths=0, zorder=7)
        dog_tag[d] = ax.annotate(dname(d), (0, 0), xytext=(0, -15), textcoords="offset points", ha="center",
                                 fontsize=10, fontweight="bold", color="#7a3e12", zorder=8, visible=False)
    trail_art = {}
    for v in ids:
        trail_art[v], = ax.plot([], [], color=ucolor[v], lw=2.4, zorder=2, visible=False, solid_capstyle="round")
    arrow_art = {}

    def draw_arrows(v, xs_, ys_):
        spots = arrow_spots(xs_, ys_, spacing=FINAL_ARROW_M)
        arts = arrow_art.setdefault(v, [])
        while len(arts) < len(spots):
            arts.append(ax.annotate("", (0, 0), xytext=(0, 0), zorder=3, arrowprops=dict(
                arrowstyle="-|>,head_length=0.9,head_width=0.45", color=ucolor[v], lw=0, shrinkA=0, shrinkB=0)))
        for i_, a_ in enumerate(arts):
            if i_ < len(spots):
                x_, y_, ux_, uy_ = spots[i_]
                a_.xy = (x_ + 0.6 * ux_, y_ + 0.6 * uy_)
                a_.set_position((x_ - 0.6 * ux_, y_ - 0.6 * uy_))
                a_.set_visible(True)
            else:
                a_.set_visible(False)

    # ---- 右邊：狀態表卡片 ----
    RX0, RX1 = 0.595, 0.985
    card(RX0, 0.47, RX1, 0.985)
    card(RX0, 0.905, RX1, 0.985, fc=NAVY, ec=NAVY)
    fig.patches.append(Rectangle((RX0, 0.905), RX1 - RX0, 0.02, transform=fig.transFigure, facecolor=NAVY,
                                 edgecolor=NAVY, zorder=0))                       # 標題列下緣改成直角
    st_title = fig.text(RX0 + 0.012, 0.955, "Live Agent Status", color="white", fontsize=15, fontweight="bold",
                        va="center")
    fig.text(RX0 + 0.012, 0.922, "Task and status update during execution", color="#dfe8f5", fontsize=10.5,
             va="center")
    TX = (RX0 + 0.012, RX0 + 0.135, RX0 + 0.255)        # 三欄的左緣
    TW = (0.118, 0.115, 0.12)
    TOP, ROW = 0.885, 0.052
    # 卡片裡直接放三欄文字（2026-11-07 使用者：欄位裡不要再畫表格，浪費空間）
    for c_, w_, head in zip(TX, TW, ("Agent", "Task", "Status")):
        fig.text(c_ + w_ / 2, TOP - ROW / 2, head, ha="center", va="center", fontsize=12, fontweight="bold", color=INK)
    MAX_ROWS = len(ids) + len(dog)
    row_art = []
    for r in range(MAX_ROWS):
        y = TOP - ROW * (r + 2)
        name = fig.text(TX[0] + TW[0] / 2, y + ROW / 2, "", ha="center", va="center", fontsize=12, fontweight="bold")
        task_t = fig.text(TX[1] + TW[1] / 2, y + ROW / 2, "", ha="center", va="center", fontsize=12, color=INK)
        stat_t = fig.text(TX[2] + TW[2] / 2, y + ROW / 2, "", ha="center", va="center", fontsize=12,
                          fontweight="bold", color=INK)
        row_art.append((name, task_t, stat_t))

    def agent_color(label):
        if label.startswith("UAV"):
            return ucolor[int(label[3:])]
        return "#7a3e12"

    # ---- 右邊：電量卡片 ----
    card(RX0, 0.27, RX1, 0.45)
    bat_title = fig.text((RX0 + RX1) / 2, 0.425, "Battery (%)", ha="center", va="center", fontsize=13,
                         fontweight="bold", color=INK)
    bat_bar, bat_val = {}, {}
    BX0, BX1 = RX0 + 0.07, RX1 - 0.075
    for i, v in enumerate(ids):
        y = 0.385 - i * 0.04
        fig.text(RX0 + 0.015, y, f"UAV{v}", va="center", fontsize=12, fontweight="bold", color=INK)
        fig.patches.append(Rectangle((BX0, y - 0.009), BX1 - BX0, 0.018, transform=fig.transFigure,
                                     facecolor="#eceff3", edgecolor="none"))
        bat_bar[v] = Rectangle((BX0, y - 0.009), 0.0, 0.018, transform=fig.transFigure, facecolor=ucolor[v],
                               edgecolor="none")
        fig.patches.append(bat_bar[v])
        bat_val[v] = fig.text(RX1 - 0.012, y, "", ha="right", va="center", fontsize=12, color=INK)

    # ---- 右邊：圖例卡片 ----
    card(RX0, 0.015, RX1, 0.25)
    MID = (RX0 + RX1) / 2
    legend_head = {}
    for x0_, x1_, head in ((RX0 + 0.012, MID - 0.01, "UAV Trajectories"), (MID + 0.005, RX1 - 0.012, "Task Flags")):
        fig.patches.append(FancyBboxPatch((x0_, 0.2), x1_ - x0_, 0.035, boxstyle="round,pad=0,rounding_size=0.008",
                                          transform=fig.transFigure, facecolor=NAVY, edgecolor=NAVY))
        legend_head[head] = fig.text((x0_ + x1_) / 2, 0.2175, head, ha="center", va="center", color="white",
                                     fontsize=11, fontweight="bold")
    fig.add_artist(Line2D([MID - 0.002, MID - 0.002], [0.03, 0.235], color="#d5dbe4", lw=1.0,
                          transform=fig.transFigure))
    # 左半：最後一格是 UAV 航跡的線色；影片途中沒有航跡，改成載具的圖例（2026-11-07 使用者）
    legend_traj, legend_agent = [], []
    for i, v in enumerate(ids):
        y = 0.165 - i * 0.045
        legend_traj.append(fig.add_artist(Line2D([RX0 + 0.018, RX0 + 0.05], [y, y], color=ucolor[v], lw=3,
                                                 transform=fig.transFigure)))
        legend_traj.append(fig.text(RX0 + 0.058, y, f"UAV{v} trajectory", va="center", fontsize=10.5, color=INK))
    for i, (kind, text) in enumerate((("uav", "UAV (UAV1–UAV3)"), ("dog", "Robot dog (dog1–dog3)"),
                                      ("arrow", "Moving direction"))):
        y = 0.165 - i * 0.045
        axi = fig.add_axes([RX0 + 0.02, y - 0.016, 0.026, 0.032])
        axi.axis("off")
        axi.set_zorder(5)
        axi.set_xlim(-1, 1)
        axi.set_ylim(-1, 1)
        if kind == "uav":
            axi.scatter([0.0], [0.0], s=130, marker=QUAD, color=VEHICLE, edgecolors=INK, linewidths=0.4)
        elif kind == "dog":
            axi.scatter([0.0], [0.0], s=300, marker=DOG_ICON, color="#7a3e12", linewidths=0)
        else:
            axi.annotate("", (0.8, 0.0), xytext=(-0.8, 0.0), arrowprops=dict(
                arrowstyle="-|>,head_length=0.6,head_width=0.3", color="#d0021b", lw=1.6, shrinkA=0, shrinkB=0))
        legend_agent.append(axi)
        legend_agent.append(fig.text(RX0 + 0.058, y, text, va="center", fontsize=10.5, color=INK))
    for i, (kind, text) in enumerate((("T", "UAV task (T)"), ("G", "Fixed task (G)"), ("F", "Fire task (F)"),
                                      ("M", "Manual assignment (M)"), ("P", "Dog patrol (A/B/C)"))):
        y = 0.172 - i * 0.032
        axf = fig.add_axes([MID + 0.008, y - 0.015, 0.018, 0.03])
        axf.axis("off")
        axf.set_zorder(5)          # 在卡片的底色上面
        axf.scatter([0.0], [-0.6], s=520, marker=FLAG, color=FLAG_COLOR[kind], edgecolors="#222222", linewidths=0.5)
        axf.set_xlim(-1, 1)
        axf.set_ylim(-1, 1)
        fig.text(MID + 0.03, y, text, va="center", fontsize=10.5, color=INK)

    def final_rows():
        """最後一格：每個做過任務的 agent → 做完的任務（照完成順序）、✓ Completed；UAV 一律列，狗只列做過的"""
        out = []
        for v in ids:
            mine = [names[tid] for tid, (tt, vv) in sorted(done.items(), key=lambda kv: kv[1][0])
                    if vv == v and tid in names]
            out.append((f"UAV{v}", ", ".join(mine) if mine else "—", "✓ Done" if mine else "—"))
        for d in sorted(dog):
            mine = [ground_name[g] for g in sorted(ground_name, key=lambda g: closed.get(g, 1e18))
                    if g in closed and any(tid == g for _, tid in dog_assigned.get(d, []))]
            if mine:
                out.append((dname(d), ", ".join(mine), "✓ Done"))
        return out

    keys = [e[0] for e in events]
    freezes = [f["spawned"] for f in fires] + [tasks[g]["t"] for g in ground] + list(closed.values())
    times = frame_times(t_start, t_end, keys, freezes, args.fps, args.speedup, args.slow, args.window, args.freeze)
    agents = [("uav", v, f"UAV{v}") for v in ids] + [("dog", d, dname(d)) for d in sorted(dog)]

    def set_rows(rows):
        status_color = {"Done": "#1d8a3a", "Executing": "#c25a00", "En Route": "#1f5fbf"}
        for r, (name, task_t, stat_t) in enumerate(row_art):
            vis = r < len(rows)
            for art in (name, task_t, stat_t):
                art.set_visible(vis)
            if not vis:
                continue
            label, task, status = rows[r]
            name.set_text(label)
            name.set_color(agent_color(label))
            task_t.set_text(task)
            stat_t.set_text(status)
            stat_t.set_color(status_color.get(status.replace("✓ ", ""), INK))

    head_art = {}

    def point_arrow(key, p, ang, show):
        """跟著載具移動的小箭頭（2026-11-07 使用者：不畫路線，用小箭頭表示移動方向）：移動中才顯示"""
        if key not in head_art:
            head_art[key] = ax.annotate("", (0, 0), xytext=(0, 0), zorder=8, arrowprops=dict(
                arrowstyle="-|>,head_length=0.6,head_width=0.3", color="#d0021b", lw=1.6, shrinkA=0, shrinkB=0))
        a_ = head_art[key]
        a_.set_visible(show)
        if show:
            r0, r1 = 1.6, 4.2           # 從圖示外緣開始，箭頭長約 2.6 m
            a_.set_position((p[0] + r0 * math.cos(ang), p[1] + r0 * math.sin(ang)))
            a_.xy = (p[0] + r1 * math.cos(ang), p[1] + r1 * math.sin(ang))

    def draw(t, final=False):
        ax.set_position(MAP_FINAL if final else MAP_VIDEO)
        for art in legend_traj:
            art.set_visible(final)
        for art in legend_agent:
            art.set_visible(not final)
        legend_head["UAV Trajectories"].set_text("UAV Trajectories" if final else "Agents")
        for art in banner:
            art.set_visible(final)
        for f, mk, lb in flag_art:
            vis = f["t"] <= t
            mk.set_visible(vis)
            lb.set_visible(vis)
            finished = (dog_patrol.is_done(f["done_ivs"], t) if f["kind"] == "P" else
                        f["done"] is not None and f["done"] <= t)
            mk.set_facecolor(FLAG_DONE if finished else FLAG_COLOR[f["kind"]])
            mk.set_edgecolor(FLAG_DONE_EDGE if finished else "#222222")
            lb.set_color(FLAG_DONE_TEXT if finished else INK)
        for v in ids:
            p = pos_at(pos[v], t)
            if p is None:
                continue
            h = heading(pos[v], t)
            if h is not None:
                uav_head[v] = h
            uav_art[v].set_offsets([p])
            uav_art[v].set_visible(not final)
            point_arrow(("uav", v), p, uav_head[v], not final and h is not None)
            uav_tag[v].xy = p
            uav_tag[v].set_visible(not final)
            if final:
                tr = [q for q in pos[v] if t_start - 1 <= q[0] <= t]
                trail_art[v].set_data([q[1] for q in tr], [q[2] for q in tr])
                trail_art[v].set_visible(True)
                draw_arrows(v, [q[1] for q in tr], [q[2] for q in tr])
        for d in dog:
            p = pos_at(dog[d], t)
            if p is None:
                continue
            dog_art[d].set_offsets([p])
            dh = heading(dog[d], t)
            point_arrow(("dog", d), p, dh if dh is not None else 0.0, not final and dh is not None)
            dog_tag[d].xy = p
            dog_tag[d].set_visible(True)
            if d in det_art:
                hit = next((f for tr, f in dog_reports.get(d, []) if tr - det_dwell - 1.0 <= t <= tr + DETECT_HOLD), None)
                c = det_art[d]
                c.center = p
                c.set_visible(not final)
                c.set_edgecolor("#d0021b" if hit else "#7a3e12")
                c.set_linewidth(2.4 if hit else 1.0)
                c.set_linestyle("-" if hit else "--")
                c.set_alpha(0.95 if hit else 0.45)
                det_txt[d].set_text(f"{dname(d)} sees {hit}" if hit else "")
                det_txt[d].xy = (p[0], p[1] + det_r)
                det_txt[d].set_visible(bool(hit) and not final)
        if final:
            set_rows(final_rows())
            st_title.set_text("Live Agent Status — Snapshot")
            clock.set_text(f"t = {t - t_start:.1f} s (end)")
        else:
            set_rows([(label,) + (uav_status(a_id, t) if kind == "uav" else dog_status(a_id, t))
                      for kind, a_id, label in agents])
            st_title.set_text("Live Agent Status")
            clock.set_text(f"t = {t - t_start:.1f} s")
        for v in bat_bar:
            pct = at(battery.get(v, []), t, None)
            bat_bar[v].set_width((BX1 - BX0) * (0.0 if pct is None else max(0.0, min(pct, 100.0)) / 100.0))
            bat_val[v].set_text("" if pct is None else f"{pct:.1f}%")
        bat_title.set_text("Final Battery (%)" if final else "Battery (%)")

    if args.status:
        last = {}
        for i in range(int((t_end - t_start) * 2) + 1):
            t = t_start + i * 0.5
            for kind, a_id, label in agents:
                row = uav_status(a_id, t) if kind == "uav" else dog_status(a_id, t)
                if last.get(label) != row:
                    print(f"{t - t_start:6.1f} s  {label:5s} {row[0]:4s} {row[1]}")
                    last[label] = row
        return
    if args.at is not None:
        draw(t_start + args.at)
        fig.savefig(os.path.join(args.out_dir, "demo_at.png"), dpi=100, facecolor="white")
        return
    if args.final_only:
        pass
    elif args.frames:
        os.makedirs(args.frames, exist_ok=True)
        for i, t in enumerate(times):
            draw(t, final=(i == len(times) - 1) or t >= t_end)
            fig.savefig(os.path.join(args.frames, f"{i:05d}.png"), dpi=100, facecolor="white")
        with open(os.path.join(args.frames, "fps.txt"), "w") as f:
            f.write(str(args.fps))
        print(f"{len(times)} 格 → {args.frames}（{len(times) / args.fps:.0f} s 影片）")
    else:
        from matplotlib.animation import FuncAnimation, PillowWriter
        anim = FuncAnimation(fig, lambda i: draw(times[i], final=times[i] >= t_end), frames=len(times), blit=False)
        anim.save(os.path.join(args.out_dir, "demo.gif"), writer=PillowWriter(fps=args.fps), dpi=70)
    draw(t_end, final=True)
    fig.savefig(os.path.join(args.out_dir, "demo_final.png"), dpi=100, facecolor="white")


if __name__ == "__main__":
    main()
