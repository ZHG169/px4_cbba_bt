#!/usr/bin/env python3
"""在 Gazebo 俯瞰錄影（gazebo.mp4）上加標籤（2026-11-05）→ gazebo_labeled.mp4。

  python3 docker/itest/label_gazebo.py docker/itest/out/<時間>_demo

伺服器端的錄影鏡頭畫不出文字，yard 的鏡頭又在 300 m 高（1 px 約 0.11 m，無人機只有幾個 px），所以錄完再加：
  - 無人機 uav1～N、狗 dog1～N（依機號排序，和 2D 動畫一樣）：黑色圓圈＋白底黑字的名字（2026-11-05 使用者：不需要太多顏色，
    畫面上的顏色只留給任務）
  - 任務：G 固定巡檢（綠）、M 人工指派（藍）、T 插入任務（黃）、F 火警（紅），和 Gazebo 的地面標記同色；做完的標籤拿掉
    （2026-11-07 使用者：itest 在做完時刪掉 Gazebo 的標記，標籤跟著消失；原本變灰、打勾）
  - 誰接了哪個任務：只在字幕（「uav1 ← G2」）；連線 2026-11-05 拿掉（使用者：畫面太亂）
  - 左上角時間（模擬時間，從錄影開始起算）和事件字幕（時間下方）、右上角圖例
鏡頭照 docker/gz/gen_patrol_site.py 的 OVERVIEW_POSE／OVERVIEW_FOV（只支援正上方往下看：yard、demo_site），
地圖座標直接換算成像素（高度 z 的點照透視縮放）。

時間對齊：gazebo.mp4 用模擬時間錄，事件是牆上時間。有 clock 事件（ros_bridge 2026-11-05 起記 /clock）時照
模擬時間換算（模擬快慢會變也對得上）；沒有時用 demo_plan.json 的 record.t0、t1 線性換算（--t0、--t1 可以手動給）。
要主機的 ffmpeg、Pillow 和中文字型（Noto Sans CJK；沒有時用預設字型，中文會變方塊）。
"""
import argparse
import bisect
import importlib.util
import json
import math
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dog_patrol  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))

# 顏色：任務同 Gazebo 的地面標記；載具一律黑白（2026-11-05）
TASK_COLORS = {"G": "#1ab233", "M": "#2666e6", "T": "#f2bf0d", "F": "#d91a14", "P": "#8e44ad"}   # P：狗的巡檢點
INK, PAPER = "#111111", "#ffffff"
FONT_FILES = ["/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
              "/usr/share/fonts/noto-cjk/NotoSansCJK-Bold.ttc"]


def font(size):
    for f in FONT_FILES:
        if os.path.exists(f):
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def camera(world):
    """gen_patrol_site.py 的俯瞰鏡頭：(cx, cy, 高度, 水平視角)；不是正上方往下看時回傳 None"""
    spec = importlib.util.spec_from_file_location("gen_patrol_site", os.path.join(REPO, "docker", "gz", "gen_patrol_site.py"))
    g = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(g)
    pose = g.OVERVIEW_POSE.get(world)
    if not pose:
        return None
    x, y, z, _, pitch, yaw = (float(v) for v in pose.split())
    if abs(pitch - math.pi / 2) > 0.01 or abs(yaw - math.pi / 2) > 0.01:
        return None
    return x, y, z, g.OVERVIEW_FOV.get(world, 1.05)


def probe(path):
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                          "stream=width,height,r_frame_rate:format=duration", "-of", "json", path],
                         capture_output=True, text=True, check=True).stdout
    j = json.loads(out)
    s = j["streams"][0]
    num, den = (int(v) for v in s["r_frame_rate"].split("/"))
    return s["width"], s["height"], num / den, float(j["format"]["duration"])


class Track:
    """一台載具的位置：牆上時間 → (x, y, z)，中間線性內插"""

    def __init__(self):
        self.t, self.p = [], []

    def add(self, t, p):
        self.t.append(t)
        self.p.append(p)

    def at(self, t):
        if not self.t or t < self.t[0] - 1.0:
            return None
        i = bisect.bisect_right(self.t, t)
        if i == 0:
            return self.p[0]
        if i >= len(self.t):
            return self.p[-1]
        t0, t1 = self.t[i - 1], self.t[i]
        a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        return tuple(u + (v - u) * a for u, v in zip(self.p[i - 1], self.p[i]))


def load(out_dir):
    plan = json.load(open(os.path.join(out_dir, "demo_plan.json"), encoding="utf-8"))
    events = [json.loads(l) for l in open(os.path.join(out_dir, "events.jsonl"), encoding="utf-8") if l.strip()]
    events.sort(key=lambda e: e["t"])
    return plan, events


def build(plan, events, out_dir):
    """事件 → 載具軌跡、任務（出現、完成）、誰在做哪個任務、字幕"""
    uavs, dogs, clock = {}, {}, []
    for e in events:
        if e["kind"] == "pos":
            uavs.setdefault(e["uav"], Track()).add(e["t"], (e["x"], e["y"], e.get("z", 0.0)))
        elif e["kind"] == "dog":
            dogs.setdefault(e["dog"], Track()).add(e["t"], (e["x"], e["y"], 0.0))
        elif e["kind"] == "clock":
            clock.append((e["t"], e["sim"]))
    dog_name = {d: f"dog{i + 1}" for i, d in enumerate(sorted(dogs))}

    closed = {}                                   # task_id → 第一台確認結束的時刻
    for e in events:
        if e["kind"] == "closed":
            closed.setdefault(e["msg"].get("task_id"), e["t"])

    tasks, by_id = [], {}                         # 每個任務：label、kind、x、y、z、t（出現）、done（完成）
    for p in plan.get("patrol", []):
        name = p.get("point", "")
        # 場景的固定點（G1～G3）是 G；2026-11-05 有一版 itest 把情境檔的 G 也記成 kind T，照名稱修正
        kind = "G" if name.startswith("G") else (p.get("kind") or (name[:1] if name[:1] in "MT" else "T"))
        t = dict(label=name, kind=kind, x=p["x"], y=p["y"], z=p.get("z", 0.0) or 0.0, t=p.get("t", plan.get("t0", 0.0)),
                 done=closed.get(p.get("task_id")), where=p.get("where", ""))
        tasks.append(t)
        by_id[p.get("task_id")] = t
    for m in plan.get("manual") or []:
        if m.get("clear") or not m.get("ok", True):
            continue
        tasks.append(dict(label=m.get("label") or m.get("name", "M"), kind="M", x=m["x"], y=m["y"], z=m.get("z", 0.0) or 0.0, t=m["t"],
                          done=m.get("finished_t"), where=m.get("where", ""), uav=m.get("target")))
    # 狗的日常巡檢點（2026-11-07，demo_plan.json 的 dog[].patrol：A1～、B1～、C1～）：一開始就在；做完（雲台檢查完、
    # 往下一點走）拿掉，下一圈再出發去它時再出現（和 itest 刪掉、重新生成 Gazebo 標記同一套判斷，dog_patrol.py）
    for u in plan.get("dog") or []:
        pts = u.get("patrol") or []
        log = os.path.join(out_dir, f"dog_bt_{u['ns']}.log")
        ivs = dog_patrol.done_intervals(dog_patrol.parse(open(log, encoding="utf-8", errors="replace")),
                                        [q["name"] for q in pts]) if os.path.exists(log) else {}
        for q in pts:
            tasks.append(dict(label=q["name"], kind="P", x=q["x"], y=q["y"], z=0.0, t=float("-inf"), done=None, where="",
                              done_ivs=ivs.get(q["name"], [])))
    fires = sorted((f for f in plan.get("fires", []) if "x" in f and f.get("spawned")), key=lambda f: f["spawned"])
    for i, f in enumerate(fires):
        t = dict(label=f"F{i + 1}", kind="F", x=f["x"], y=f["y"], z=f.get("z", 0.0) or 0.0, t=f["spawned"],
                 done=closed.get(f.get("ground_task")), where=f.get("where", ""), fire=f)
        tasks.append(t)
        if f.get("ground_task"):
            by_id[f["ground_task"]] = t

    # 誰在做哪個任務：(t, 載具名稱, task 或 None)
    doing = []
    for e in events:
        if e["kind"] == "assigned":
            doing.append((e["t"], f"uav{e['uav']}", by_id.get(e["msg"].get("task_id"))))
        elif e["kind"] == "dog_assigned":
            doing.append((e["t"], dog_name.get(e["dog"], f"dog{e['dog']}"), by_id.get(e["msg"].get("task_id"))))

    # 字幕
    subs = []
    seen = set()
    for t_, who, task in doing:
        if task and (who, task["label"]) not in seen:
            seen.add((who, task["label"]))
            subs.append((t_, f"{who} ← {task['label']}", task["kind"]))
    for e in events:
        if e["kind"] == "result" and e["msg"].get("success") and e["msg"].get("task_id") in by_id:
            task = by_id[e["msg"]["task_id"]]
            subs.append((e["t"], f"uav{e['uav']} 完成 {task['label']}", task["kind"]))
    for task in tasks:
        where = f"（{task['where']}）" if task.get("where") else ""
        if task["kind"] == "M":
            subs.append((task["t"], f"人工指派 {task['label']} → {task.get('uav', '')}{where}", "M"))
        elif task["kind"] == "T":
            subs.append((task["t"], f"插入任務 {task['label']}{where}", "T"))
        elif task["kind"] == "F":
            subs.append((task["t"], f"起火 {task['label']}{where}", "F"))
            f = task["fire"]
            if f.get("reported_t"):
                who = f.get("reported_by", "?")
                who = who if isinstance(who, str) else f"uav{who}"     # 狗的模擬偵測（2026-11-05）是 "dogN"
                subs.append((f["reported_t"], f"{who} 發現 {task['label']} → 地面任務", "F"))
            if task["done"]:
                subs.append((task["done"], f"{task['label']} 處理完（地面任務結束）", "F"))
    subs.sort(key=lambda s: s[0])
    return uavs, dogs, dog_name, clock, tasks, doing, subs


def wall_clock(plan, clock, duration, t0_arg, t1_arg):
    """影片時間（s）→ 牆上時間"""
    rec = plan.get("record") or {}
    t0 = t0_arg or rec.get("t0")
    t1 = t1_arg or rec.get("t1")
    if t0 is None:
        sys.exit("demo_plan.json 沒有錄影的開始時間（record.t0；2026-11-05 以前的輸出）：用 --t0、--t1 給牆上時間")
    if len(clock) >= 2:
        ts, sims = [c[0] for c in clock], [c[1] for c in clock]

        def sim_at(t):
            i = min(max(bisect.bisect_right(ts, t), 1), len(ts) - 1)
            a = (t - ts[i - 1]) / (ts[i] - ts[i - 1]) if ts[i] > ts[i - 1] else 0.0
            return sims[i - 1] + (sims[i] - sims[i - 1]) * a
        sim0 = sim_at(t0)

        def wall(tv):
            target = sim0 + tv
            i = min(max(bisect.bisect_right(sims, target), 1), len(sims) - 1)
            a = (target - sims[i - 1]) / (sims[i] - sims[i - 1]) if sims[i] > sims[i - 1] else 0.0
            return ts[i - 1] + (ts[i] - ts[i - 1]) * a
        return wall, t0, "模擬時鐘"
    if t1 is None:
        sys.exit("沒有 clock 事件，也沒有錄影的結束時間（record.t1）：用 --t1 給")
    return (lambda tv: t0 + tv * (t1 - t0) / duration), t0, "線性（沒有 clock 事件）"


def draw_tag(d, xy, text, fill, fnt, fg=PAPER, anchor="mb", border=None):
    """名字的小方框：xy 是框的下緣中點（anchor mb）或上緣中點（mt）"""
    x, y = xy
    l, t, r, b = d.textbbox((0, 0), text, font=fnt)
    w, h = r - l + 8, b - t + 4
    top = y - h if anchor == "mb" else y
    d.rounded_rectangle([x - w / 2, top, x + w / 2, top + h], radius=3, fill=fill, outline=border, width=1 if border else 0)
    d.text((x - (r - l) / 2 - l, top + 2 - t), text, font=fnt, fill=fg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir")
    ap.add_argument("--video", default="gazebo.mp4")
    ap.add_argument("--output", default="gazebo_labeled.mp4")
    ap.add_argument("--t0", type=float, default=None, help="錄影開始的牆上時間（沒有 record.t0 時）")
    ap.add_argument("--t1", type=float, default=None, help="錄影結束的牆上時間（沒有 clock 事件、record.t1 時）")
    args = ap.parse_args()

    src = os.path.join(args.out_dir, args.video)
    plan, events = load(args.out_dir)
    cam = camera(plan.get("world", ""))
    if cam is None:
        sys.exit(f"場景 {plan.get('world')} 的俯瞰鏡頭不是正上方往下看，不支援")
    W, H, fps, duration = probe(src)
    cx, cy, ch, fov = cam
    focal = (W / 2) / math.tan(fov / 2)

    def px(x, y, z=0.0):
        s = focal / (ch - z)
        return W / 2 + s * (x - cx), H / 2 - s * (y - cy)

    uavs, dogs, dog_name, clock, tasks, doing, subs = build(plan, events, args.out_dir)
    wall, _, how = wall_clock(plan, clock, duration, args.t0, args.t1)
    f_tag, f_small, f_hud, f_sub = font(13), font(11), font(18), font(17)
    tracks = {f"uav{v}": tr for v, tr in uavs.items()}
    tracks.update({dog_name[d]: tr for d, tr in dogs.items()})

    # 狗的偵測範圍（2026-11-09 使用者：影片看不出狗「看得到」火）：demo.dog_detect.radius（10 m）的圓跟著狗；
    # 平常淡色虛線，發現火的前後（持續 dwell 秒開始、回報後 4 s）那隻狗的圓變紅、實線，標「dogN 看到 Fk」
    det = plan.get("dog_detect") or {}
    if not det:
        try:
            det = json.load(open(os.path.join(HERE, "config.json"), encoding="utf-8"))["demo"].get("dog_detect") or {}
        except (OSError, KeyError, ValueError):
            det = {}
    det_r, det_dwell = float(det.get("radius", 0.0)), float(det.get("dwell", 1.0))
    fire_tasks = [x for x in tasks if x["kind"] == "F"]
    dog_seen = {}                    # dogN → [(回報時刻, 火的編號)]
    for e in events:
        if e.get("kind") == "new_task" and e.get("source") == "dog_detect" and e.get("dog") in dog_name:
            pxy = e["msg"]["position"]
            f = next((x["label"] for x in fire_tasks if math.hypot(x["x"] - pxy["x"], x["y"] - pxy["y"]) <= 1.0), "火")
            dog_seen.setdefault(dog_name[e["dog"]], []).append((e["t"], f))

    dec = subprocess.Popen(["ffmpeg", "-v", "error", "-i", src, "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                           stdout=subprocess.PIPE)
    out = os.path.join(args.out_dir, args.output)
    enc = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                            "-r", f"{fps:g}", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out],
                           stdin=subprocess.PIPE)
    frame_bytes = W * H * 3
    k = 0
    while True:
        raw = dec.stdout.read(frame_bytes)
        if len(raw) < frame_bytes:
            break
        t = wall(k / fps)
        im = Image.frombytes("RGB", (W, H), raw)
        d = ImageDraw.Draw(im, "RGBA")

        # 任務（出現之後才畫）
        for task in tasks:
            if t < task["t"] or (task["done"] is not None and t >= task["done"]) or \
                    (task["kind"] == "P" and dog_patrol.is_done(task["done_ivs"], t)):
                continue                    # 還沒出現、已經做完（Gazebo 的標記也刪掉了）都不畫
            color = TASK_COLORS[task["kind"]]
            u, v = px(task["x"], task["y"], task["z"])
            r = 16 if task["kind"] == "F" else 9 if task["kind"] == "P" else 13      # 巡檢點的圓盤小一號（直徑 2 m）
            d.ellipse([u - r, v - r, u + r, v + r], outline=color, width=3)
            # 火的名字放在圓圈上方、任務放下方：任務點同時起火（T2 + F2）時不會疊在一起
            if task["kind"] == "F":
                draw_tag(d, (u, v - r - 2), task["label"], color, f_tag)
            else:
                draw_tag(d, (u, v + r + 2), task["label"], color, f_tag, anchor="mt")

        # 狗的偵測範圍（在載具底下）
        if det_r > 0:
            for who, tr in tracks.items():
                if not who.startswith("dog"):
                    continue
                p = tr.at(t)
                if p is None:
                    continue
                u, v = px(*p[:2])
                rr = det_r * focal / ch
                hit = next((f for tr_, f in dog_seen.get(who, []) if tr_ - det_dwell - 1.0 <= t <= tr_ + 4.0), None)
                box = [u - rr, v - rr, u + rr, v + rr]
                if hit:
                    d.ellipse(box, outline=TASK_COLORS["F"] + "f0", width=4)
                    draw_tag(d, (u, v - rr - 4), f"{who} 看到 {hit}", TASK_COLORS["F"], f_tag)
                else:
                    for a in range(0, 360, 16):           # 虛線：每 16° 畫 8°
                        d.arc(box, a, a + 8, fill="#7a3e12a0", width=2)

        # 載具
        for who, tr in tracks.items():
            p = tr.at(t)
            if p is None:
                continue
            u, v = px(*p)
            r = 11 if who.startswith("uav") else 9
            d.ellipse([u - r, v - r, u + r, v + r], outline=PAPER, width=4)
            d.ellipse([u - r, v - r, u + r, v + r], outline=INK, width=2)
            draw_tag(d, (u, v - r - 2), who, PAPER, f_small, fg=INK, border=INK)

        # 左上：影片時間（影片照模擬時間錄，播放的快慢和這個時間一致）；右上：圖例
        sec = k / fps
        draw_tag(d, (60, 8), f"{int(sec // 60):02d}:{int(sec % 60):02d}", INK + "c0", f_hud, anchor="mt")
        legend = [("G 固定巡檢", TASK_COLORS["G"]), ("M 人工指派", TASK_COLORS["M"]), ("T 插入任務", TASK_COLORS["T"]),
                  ("F 火警", TASK_COLORS["F"])] + ([("狗巡檢點", TASK_COLORS["P"])] if any(x["kind"] == "P" for x in tasks) else [])
        lx, ly = W - 132, 8
        d.rounded_rectangle([lx - 8, ly - 4, W - 6, ly + 22 * len(legend) + 2], radius=4, fill=PAPER + "d8")
        for j, (text, c) in enumerate(legend):
            yy = ly + 22 * j
            d.ellipse([lx, yy + 3, lx + 14, yy + 17], outline=c, width=3)
            d.text((lx + 22, yy), text, font=f_small, fill=INK)

        # 左上（時間下方）：最近 8 s 的事件（最多 3 則，最新的在最上面）。2026-11-05 使用者：放在下方會被播放器的
        # 進度列擋住
        recent = [s for s in subs if t - 8.0 <= s[0] <= t][-3:]
        for j, (_, text, kind) in enumerate(reversed(recent)):
            yy = 44 + 28 * j
            l_, t_, r_, b_ = d.textbbox((0, 0), text, font=f_sub)
            d.rounded_rectangle([12, yy - 3, 12 + r_ - l_ + 26, yy + b_ - t_ + 7], radius=4, fill=INK + "c8")
            d.rectangle([12, yy - 3, 18, yy + b_ - t_ + 7], fill=TASK_COLORS.get(kind, PAPER))
            d.text((24 - l_, yy + 2 - t_), text, font=f_sub, fill=PAPER)

        enc.stdin.write(im.tobytes())
        k += 1
    enc.stdin.close()
    enc.wait()
    dec.wait()
    print(f"{out}（{k} 格，時間對齊：{how}）")


if __name__ == "__main__":
    main()
