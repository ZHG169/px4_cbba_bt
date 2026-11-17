#!/usr/bin/env python3
"""itest group 的影片（2026-11-16；2026-11-17 照〈CBBA G0～G15 測試與影片規劃〉第 8 頁重新編排）。

    python3 animate_group.py <itest 輸出目錄> --frames <目錄> [--fps 10]

真的 cbba_node＋閘道，載具是 ros_bridge 的假身體（不用真的 BT）。讀 events.jsonl 與 group_plan.json
（每段測試的起訖、預期、角色；itest 的 note：key = 關鍵狀態改變、timer = 計時器、event = detection；results = 各項結果），
照章節重新排列成影片（規劃的 80～90 s × --slow，預設 1.8 倍 ≈ 2.5 分鐘，2026-11-17 使用者：原本太快）：

  0:00 測試架構（3 UAV＋3 Dog＋Gateway、圖例）   ① 異質可行性 G8   ② 必要角色完整性 G4＋G5   ③ Optional 降級 G12
  ④ 時序協同 G1＋G3   ⑤ 一致與恢復 G9、G10、G11、G13（等待片段加速＋計時器）   G14 端到端   G15 跨平台交接   結論（G0～G15 狀態總覽）

影片原則：每段先顯示「Expected」；關鍵狀態改變時凍結 1.5 s 並顯示 PASS／FAIL 證據；倍速依章節長度自動算（等待多的段落較快）；
G14、G15 標出偵測來源（星號＋SOURCE）與最後的執行者。圖示和 demo 動畫一樣（animate_demo.py 的無人機、狗、旗子）。
文字用英文：容器沒有中文字型。畫面最下面 12 % 留白，播放器的進度列不會擋到內容。每一格寫成 frames/%05d.png 與 fps.txt，itest 用主機的 ffmpeg 合成 group.mp4。
"""
import argparse
import json
import os
import sys
from collections import deque

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from animate_demo import DOG_ICON, FLAG, QUAD  # noqa: E402

# 旗子顏色照 demo（F 紅、M 藍、T 黃、G 綠）依優先級；做完淺灰
PRIO_COLOR = {0: "#d91a14", 1: "#2666e6", 2: "#f2bf0d", 3: "#1ab233"}
FLAG_DONE = "#c8c8c8"
UAV_COLOR = "#3a3a3a"
DOG_COLOR = "#7a3e12"
SOURCE = "#b0179a"
PASS_GREEN = "#1e8e3e"
FAIL_RED = "#c62828"

CHAPTERS = [
    ("Test setup", [], 4.0),
    ("1  Heterogeneous feasibility", ["G8"], 7.0),
    ("2  Mandatory role completeness", ["G4", "G5"], 11.0),
    ("3  Optional degradation", ["G12"], 6.0),
    ("4  Timing coordination", ["G1", "G3"], 12.0),
    ("5  Consistency & recovery", ["G9", "G10", "G11", "G13"], 22.0),
    ("G14  End-to-end detection", ["G14"], 10.0),
    ("G15  Cross-platform handover", ["G15"], 10.0),
    ("Conclusion", [], 4.0),
]
NAMES = {
    "G0": "Nodes + gateway online", "G1": "Precedence", "G2": "Incident revision", "G3": "Sync point",
    "G4": "Distinct agents", "G5": "All-or-nothing", "G6": "TaskSpec partial revision", "G7": "Fallback rebuild",
    "G8": "Heterogeneous roles", "G9": "Holder lost: takeover", "G10": "Takeover timeout: fallback",
    "G11": "Cascade cancel", "G12": "Optional skipped", "G13": "Priority preemption",
    "G14": "Dog detects person -> intrusion", "G14D": "Detection dedup", "G15": "UAV detects fire -> dog",
    "G15R": "Repair after rejoin",
}
EXPECT_SEC = 2.5      # 每段開頭顯示「預期」（2026-11-17 使用者：太快，1.2 → 2.5 s）
KEY_SEC = 2.5         # 關鍵狀態凍結（1.5 → 2.5 s）
MAX_SPEED = 5.0       # 最高倍速（8 → 5；規劃寫等待片段 4～6 倍）
SLOW = 1.8            # 章節長度乘這個倍數（--slow；規劃的 86 s × 1.8 ≈ 2.5 分鐘）
BOTTOM = 0.12         # 畫面最下面留白（播放器的進度列會蓋住這一條）


def load(out):
    with open(os.path.join(out, "group_plan.json"), encoding="utf-8") as f:
        plan = json.load(f)
    events = []
    with open(os.path.join(out, "events.jsonl"), encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    events.sort(key=lambda e: e.get("t", 0.0))
    return plan, events


class State:
    """依時間重播事件：位置、手上的任務、gate、斷線、mode、任務結束、偵測、計時器"""

    def __init__(self, plan):
        self.vehicles = {int(k): v for k, v in plan["vehicles"].items()}
        self.pos, self.trail, self.cur, self.gates, self.mode, self.closed = {}, {}, {}, {}, {}, {}
        self.offline, self.arrived = set(), set()
        self.detections = []     # (t, 機號, x, y, label)
        self.timer = None        # (開始時刻, 名稱)

    def apply(self, e):
        k, v = e["kind"], e.get("uav")
        if k == "pos" and v in self.vehicles:
            self.pos[v] = (e["x"], e["y"])
            tr = self.trail.setdefault(v, deque())
            tr.append((e["t"], e["x"], e["y"]))
            while tr and e["t"] - tr[0][0] > 6.0:
                tr.popleft()
        elif k == "assigned" and v in self.vehicles:
            m = e["msg"]
            self.cur[v] = 0 if int(m.get("status", 0)) == 2 else int(m.get("task_id", 0))
        elif k == "teammates" and v in self.vehicles:
            self.gates[v] = e["msg"]
        elif k == "closed":
            m = e["msg"]
            self.closed.setdefault(int(m.get("task_id", 0)), (int(m.get("status", 0)), e["t"]))
        elif k == "arrived":
            self.arrived.add((v, int(e["task_id"])))
        elif k == "body_hold":
            (self.offline.add if e["hold"] else self.offline.discard)(v)
        elif k == "body_mode":
            self.mode[v] = e["mode"]
        elif k == "note":
            if e.get("event") == "detection":
                self.detections.append((e["t"], e.get("vehicle"), e["x"], e["y"], e.get("label", "")))
            if e.get("timer"):
                self.timer = (e["t"], e["timer"])

    def gate_text(self, vid, tid):
        msg = self.gates.get(vid) or {}
        if int(msg.get("task_id", 0)) != tid:
            return ""
        gates = msg.get("gates") or []
        if any(int(g.get("state", 0)) == 1 for g in gates):
            return "WAITING"
        if any(int(g.get("state", 0)) == 2 for g in gates):
            return "UNSAT"
        return "OPEN" if gates else ""


def segments_by_id(plan):
    return {s["id"]: s for s in plan["segments"]}


def keys_of(events, seg):
    return [e for e in events if e["kind"] == "note" and e.get("key") == seg["id"] and seg["t0"] <= e["t"] <= seg["t1"]]


def timeline(plan, events, fps, slow=SLOW):
    """影片的每一格：(章節, 段落 id 或 None, 模擬時刻, 疊加資訊)。段落照章節重排；倍速依章節長度"""
    segs = segments_by_id(plan)
    frames = []
    for title, ids, target in CHAPTERS:
        target *= slow
        present = [segs[i] for i in ids if i in segs and segs[i].get("t1")]
        if not ids:
            frames += [(title, None, None, {"kind": title})] * int(target * fps)
            continue
        if not present:
            continue
        real = sum(s["t1"] - s["t0"] for s in present)
        for s in present:
            dur = s["t1"] - s["t0"]
            keys = keys_of(events, s)
            alloc = target * dur / real
            play = max(1.0, alloc - EXPECT_SEC - KEY_SEC * len(keys))
            speed = min(MAX_SPEED, max(1.0, dur / play))
            frames += [(title, s["id"], s["t0"], {"kind": "expect", "speed": speed})] * int(EXPECT_SEC * fps)
            t = s["t0"]
            ki = 0
            while t <= s["t1"]:
                while ki < len(keys) and keys[ki]["t"] <= t:
                    frames += [(title, s["id"], keys[ki]["t"], {"kind": "key", "text": keys[ki]["text"],
                                                                 "speed": speed})] * int(KEY_SEC * fps)
                    ki += 1
                frames.append((title, s["id"], t, {"kind": "play", "speed": speed}))
                t += speed / fps
            for k in keys[ki:]:
                frames += [(title, s["id"], k["t"], {"kind": "key", "text": k["text"], "speed": speed})] * int(KEY_SEC * fps)
    return frames


def bounds(plan, events, ids):
    segs = segments_by_id(plan)
    spans = [(segs[i]["t0"], segs[i].get("t1", segs[i]["t0"])) for i in ids if i in segs]
    xs, ys = [], []
    for e in events:
        if e["kind"] == "pos" and any(a <= e["t"] <= b for a, b in spans):
            xs.append(e["x"])
            ys.append(e["y"])
    for t in plan["tasks"].values():
        if t.get("segment") in ids:
            xs.append(t["x"])
            ys.append(t["y"])
    if not xs:
        return (-10, 30, -10, 30)
    x0, x1, y0, y1 = min(xs) - 4, max(xs) + 4, min(ys) - 4, max(ys) + 6
    w, h = x1 - x0, y1 - y0
    if w > h:
        y0, y1 = y0 - (w - h) / 2, y1 + (w - h) / 2
    else:
        x0, x1 = x0 - (h - w) / 2, x1 + (h - w) / 2
    return x0, x1, y0, y1


def legend(ax, x, y):
    ax.scatter([x], [y], s=260, marker=QUAD, color=UAV_COLOR, transform=ax.transAxes, clip_on=False)
    ax.text(x + 0.04, y, "UAV", transform=ax.transAxes, va="center", fontsize=10)
    ax.scatter([x + 0.16], [y], s=420, marker=DOG_ICON, color=DOG_COLOR, transform=ax.transAxes, clip_on=False)
    ax.text(x + 0.21, y, "Dog", transform=ax.transAxes, va="center", fontsize=10)
    for i, (p, c) in enumerate(PRIO_COLOR.items()):
        ax.scatter([x + 0.34 + 0.11 * i], [y - 0.02], s=380, marker=FLAG, color=c, edgecolors="#222222", linewidths=0.5,
                   transform=ax.transAxes, clip_on=False)
        ax.text(x + 0.37 + 0.11 * i, y, f"P{p}", transform=ax.transAxes, va="center", fontsize=10)
    ax.scatter([x + 0.80], [y - 0.02], s=380, marker=FLAG, color=FLAG_DONE, edgecolors="#222222", linewidths=0.5,
               transform=ax.transAxes, clip_on=False)
    ax.text(x + 0.83, y, "done", transform=ax.transAxes, va="center", fontsize=10)


def draw_card(fig, plan, kind):
    ax = fig.add_axes([0, BOTTOM, 1, 1 - BOTTOM])    # 底部留白：不被播放器的進度列擋住
    ax.axis("off")
    if kind == "Test setup":
        ax.text(0.5, 0.80, "Team CBBA integration test  (G0 - G15)", ha="center", fontsize=26, fontweight="bold")
        nu = sum(v["platform"] == "uav" for v in plan["vehicles"].values())
        nd = sum(v["platform"] == "dog" for v in plan["vehicles"].values())
        ax.text(0.5, 0.68, f"{nu} UAV + {nd} Dog + Gateway   |   real cbba_node + gateway, simulated bodies (no BT)",
                ha="center", fontsize=14)
        ax.text(0.5, 0.60, "Parameters: w_E 25, half-life 60/120/300/600 s, UAV 0.5 m/s, dog 1.0 m/s, no packet loss",
                ha="center", fontsize=12, color="#555555")
        ax.text(0.5, 0.46, "Each part: Expected  ->  run  ->  PASS evidence (frozen 1.5 s)", ha="center", fontsize=13)
        legend(ax, 0.22, 0.30)
        return
    ax.text(0.5, 0.92, "Results  G0 - G15", ha="center", fontsize=24, fontweight="bold")
    res = plan.get("results", {})
    codes = ["G0", "G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8", "G9", "G10", "G11", "G12", "G13", "G14", "G14D",
             "G15", "G15R"]
    for i, c in enumerate(codes):
        col, row = i // 9, i % 9
        st = res.get(c, "not run")
        color = PASS_GREEN if st == "PASS" else FAIL_RED if st == "FAIL" else "#777777"
        x, y = 0.10 + col * 0.45, 0.80 - row * 0.075
        ax.text(x, y, c, fontsize=13, fontweight="bold", family="monospace")
        ax.text(x + 0.07, y, NAMES.get(c, ""), fontsize=12)
        ax.text(x + 0.36, y, st, fontsize=13, color=color, fontweight="bold")
    n_pass = sum(v == "PASS" for v in res.values())
    ax.text(0.5, 0.06, f"{n_pass} / {len(res)} PASS   (items not run are not marked PASS)", ha="center", fontsize=13)


def draw(fig, plan, st, tasks, chapter, seg, t, info, lim):
    fig.clf()
    if seg is None:
        draw_card(fig, plan, info["kind"])
        return
    ax = fig.add_axes([0.03, BOTTOM + 0.02, 0.53, 0.88 - BOTTOM - 0.02])
    side = fig.add_axes([0.59, BOTTOM + 0.02, 0.39, 0.88 - BOTTOM - 0.02])
    top = fig.add_axes([0, 0.9, 1, 0.1])
    for a in (side, top):
        a.axis("off")
        a.set_xlim(0, 1)
        a.set_ylim(0, 1)
    ax.set_xlim(lim[0], lim[1])
    ax.set_ylim(lim[2], lim[3])
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.tick_params(labelsize=8)
    result = plan.get("results", {}).get(seg["id"], "")
    top.text(0.02, 0.62, chapter, fontsize=16, fontweight="bold", va="center")
    top.text(0.02, 0.18, f"{seg['id']}  {seg['title']}", fontsize=12, va="center", color="#333333")
    top.text(0.98, 0.62, f"x{info['speed']:.0f}" if info["kind"] == "play" and info["speed"] > 1.05 else "",
             fontsize=14, ha="right", va="center", color="#777777")

    # ---------- 地圖：這一段的任務旗子 ----------
    shown = []
    for tid, f in tasks.items():
        if f.get("segment") != seg["id"] or t < f["t"]:
            continue
        closed = st.closed.get(tid)
        if closed and t - closed[1] > 5.0:
            continue
        shown.append((tid, f))
        color = FLAG_DONE if closed else PRIO_COLOR.get(f["priority"], FLAG_DONE)
        ax.scatter([f["x"]], [f["y"]], s=700, marker=FLAG, color="white" if f["optional"] and not closed else color,
                   edgecolors=color if f["optional"] and not closed else "#222222", linewidths=1.5 if f["optional"] else 0.5,
                   zorder=3)
        tag = f["label"] + (" (opt)" if f["optional"] else "")
        if closed:
            tag += "  DONE" if closed[0] == 1 else ("  SKIPPED" if f["optional"] else "  CANCELLED")
        ax.annotate(tag, (f["x"], f["y"]), textcoords="offset points", xytext=(10, -14), fontsize=8,
                    color="#888888" if closed else "#222222")
    # 偵測（G14、G15）：星號＋來源
    for dt, vid, x, y, label in st.detections:
        if seg["t0"] <= dt <= t:
            ax.scatter([x], [y], s=380, marker="*", color=SOURCE, edgecolors="black", zorder=4)
            ax.annotate(f"{label} (detected by {st.vehicles.get(vid, {}).get('name', '?')})", (x, y),
                        textcoords="offset points", xytext=(10, 8), fontsize=9, color=SOURCE, fontweight="bold")
    # ---------- 載具 ----------
    source = {vid for dt, vid, *_ in st.detections if seg["t0"] <= dt <= t}
    for vid, v in st.vehicles.items():
        if vid not in st.pos:
            continue
        x, y = st.pos[vid]
        off, manual = vid in st.offline, st.mode.get(vid, 0) == 1
        color = UAV_COLOR if v["platform"] == "uav" else DOG_COLOR
        alpha = 0.3 if (off or manual) else 1.0
        tr = st.trail.get(vid)
        if tr and len(tr) > 1:
            ax.plot([p[1] for p in tr], [p[2] for p in tr], "-", color=color, alpha=0.25, lw=2, zorder=1)
        tid = st.cur.get(vid, 0)
        if tid in tasks and not off and tasks[tid].get("segment") == seg["id"]:
            ax.plot([x, tasks[tid]["x"]], [y, tasks[tid]["y"]], "--", color=color, lw=1, alpha=0.6, zorder=2)
        if vid in source:
            ax.scatter([x], [y], s=1300, facecolors="none", edgecolors=SOURCE, linewidths=2.5, zorder=4)
        if v["platform"] == "uav":
            ax.scatter([x], [y], s=360, marker=QUAD, color=color, alpha=alpha, zorder=5)
        else:
            ax.scatter([x], [y], s=650, marker=DOG_ICON, color=color, alpha=alpha, linewidths=0, zorder=5)
        tag = v["name"] + ("  OFFLINE" if off else "  MANUAL" if manual else "") + ("  SOURCE" if vid in source else "")
        ax.annotate(tag, (x, y), textcoords="offset points", xytext=(10, 8), fontsize=9,
                    color=FAIL_RED if off else SOURCE if vid in source else "black",
                    fontweight="bold" if (off or vid in source) else "normal", zorder=6)
        if off:
            ax.scatter([x], [y], s=500, marker="x", color=FAIL_RED, linewidths=3, zorder=6)

    # ---------- 右側：預期、角色、計時器 ----------
    y = 0.98
    side.text(0.0, y, "Expected", fontsize=11, fontweight="bold", va="top", color="#555555")
    side.text(0.0, y - 0.05, seg.get("expected", ""), fontsize=10, va="top", wrap=True,
              bbox=dict(boxstyle="round", fc="#eef3fb", ec="#9db4d8") if info["kind"] == "expect" else None)
    y -= 0.25
    side.text(0.0, y, f"{'role':<20}{'holder':<8}{'status':<10}gate", fontsize=8.5, family="monospace", va="top",
              color="#666666")
    y -= 0.045
    for tid, f in shown:
        holders = [st.vehicles[v]["name"] for v, c in st.cur.items() if c == tid and v in st.vehicles and v not in st.offline]
        closed = st.closed.get(tid)
        if closed:
            status = "DONE" if closed[0] == 1 else ("SKIPPED" if f["optional"] else "CANCELLED")
        elif holders:
            status = "working" if any((v, tid) in st.arrived for v, c in st.cur.items() if c == tid) else "en route"
        else:
            status = "open"
        gate = next((st.gate_text(v, tid) for v, c in st.cur.items() if c == tid and st.gate_text(v, tid)), "")
        color = FLAG_DONE if closed else PRIO_COLOR.get(f["priority"], "black")
        side.text(0.0, y, f"{(f['label'] + ' P' + str(f['priority']))[:19]:<20}{(holders[0] if holders else '-'):<8}"
                          f"{status:<10}{gate}", fontsize=8.5, family="monospace", va="top",
                  color="#888888" if closed else "black")
        side.scatter([-0.025], [y - 0.012], s=60, marker="s", color=color, clip_on=False)
        y -= 0.045
    if st.timer and seg["t0"] <= st.timer[0] <= t:
        side.text(0.0, 0.17, f"{st.timer[1]}: {t - st.timer[0]:5.1f} s", fontsize=15, fontweight="bold",
                  family="monospace", color="#b45f06")
    if info["kind"] == "key":
        ok = result in ("PASS", "")
        side.text(0.0, 0.07, (result + "  " if result else "") + info["text"], fontsize=11,
                  fontweight="bold", color="white", va="center", wrap=True,
                  bbox=dict(boxstyle="round,pad=0.5", fc=PASS_GREEN if ok else FAIL_RED, ec="none"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--frames", required=True)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--slow", type=float, default=SLOW, help="章節長度的倍數（1 = 規劃的 86 s；越大越慢）")
    args = ap.parse_args()
    plan, events = load(args.out)
    if not plan["segments"]:
        raise SystemExit("group_plan.json 沒有測試段落")
    for i, s in enumerate(plan["segments"]):
        s.setdefault("t1", plan["segments"][i + 1]["t0"] if i + 1 < len(plan["segments"]) else events[-1]["t"])
    tasks = {int(k): v for k, v in plan["tasks"].items()}
    frames = timeline(plan, events, args.fps, args.slow)
    os.makedirs(args.frames, exist_ok=True)
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    segs = segments_by_id(plan)
    lims = {}
    st, cur_seg, ei = None, None, 0
    for k, (chapter, sid, t, info) in enumerate(frames):
        if sid is not None and sid != cur_seg:
            # 換段落：從頭重播到這段開始（段落照章節重排，時間會往回跳）
            st, ei, cur_seg = State(plan), 0, sid
            chapter_ids = next(ids for title, ids, _ in CHAPTERS if title == chapter)
            lims[sid] = bounds(plan, events, chapter_ids)
        if sid is not None:
            while ei < len(events) and events[ei].get("t", 0.0) <= t:
                st.apply(events[ei])
                ei += 1
        draw(fig, plan, st, tasks, chapter, segs.get(sid), t, info, lims.get(sid))
        fig.savefig(os.path.join(args.frames, f"{k:05d}.png"))
    with open(os.path.join(args.frames, "fps.txt"), "w") as f:
        f.write(str(args.fps))
    print(f"{len(frames)} 格（{len(frames) / args.fps:.0f} s、{args.fps} fps）→ {args.frames}")


if __name__ == "__main__":
    main()
