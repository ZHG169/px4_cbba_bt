#!/usr/bin/env python3
"""itest group 的影片（2026-11-16；2026-11-20 照〈CBBA G0～G21 三個模擬場景〉第 3～7 頁改成三個場景）。

    python3 animate_group.py <itest 輸出目錄> --frames <目錄> [--fps 10] [--slow 1.0]

真的 cbba_node＋閘道，載具是 ros_bridge 的假身體（不用真的 BT）。讀 events.jsonl、group_plan.json（每段測試的起訖、預期、
任務；itest 的 note：key = 關鍵狀態改變、timer = 計時器〔limit = 期限〕、event = detection）、report.json（各項結果與量到的時間）、
node_*.log 的出價表（候選載具的出價與原因）。版面照使用者給的「CBBA 即時任務與 Agent 監控」（左地圖卡、右任務卡＋相關 Agent）。
25 個項目分在三個場景：

  0:00 標題（速度、yard、3 UAV＋3 Dog＋閘道上線＝G0）
  場景 1 正常協同：G14（含 G14D）→ G1、G3、G4 → G2、G6 → G8 → G15 → G18 → G19 → G20
  場景 2 失聯降級：G5、G7 → G9 → G10 → G11 → G12（G15R 在 G15 那段裡）
  場景 3 政策排程：G16 → G17 → G13 → G21A → G21B
  結果頁（G0～G21 的結果、量到的時間、設定的期限）

原速播放、中間不暫停（2026-11-20 使用者）：Expected 顯示在右下，關鍵狀態發生後那裡改成 PASS／FAIL 證據 4 s；
等待用計時器＋期限表示。全部約等於測試實際的時間（約 5 分鐘）。--slow 可以再放慢。
文字用英文：容器沒有中文字型。畫面最下面 12 % 留白，播放器的進度列不會擋到內容。每一格寫成 frames/%05d.png 與 fps.txt，
itest 用主機的 ffmpeg 合成 group.mp4。
"""
import argparse
import json
import os
import re
import sys
from collections import deque

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402
from matplotlib.ticker import MultipleLocator  # noqa: E402

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

# 三個場景（〈CBBA G0～G21〉第 3～6 頁）：(標題, 說明, [(階段, 段落 id)])；25 個項目都在這三個場景裡
# 三個場景（2026-11-22 使用者：一個場景是一次連續的模擬，在裡面驗證各檢查點；不是每項做完再剪）。
# itest group 每個場景一段（S1～S4），檢查點記在 group_plan.json 的 checkpoints
SCENES = [
    ("Scene 1  Intrusion + fire: full closed loop", "one simulation: intrusion team, localize, revise, done; fire, "
     "approval, verify, report", [("", "S1")]),
    ("Scene 2  Resource shortage, lost agents, degraded recovery", "one simulation: UAVs grounded, 4-dog event, "
     "fallback, takeover, last dog lost, cascade cancel", [("", "S2")]),
    ("Scene 3  Zone policy, coordinated scheduling, priority preemption", "one simulation: no-dog zone, coordinated "
     "task while patrolling, P0 preemption, preempt margin, P0 watchdog", [("", "S3")]),
    # 2026-11-23 L5＋L6 C 批：2-1 階段 1 的案例 3、5、9（偵測 → event_manager）
    ("Scene 4  Event pipeline: confidence, priority, alarm storm", "one simulation: medium confidence -> one dog, "
     "fire and left object at once, alarm storm -> one area patrol", [("", "S4")]),
]
SCENE_CODES = {"S1": ["G0", "G14", "G14D", "G8", "G4", "E6", "G1", "G3", "G2", "G6", "EP1", "G20", "G18", "G15", "G15R",
                      "G19"],
               "S2": ["G12", "G5", "G7", "G9", "EP2", "G10", "G11"], "S3": ["G16", "G17", "G13", "G21A", "G21B"],
               "S4": ["E3", "E5", "E9"]}
TITLE_SEC = 8.0
SCENE_CARD_SEC = 0.0
RESULT_SEC = 10.0
NAMES = {
    "G0": "Nodes + gateway online", "G1": "Precedence", "G2": "Incident revision", "G3": "Sync point",
    "G4": "Distinct agents", "G5": "All-or-nothing", "G6": "TaskSpec partial revision", "G7": "Fallback rebuild",
    "G8": "Heterogeneous roles", "G9": "Holder lost: takeover", "G10": "Takeover timeout: fallback",
    "G11": "Cascade cancel", "G12": "Optional skipped", "G13": "Priority preemption",
    "G14": "Dog detects person -> intrusion", "G14D": "Detection dedup", "G15": "UAV detects fire -> dog",
    "G15R": "Repair after rejoin", "G16": "Zone: dogs kept out", "G20": "Bad TaskSpec rejected",
    "G17": "Coop bid: patrol while waiting", "G18": "Approval before inject", "G19": "Report + evidence",
    "G21A": "Preempt margin", "G21B": "P0 watchdog",
    "E6": "Small move: no re-plan", "EP1": "Incident progress (valid version)", "EP2": "Takeover resets progress",
    "E3": "Medium confidence: one dog", "E5": "Two incidents: P0 first", "E9": "Alarm storm -> area patrol",
}
# 2026-11-20 使用者：用原速、中間不要有任何暫停。Expected、關鍵狀態都不凍結畫面（關鍵狀態在右下顯示 KEY_SHOW 秒）、
# 場景之間不插卡片（場景名稱在地圖卡標題）；只有片頭標題、片尾結果頁
EXPECT_SEC = 0.0      # 每段開頭停幾秒（0 = 不停）
KEY_SEC = 0.0         # 關鍵狀態凍結幾秒（0 = 不停）
KEY_SHOW = 4.0        # 關鍵狀態的 PASS／FAIL 證據在右下顯示多久（不停畫面）
MOVE_SPEED = 1.0      # 有載具在動時的倍速
IDLE_SPEED = 1.0      # 都沒在動時的倍速
SLOW = 1.0            # 全部時間的倍數（--slow；覺得太快就給 1.5）
# 出價表的原因 → 畫面上的字（cbba_core 的 bidFor）
WHY = {"in_bundle": "WINNER", "bid": "LOWER_BID", "locked": "LOCKED", "inactive": "MANUAL/OFF", "platform": "PLATFORM",
       "capability": "NO_CAPABILITY", "distinct": "DISTINCT", "zone_forbidden": "ZONE_FORBIDDEN",
       "infeasible": "INFEASIBLE", "unsupported_action": "UNSUPPORTED"}
ZONES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "site-config", "yard", "zones.geojson")
BOTTOM = 0.035        # 畫面最下面留白（播放器的進度列）；2026-11-20 使用者：0.12 太空，畫面擠


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


def load_report(out):
    """report.json → {項目: (結果, 量到的時間 s 或 None)}"""
    try:
        with open(os.path.join(out, "report.json"), encoding="utf-8") as f:
            rows = json.load(f)
    except (OSError, ValueError):
        return {}
    return {r["code"]: (r["status"], (r.get("metrics") or {}).get("latency")) for r in rows}


def task_name(tid):
    """cbba_node log 裡的任務名稱（建立者.世代.流水號，十六進位）"""
    return f"{(tid >> 48) & 0xFFFF:04X}.{(tid >> 32) & 0xFFFF:04X}.{tid & 0xFFFFFFFF:08X}"


def load_bids(out):
    """node_*.log 的出價表 → {任務名稱: [(時刻, 機號, 出價, 原因, 得標者)]}（照時間）"""
    pat = re.compile(r"\[(\d+\.\d+)\].*?出價表 task=(\S+) .*?agent=(\d+) bid=(\S+) why=(\S+)(?: winner=(\d+))?")
    bids = {}
    for name in sorted(os.listdir(out)):
        if not (name.startswith("node_") and name.endswith(".log")):
            continue
        with open(os.path.join(out, name), encoding="utf-8", errors="replace") as f:
            for line in f:
                m = pat.search(line)
                if m:
                    bids.setdefault(m.group(2), []).append((float(m.group(1)), int(m.group(3)), float(m.group(4)),
                                                            m.group(5), int(m.group(6) or 0)))
    for rows in bids.values():
        rows.sort()
    return bids


def load_zones(path=ZONES):
    """[(id, 不能進的平台 set〔quadruped、uav〕, 多邊形)]；讀不到時空清單"""
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return []
    out = []
    for feat in doc.get("features", []):
        prop = feat.get("properties", {})
        ring = feat.get("geometry", {}).get("coordinates", [[]])[0]
        out.append((prop.get("id", ""), set(prop.get("no_entry", [])), [(p[0], p[1]) for p in ring]))
    return out


def yard_limits(zones):
    """整個 yard（Z_YARD）外擴 2 m；沒有 → None"""
    yard = next((z[2] for z in zones if z[0] == "Z_YARD"), None)
    if not yard:
        return None
    return (min(p[0] for p in yard) - 2, max(p[0] for p in yard) + 2, min(p[1] for p in yard) - 2,
            max(p[1] for p in yard) + 2)


def in_polygon(x, y, ring):
    inside = False
    for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


# 禁入區的顏色與圖例（2026-11-20 使用者：不是只有狗，無人機也有禁飛區）
ZONE_STYLE = {"quadruped": (FAIL_RED, "No-dog zone"), "uav": ("#7c3aed", "No-fly zone")}


def zone_style(no_entry):
    if {"quadruped", "uav"} <= no_entry:
        return "#7f1d1d", "No-entry zone"
    return ZONE_STYLE.get(next(iter(sorted(no_entry)), ""), (GREY, "Restricted zone"))


def zones_for(zones, tasks, sid):
    """這段有遇到的禁區（2026-11-20 使用者：不用一直顯示，有遇到禁區的模擬才畫）：這段的任務點落在裡面，而且任務的平台
    被這個區域禁止、或任務不限平台（例如 G16 的 observer：狗被擋、無人機接）。無人機角色在禁狗區裡不算遇到"""
    name = {1: "uav", 2: "quadruped"}
    pts = [(f["x"], f["y"], f.get("platform", 0)) for f in tasks.values() if f.get("segment") == sid]
    return [z for z in zones if z[1] and any(in_polygon(x, y, z[2]) and (p == 0 or name.get(p) in z[1])
                                             for x, y, p in pts)]


class State:
    """依時間重播事件：位置、手上的任務、gate、斷線、mode、任務結束、偵測、計時器"""

    def __init__(self, plan):
        self.vehicles = {int(k): v for k, v in plan["vehicles"].items()}
        self.pos, self.trail, self.cur, self.gates, self.mode, self.closed = {}, {}, {}, {}, {}, {}
        self.offline, self.arrived = set(), set()
        self.detections = {}     # 偵測編號 -> {t, vid, x, y, label, path, status, t_status}（2026-11-22：可移動、可標誤報）
        self.timer = None        # (開始時刻, 名稱, 期限)
        self.key = None          # 最近的關鍵狀態 (時刻, 證據, 段落)

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
                d = e.get("det") or f"{e.get('vehicle')}:{e['x']}:{e['y']}"
                self.detections[d] = {"t": e["t"], "vid": e.get("vehicle"), "x": e["x"], "y": e["y"],
                                      "label": e.get("label", ""), "path": [(e["x"], e["y"])], "status": "",
                                      "t_status": None}
            elif e.get("event") == "det_move" and e.get("det") in self.detections:
                d = self.detections[e["det"]]
                d["x"], d["y"] = e["x"], e["y"]
                d["path"].append((e["x"], e["y"]))
            elif e.get("event") == "det_status" and e.get("det") in self.detections:
                self.detections[e["det"]].update(status=e.get("status", ""), t_status=e["t"])
            if e.get("timer"):
                self.timer = (e["t"], e["timer"], e.get("limit", ""))
            if e.get("key"):
                self.key = (e["t"], e["text"], e["key"])

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


def moving_times(events):
    """有載具在動的時刻（位置和前一筆差 > 5 cm），排好的 list"""
    last, out = {}, []
    for e in events:
        if e["kind"] != "pos":
            continue
        v, p = e.get("uav"), (e["x"], e["y"])
        q = last.get(v)
        if q is not None and abs(p[0] - q[0]) + abs(p[1] - q[1]) > 0.05:
            out.append(e["t"])
        last[v] = p
    return out


def timeline(plan, events, fps, slow=SLOW):
    """影片的每一格：(場景, 階段, 段落 id 或 None, 模擬時刻, 疊加資訊)。段落照三個場景重排；
    倍速看這個時刻有沒有載具在動（MOVE_SPEED／IDLE_SPEED）"""
    import bisect
    moves = moving_times(events)

    def moving(t):
        i = bisect.bisect_left(moves, t - 0.6)
        return i < len(moves) and moves[i] <= t + 0.6

    segs = segments_by_id(plan)
    frames = [(None, "", None, None, {"kind": "title"})] * int(TITLE_SEC * slow * fps)
    for title, note, stages in SCENES:
        present = [(stage, segs[i]) for stage, i in stages if i in segs and segs[i].get("t1")]
        if not present:
            continue
        frames += [(title, "", None, None, {"kind": "scene", "note": note,
                                            "ids": [i for _, i in stages]})] * int(SCENE_CARD_SEC * slow * fps)
        for stage, s in present:
            ks = keys_of(events, s)
            frames += [(title, stage, s["id"], s["t0"], {"kind": "expect", "speed": 1.0})] * int(EXPECT_SEC * slow * fps)
            t = s["t0"]
            ki = 0
            while t <= s["t1"]:
                speed = MOVE_SPEED if moving(t) else IDLE_SPEED
                while ki < len(ks) and ks[ki]["t"] <= t:
                    frames += [(title, stage, s["id"], ks[ki]["t"], {"kind": "key", "text": ks[ki]["text"],
                                                                      "speed": speed})] * int(KEY_SEC * slow * fps)
                    ki += 1
                frames.append((title, stage, s["id"], t, {"kind": "play", "speed": speed}))
                t += speed / (fps * slow)
            for k in ks[ki:]:
                frames += [(title, stage, s["id"], k["t"], {"kind": "key", "text": k["text"], "speed": 1.0})] * \
                    int(KEY_SEC * slow * fps)
    frames += [(None, "", None, None, {"kind": "results"})] * int(RESULT_SEC * slow * fps)
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


def draw_card(fig, plan, report, scene, info):
    ax = fig.add_axes([0, BOTTOM, 1, 1 - BOTTOM])    # 底部留白：不被播放器的進度列擋住
    ax.axis("off")
    ax.set_xlim(0, 1)     # 固定：圖例的圖示會把自動範圍拉走，文字跑到畫面外（2026-11-20）
    ax.set_ylim(0, 1)
    ax.set_autoscale_on(False)
    if info["kind"] == "title":
        ax.text(0.5, 0.80, "Team CBBA integration test  (G0 - G21)", ha="center", fontsize=26, fontweight="bold")
        nu = sum(v["platform"] == "uav" for v in plan["vehicles"].values())
        nd = sum(v["platform"] == "dog" for v in plan["vehicles"].values())
        g0 = report.get("G0", (plan.get("results", {}).get("G0", ""), None))[0]
        ax.text(0.5, 0.68, f"{nu} UAV + {nd} Dog + Gateway online" + (f"  (G0 {g0})" if g0 else "") +
                "   |   real cbba_node + gateway, simulated bodies (no BT)", ha="center", fontsize=14)
        # 速度照這次測試的假身體（plan 的 vehicles；2026-11-19 起 itest group 是 UAV 5、dog 3 m/s）
        speed = {p: next((v["speed"] for v in plan["vehicles"].values() if v["platform"] == p), None) for p in ("uav", "dog")}
        fmt = lambda v: "?" if v is None else f"{v:g}"
        ax.text(0.5, 0.60, f"Parameters: w_E 25, half-life 60/120/300/600 s, UAV {fmt(speed['uav'])} m/s, "
                f"dog {fmt(speed['dog'])} m/s, yard map + zones, registry + TaskSpec check, no packet loss",
                ha="center", fontsize=12, color="#555555")
        ax.text(0.5, 0.50, "Scene 1  normal coordination      Scene 2  degraded fleet      Scene 3  policy & priority",
                ha="center", fontsize=13)
        ax.text(0.5, 0.43, "Real-time playback   |   task points random in the yard" +
                (f" (seed {plan['seed']})" if plan.get("seed") is not None else ""), ha="center", fontsize=12,
                color="#555555")
        legend(ax, 0.10, 0.28)
        return
    if info["kind"] == "scene":
        ax.text(0.5, 0.70, scene, ha="center", fontsize=24, fontweight="bold")
        ax.text(0.5, 0.58, info["note"], ha="center", fontsize=14, color="#555555")
        ax.text(0.5, 0.46, "covers  " + "  ".join(dict.fromkeys(info["ids"])), ha="center", fontsize=14,
                family="monospace")
        return
    ax.text(0.5, 0.93, "Results  G0 - G21, events", ha="center", fontsize=24, fontweight="bold")
    res = {c: v[0] for c, v in report.items()} or plan.get("results", {})
    codes = ["G0", "G1", "G2", "G3", "G4", "G5", "G6", "G7", "G8", "G9", "G10", "G11", "G12", "G13", "G14", "G14D",
             "G15", "G15R", "G16", "G17", "G18", "G19", "G20", "G21A", "G21B", "E3", "E5", "E6", "E9", "EP1", "EP2"]
    for i, c in enumerate(codes):
        col, row = i // 11, i % 11
        st = res.get(c, "not run")
        lat = report.get(c, (None, None))[1]
        color = PASS_GREEN if st == "PASS" else FAIL_RED if st == "FAIL" else "#777777"
        x, y = 0.02 + col * 0.33, 0.82 - row * 0.058
        ax.text(x, y, c, fontsize=11, fontweight="bold", family="monospace")
        ax.text(x + 0.045, y, NAMES.get(c, ""), fontsize=9)
        ax.text(x + 0.215, y, st, fontsize=11, color=color, fontweight="bold")
        if lat is not None:
            ax.text(x + 0.255, y, f"{lat:.1f} s", fontsize=9, color="#555555")
    n_pass = sum(v == "PASS" for v in res.values())
    ax.text(0.5, 0.17, "Configured limits: takeover P0 10 s / P1 15 s / P2-P3 30 s,  group recovery 10 s,  "
            "P0 watchdog 10 s,  group commit 30 s,  assign_hold 0.4 s,  lost 1.5 s", ha="center", fontsize=11,
            color="#555555")
    ax.text(0.5, 0.10, f"{n_pass} / {len(res)} PASS   (items not run are not marked PASS; times = measured latency)",
            ha="center", fontsize=13)


def task_at(f, t):
    """任務在時刻 t 的位置（itest 記的 moves：注入、改版各一筆）；沒有歷史就用最後的位置"""
    moves = f.get("moves")
    if not moves:
        return f
    x, y = moves[0][1], moves[0][2]
    for tm, mx, my in moves:
        if tm <= t:
            x, y = mx, my
    return dict(f, x=x, y=y)


def focus_task(shown, st):
    """候選表要看哪個任務：這段還開著、最晚注入的；都結束了就最後一個"""
    open_ = [(f["t"], tid) for tid, f in shown if tid not in st.closed]
    pool = open_ or [(f["t"], tid) for tid, f in shown]
    return max(pool)[1] if pool else None


# ---------- 監控畫面（2026-11-20，使用者給的「CBBA 即時任務與 Agent 監控」版面）----------
NAVY = "#17375e"
PAGE = "#eef2f7"
CARD_EDGE = "#d5dde8"
BLUE = "#2563eb"
ORANGE = "#e07b00"
GREY = "#6b7280"
STATUS_COLOR = {"Executing": ORANGE, "En route": BLUE, "Waiting": "#d69e00", "Blocked": FAIL_RED, "Outbid": GREY,
                "Busy": GREY, "Manual": FAIL_RED, "Offline": FAIL_RED, "Idle": "#9ca3af", "Winner": PASS_GREEN}
BLOCKING = {"platform", "capability", "distinct", "zone_forbidden", "infeasible", "unsupported_action", "inactive"}
FAKE_BATTERY = 90.0      # 假身體的 robot_state 固定送 90 %（config.json 的 messages.robot_state）


def box(ax, x, y, w, h, fc="white", ec=CARD_EDGE, lw=1.0, r=0.012, z=0):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}", fc=fc, ec=ec, lw=lw,
                                transform=ax.transAxes, zorder=z, clip_on=False))


def header(fig, subtitle="", right="", speed=""):
    """深藍標題列：標題、副標題（場景、階段）、倍速、LIVE"""
    fig.patch.set_facecolor(PAGE)
    top = fig.add_axes([0, 0.915, 1, 0.085])
    top.axis("off")
    top.add_patch(plt.Rectangle((0, 0), 1, 1, color=NAVY, transform=top.transAxes))
    top.text(0.015, 0.5, "CBBA Live Task & Agent Monitor", color="white", fontsize=17, va="center")
    top.text(0.29, 0.5, subtitle, color="#c7d3e6", fontsize=10.5, va="center")
    if speed:
        top.text(0.80, 0.5, speed, color="#c7d3e6", fontsize=12, va="center", ha="right")
    if right:
        top.scatter([0.825], [0.5], s=70, color="#22c55e", transform=top.transAxes)
        top.text(0.835, 0.5, right, color="white", fontsize=11, va="center")


# 標籤的位置（2026-11-22 使用者：標籤跟著圖示走就好，不要兩個靠近時又換位置）：每個東西第一次出現時挑一次，
# 之後一直用同一個偏移；換場景時清掉
LABEL_OFFSETS = {}


class Labels:
    """地圖標籤避開彼此（2026-11-20：狗靠在一起、角色旗子擠在一起時字會疊）：依序試幾個位置，選第一個不和已放的字、
    圖示重疊的；都會重疊就用重疊最少的。字的大小用字數估（不用 renderer，比較快）"""
    OFFSETS = [(9, -3, "left", "center"), (0, -15, "center", "center"), (0, 13, "center", "center"),
               (-9, -3, "right", "center"), (9, 11, "left", "center"), (9, -17, "left", "center"),
               (-9, 11, "right", "center"), (-9, -17, "right", "center"), (0, -27, "center", "center"),
               (0, 25, "center", "center")]

    def __init__(self, ax, dpi):
        self.ax, self.k, self.boxes = ax, dpi / 72.0, []
        self.area = ax.bbox.extents        # 地圖框（顯示座標）：標籤不要跑出去

    def block(self, x, y, r=10):
        """圖示（資料座標）佔的地方"""
        px, py = self.ax.transData.transform((x, y))
        self.boxes.append((px - r, py - r, px + r, py + r))

    def block_display(self, box):
        self.boxes.append(box)

    def _box(self, px, py, dx, dy, ha, text, size):
        w, h = len(text) * size * 0.62 * self.k, size * 1.35 * self.k
        x0 = px + dx * self.k - (w if ha == "right" else w / 2 if ha == "center" else 0)
        y0 = py + dy * self.k - h / 2
        return (x0, y0, x0 + w, y0 + h)

    @staticmethod
    def _over(a, b):
        return max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(0.0, min(a[3], b[3]) - max(a[1], b[1]))

    def put(self, x, y, text, size=8.5, own=10, key=None, **kw):
        px, py = self.ax.transData.transform((x, y))
        mine = (px - own, py - own, px + own, py + own)
        if key is not None and key in LABEL_OFFSETS:
            dx, dy, ha, va = LABEL_OFFSETS[key]
            self.boxes.append(self._box(px, py, dx, dy, ha, text, size))
            self.ax.annotate(text, (x, y), textcoords="offset points", xytext=(dx, dy), ha=ha, va=va, fontsize=size, **kw)
            return
        best = None
        for dx, dy, ha, va in self.OFFSETS:
            b = self._box(px, py, dx, dy, ha, text, size)
            inside = self._over(b, self.area)
            whole = (b[2] - b[0]) * (b[3] - b[1])
            cost = sum(self._over(b, o) for o in self.boxes if o != mine) + 5 * (whole - inside)
            if best is None or cost < best[0]:
                best = (cost, dx, dy, ha, va, b)
            if cost == 0:
                break
        _, dx, dy, ha, va, b = best
        if key is not None:
            LABEL_OFFSETS[key] = (dx, dy, ha, va)
        self.boxes.append(b)
        self.ax.annotate(text, (x, y), textcoords="offset points", xytext=(dx, dy), ha=ha, va=va, fontsize=size, **kw)


def wrap_lines(text, width, lines):
    """固定欄寬換行（matplotlib 的 wrap 照整張圖的寬度，會超出卡片）；超過行數就截斷加 …"""
    import textwrap
    out = textwrap.wrap(text, width) or [""]
    if len(out) > lines:
        out = out[:lines]
        out[-1] = out[-1][:width - 1] + "…"
    return "\n".join(out)


def short_id(tid):
    return f"T-{tid & 0xFFFF:04X}"


# 任務卡的事件名稱（2026-11-20 使用者：要有像「P0 · 入侵事件」的名稱；容器沒有中文字型，用英文）。itest 的事件標記 → 名稱
EVENT_TITLE = {
    "aon": "All-or-nothing team", "approval": "Intrusion check (needs approval)", "approval_no": "False alarm (rejected)",
    "bad_spec": "Invalid TaskSpec", "cascade": "Cascade team", "coop": "Coordinated inspection", "dist": "Front / rear check",
    "hetero": "Mixed-platform team", "optional": "Inspection, optional overwatch", "p0": "Urgent response",
    "patrol": "Routine patrol", "prA": "Routine task A", "prB1": "Same-level task B1", "prB2": "High-value task B2",
    "prec": "Intrusion check", "report": "Inspection report", "sync": "Synchronized inspection",
    "takeover": "Fire response (holder lost)", "timeout": "Fire response (takeover timeout)", "urgent": "Urgent response",
    "zone_dog": "High-voltage area check", "zone_free": "High-voltage area observation",
    "fallback": "Fallback: single dog verify",
}


def incident_title(inc, members=()):
    """事件名稱：itest-coop-41846-3 → Coordinated inspection；偵測來的（det-50-7）看角色：front/rear → Perimeter intrusion、
    ground_fire_verify → Fire response"""
    parts = (inc or "").split("-")
    if parts and parts[0] == "det":
        roles = {f["label"] for _, f in members}
        if "ground_fire_verify" in roles:
            return "Fire response"
        if roles & {"front", "rear"}:
            return "Perimeter intrusion"
        return "Detected event"
    if parts and parts[0] == "itest" and len(parts) > 1:
        tag = parts[1]
        return EVENT_TITLE.get(tag, tag.replace("_", " ").capitalize())
    return inc or "Task"


def agent_view(st, tasks, vid, focus, group, latest, t):
    """相關 Agent 表的一列：(角色, 狀態, 原因／進度, 原因是不是紅字)"""
    v = st.vehicles[vid]
    if vid in st.offline:
        return "-", "Offline", "OFFLINE", True
    cur = st.cur.get(vid, 0)
    if cur in group:
        f = tasks[cur]
        gate = st.gate_text(vid, cur)
        if gate == "WAITING":
            return f["label"], "Waiting", "gate WAITING", False
        if (vid, cur) in st.arrived:
            return f["label"], "Executing", "on site", False
        x, y = st.pos.get(vid, (f["x"], f["y"]))
        eta = ((x - f["x"]) ** 2 + (y - f["y"]) ** 2) ** 0.5 / max(0.1, v.get("speed", 1.0))
        return f["label"], "En route", f"ETA {eta:.0f} s", False
    if st.mode.get(vid, 0) == 1:
        return "-", "Manual", "MANUAL", True
    if cur and cur in tasks:
        return "-", "Busy", f"on {tasks[cur]['label']}", False
    if vid in latest:
        bid, why, winner = latest[vid]
        if why in BLOCKING:
            return "-", "Blocked", WHY.get(why, why.upper()), True
        if why == "in_bundle" and winner in (0, vid):
            return "-", "Winner", f"WINNER {bid:.1f}", False
        if why in ("bid", "in_bundle"):
            return "-", "Outbid", f"LOWER_BID {bid:.1f}", False
        if why == "locked":
            return "-", "Idle", "LOCKED (taken)", False
    return "-", "Idle", "", False


def draw(fig, plan, report, bids, zones, st, tasks, scene, stage, seg, t, info, lim):
    fig.clf()
    online = sum(1 for v in st.vehicles if v not in st.offline) if st else len(plan["vehicles"])
    live = f"LIVE  ·  {online}/{len(plan['vehicles'])} Agents Online"
    fig.patch.set_facecolor(PAGE)     # 2026-11-20 使用者：上面的深藍標題列先拿掉（header() 留著）
    if seg is None:
        draw_card(fig, plan, report, scene, info)
        return
    speed = f"x{info['speed']:.0f}" if info["kind"] == "play" and info["speed"] > 1.05 else ""
    tasks = {tid: task_at(f, t) for tid, f in tasks.items()}     # 改版前畫在原本的位置（2026-11-22）
    result = report.get(seg["id"], (plan.get("results", {}).get(seg["id"], ""), None))[0]

    # ---------- 左：地圖卡 ----------
    card = fig.add_axes([0.012, BOTTOM + 0.005, 0.585, 0.98 - BOTTOM])
    card.axis("off")
    card.set_xlim(0, 1)
    card.set_ylim(0, 1)
    box(card, 0, 0, 1, 1, r=0.015)
    # 卡片標題（原本在深藍標題列）：場景・階段・項目；下一行這段在測什麼；右上倍速
    # 卡片標題：場景；下一行是這個場景的檢查點進度（2026-11-22：一個場景一次連續的模擬）
    card.text(0.03, 0.965, seg["title"], fontsize=12, fontweight="bold", va="center", color="#111827")
    codes = SCENE_CODES.get(seg["id"], [])
    cps = [c for c in plan.get("checkpoints", []) if c.get("scene") == seg["id"] and c["t"] <= t]
    n_ok = sum(c["status"] == "PASS" for c in cps)
    card.text(0.03, 0.925, f"Checkpoints {n_ok}/{len(codes)} passed", fontsize=9, va="center", color=GREY)
    # 檢查點列：通過綠、失敗紅、還沒到灰
    done = {c["code"]: c["status"] for c in cps}
    cx = 0.28
    for code in codes:
        st_ = done.get(code)
        col = PASS_GREEN if st_ == "PASS" else FAIL_RED if st_ == "FAIL" else "#c4cad3"
        card.text(cx, 0.925, code, fontsize=6.8, va="center", color=col, fontweight="bold" if st_ else "normal")
        cx += 0.0095 * len(code) + 0.01
    if speed:
        card.text(0.97, 0.965, speed, fontsize=12, va="center", ha="right", color=GREY)
    ax = fig.add_axes([0.022, BOTTOM + 0.015, 0.571, 0.895 - BOTTOM - 0.015])
    ax.set_facecolor("white")
    for sp in ax.spines.values():
        sp.set_visible(False)
    # 範圍照地圖框的長寬比往外擴，地圖填滿卡片（2026-11-20：原本等比例縮在中間、左右留白）
    W, H = fig.bbox.width, fig.bbox.height
    p_ = ax.get_position()
    want = (p_.width * W) / (p_.height * H)
    x0, x1, y0, y1 = lim
    if (x1 - x0) / (y1 - y0) < want:
        d = (want * (y1 - y0) - (x1 - x0)) / 2
        x0, x1 = x0 - d, x1 + d
    else:
        d = ((x1 - x0) / want - (y1 - y0)) / 2
        y0, y1 = y0 - d, y1 + d
    lim = (x0, x1, y0, y1)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal", adjustable="box")
    # 方格：x、y 同一個間距（2026-11-20 使用者：原本兩軸各自挑刻度，格子變成長短不一的矩形）
    span_ = x1 - x0
    step = next((d for d in (1, 2, 5, 10, 20, 25, 50, 100) if span_ / d <= 12), 100)
    ax.xaxis.set_major_locator(MultipleLocator(step))
    ax.yaxis.set_major_locator(MultipleLocator(step))
    ax.grid(color="#e3e8ef", lw=0.8)
    ax.tick_params(labelbottom=False, labelleft=False, length=0)
    ax.set_autoscale_on(False)
    ax.apply_aspect()            # 先定好座標轉換，標籤才算得出位置
    W, H = fig.bbox.width, fig.bbox.height
    labels = Labels(ax, fig.dpi)
    # 方位（左下）
    ax.annotate("", (0.075, 0.035), (0.025, 0.035), xycoords="axes fraction",
                arrowprops=dict(arrowstyle="-|>", color="#111827", lw=1.2))
    ax.annotate("", (0.025, 0.11), (0.025, 0.035), xycoords="axes fraction",
                arrowprops=dict(arrowstyle="-|>", color="#111827", lw=1.2))
    ax.text(0.08, 0.035, "x east", transform=ax.transAxes, fontsize=7, va="center", color=GREY)
    ax.text(0.025, 0.125, "y north", transform=ax.transAxes, fontsize=7, ha="center", color=GREY)
    p_ = ax.get_position()
    labels.block_display((p_.x0 * W, p_.y0 * H, (p_.x0 + 0.06) * W, (p_.y0 + 0.11) * H))
    # 區域：狗的禁入區半透明紅（名稱不寫在地圖上，免得和標籤疊；圖例有說明）
    for zid, no_entry, ring in zones:
        if zid == "Z_YARD":      # 圍牆：淡淡的虛線，看得出場地範圍
            ax.plot([p[0] for p in ring] + [ring[0][0]], [p[1] for p in ring] + [ring[0][1]], color="#b8c2cf", lw=1,
                    ls=":", zorder=0)
    met = zones_for(zones, tasks, seg["id"])
    styles = {}
    for zid, no_entry, ring in met:
        color, name = zone_style(no_entry)
        styles[name] = color
        xs, ys = [p[0] for p in ring] + [ring[0][0]], [p[1] for p in ring] + [ring[0][1]]
        ax.fill(xs, ys, color=color, alpha=0.10, zorder=0)
        ax.plot(xs, ys, color=color, alpha=0.45, lw=1.2, zorder=0)
    # 圖例（右下）、計時器（右上）佔的地方；有畫禁區時圖例多一格
    lg_w = 0.16 + 0.085 * len(styles)
    lg_rect = [0.585 - lg_w, BOTTOM + 0.025, lg_w, 0.05]
    labels.block_display((lg_rect[0] * W, lg_rect[1] * H, (lg_rect[0] + lg_rect[2]) * W, (lg_rect[1] + lg_rect[3]) * H))
    timer_on = st.timer and seg["t0"] <= st.timer[0] <= t
    if timer_on:
        p = ax.get_position()
        labels.block_display(((p.x1 - 0.2) * W, (p.y1 - 0.08) * H, p.x1 * W, p.y1 * H))
    shown = []
    for tid, f in tasks.items():
        if f.get("segment") != seg["id"] or t < f["t"]:
            continue
        closed = st.closed.get(tid)
        if closed and t - closed[1] > 5.0:
            continue
        shown.append((tid, f))
        color = FLAG_DONE if closed else FAIL_RED
        ax.scatter([f["x"]], [f["y"]], s=760, marker=FLAG, color=color, edgecolors="#222222", linewidths=0.4, zorder=3)
        labels.block(f["x"], f["y"], 13)
    # 偵測：入侵者走過的路畫虛線；誤報（名稱有 ?）灰色，被拒絕後標 3 s 就消失（2026-11-22）
    dets = []
    for key, d in st.detections.items():
        if not (seg["t0"] <= d["t"] <= t):
            continue
        if d["status"] == "rejected" and t - d["t_status"] > 3.0:
            continue
        doubtful = "?" in d["label"] or d["status"] == "rejected"
        color = "#9ca3af" if doubtful else SOURCE
        if len(d["path"]) > 1:
            ax.plot([p[0] for p in d["path"]], [p[1] for p in d["path"]], ":", color=color, lw=2, zorder=3)
            ax.scatter([d["path"][0][0]], [d["path"][0][1]], s=40, color=color, alpha=0.5, zorder=3)
        ax.scatter([d["x"]], [d["y"]], s=320, marker="*", color=color, edgecolors="black", zorder=4)
        if d["status"] == "rejected":
            ax.scatter([d["x"]], [d["y"]], s=420, marker="x", color=FAIL_RED, linewidths=2.5, zorder=5)
        labels.block(d["x"], d["y"], 9)
        dets.append((key, d, color))
    span = lim[1] - lim[0]
    vlabels = []
    for vid, v in st.vehicles.items():
        if vid not in st.pos:
            continue
        x, y = st.pos[vid]
        off, manual = vid in st.offline, st.mode.get(vid, 0) == 1
        color = "black" if v["platform"] == "uav" else DOG_COLOR
        alpha = 0.3 if (off or manual) else 1.0
        if v["platform"] == "uav":
            ax.scatter([x], [y], s=480, marker=QUAD, color=color, alpha=alpha, zorder=5)
        else:
            ax.scatter([x], [y], s=820, marker=DOG_ICON, color=color, alpha=alpha, linewidths=0, zorder=5)
        labels.block(x, y, 15)
        tr = st.trail.get(vid)
        if tr and len(tr) > 1 and not off:
            (t0_, x0, y0), (t1_, x1, y1) = tr[max(0, len(tr) - 4)], tr[-1]
            d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            if d > 0.3 and t - t1_ < 1.0:
                L = span * 0.06
                tip = (x + (x1 - x0) / d * L * 1.6, y + (y1 - y0) / d * L * 1.6)
                ax.annotate("", tip, (x + (x1 - x0) / d * L * 0.5, y + (y1 - y0) / d * L * 0.5),
                            arrowprops=dict(arrowstyle="-|>", color="#dc2626", lw=1.8), zorder=6)
                labels.block((x + tip[0]) / 2, (y + tip[1]) / 2, 8)
        if off:
            ax.scatter([x], [y], s=420, marker="x", color=FAIL_RED, linewidths=2.5, zorder=6)
        tag = v["name"].upper() if v["platform"] == "uav" else v["name"].capitalize()
        tag += "  OFFLINE" if off else "  MANUAL" if manual else ""
        vlabels.append((x, y, tag, dict(color=FAIL_RED if off else color, fontweight="bold" if off else "normal",
                                        zorder=6, key=f"v{vid}")))
    # 標籤：載具 → 任務 → 偵測（重要的先放）
    for x, y, tag, kw in vlabels:
        labels.put(x, y, tag, size=10, own=15, **kw)
    for tid, f in shown:
        closed = st.closed.get(tid)
        labels.put(f["x"], f["y"], f"{f['label']} · P{f['priority']}" + (" (opt)" if f["optional"] else ""), size=9.5,
                   own=13, color="#9ca3af" if closed else "#111827", zorder=3, key=f"t{tid}")
    for key, d, color in dets:
        who = st.vehicles.get(d["vid"], {}).get("name", "?")
        if d["status"] == "rejected":
            text, color = "false alarm - rejected", FAIL_RED
        else:
            text = f"{d['label']} (by {who})" + ("  moved" if len(d["path"]) > 1 else "")
        labels.put(d["x"], d["y"], text, size=9.5, own=10, color=color, fontweight="bold", zorder=4, key=f"d{key}")
    for zid, no_entry, ring in met:
        cx, cy = sum(p[0] for p in ring) / len(ring), sum(p[1] for p in ring) / len(ring)
        labels.put(cx, cy, zid, size=8, own=0, color=zone_style(no_entry)[0], alpha=0.8, zorder=1, key=f"z{zid}")
    # 圖例、計時器
    lg = fig.add_axes(lg_rect)
    lg.axis("off")
    lg.set_xlim(0, 1)
    lg.set_ylim(0, 1)
    box(lg, 0, 0, 1, 1, ec="#cbd5e1", r=0.08)
    u = 0.16 / lg_w          # 一個單位（UAV、Dog、Task 三格共 0.16）換成圖例框內的比例
    lg.scatter([0.12 * u], [0.5], s=110, marker=QUAD, color="black")
    lg.text(0.24 * u, 0.5, "UAV", fontsize=8.5, va="center")
    lg.scatter([0.45 * u], [0.5], s=180, marker=DOG_ICON, color=DOG_COLOR, linewidths=0)
    lg.text(0.56 * u, 0.5, "Dog", fontsize=8.5, va="center")
    lg.scatter([0.76 * u], [0.45], s=140, marker=FLAG, color=FAIL_RED)
    lg.text(0.82 * u, 0.5, "Task", fontsize=8.5, va="center")
    for i, (name, color) in enumerate(styles.items()):
        x = (0.16 + 0.085 * i) / lg_w
        lg.add_patch(plt.Rectangle((x, 0.3), 0.012 / lg_w, 0.4, fc=color, alpha=0.18, ec=color, lw=0.8))
        lg.text(x + 0.017 / lg_w, 0.5, name, fontsize=8, va="center")
    if timer_on:
        ax.text(0.99, 0.99, f"{st.timer[1]}: {t - st.timer[0]:4.1f} s" + (f"\n{st.timer[2]}" if st.timer[2] else ""),
                transform=ax.transAxes, ha="right", va="top", fontsize=11, fontweight="bold", color="#b45309",
                family="monospace", bbox=dict(boxstyle="round,pad=0.4", fc="#fff7ed", ec="#fdba74"), zorder=7)

    # ---------- 右：任務卡 ----------
    side = fig.add_axes([0.61, BOTTOM + 0.005, 0.378, 0.98 - BOTTOM])
    side.axis("off")
    side.set_xlim(0, 1)
    side.set_ylim(0, 1)
    box(side, 0, 0, 1, 1, r=0.015)
    side.text(0.04, 0.955, "Live Task Status", fontsize=15, va="center", color="#111827")
    side.scatter([0.55], [0.955], s=55, color="#22c55e")
    side.text(0.96, 0.955, live, fontsize=9.5, va="center", ha="right", color=GREY)

    ft = focus_task(shown, st)
    latest = {}
    for (bt, agent, bid, why, winner) in (bids.get(task_name(ft), []) if ft is not None else []):
        if bt <= t and agent in st.vehicles:
            latest[agent] = (bid, why, winner)
    holder_of = {tid: [v for v, c in st.cur.items() if c == tid and v not in st.offline] for tid, _ in shown}
    open_ = [tid for tid, _ in shown if tid not in st.closed]
    assigned = [tid for tid in open_ if holder_of[tid]]
    unassignable = []
    for tid in open_:
        if holder_of[tid]:
            continue
        rows = {}
        for (bt, agent, bid, why, winner) in bids.get(task_name(tid), []):
            if bt <= t:
                rows[agent] = (bid, why)
        if rows and all(b <= 0.0 for b, _ in rows.values()):
            unassignable.append(tid)
    for i, (num, label, color) in enumerate(((len(open_), "active tasks", BLUE), (len(assigned), "assigned", PASS_GREEN),
                                             (len(unassignable), "unassignable", FAIL_RED))):
        x = 0.04 + i * 0.31
        box(side, x, 0.82, 0.29, 0.095, fc="#f8fafc")
        side.text(x + 0.145, 0.88, str(num), fontsize=20, ha="center", va="center", color=color)
        side.text(x + 0.145, 0.838, label, fontsize=9, ha="center", va="center", color=GREY)

    # 一個事件（角色組）一張卡
    groups = {}
    for tid, f in shown:
        groups.setdefault(f.get("incident") or str(tid), []).append((tid, f))
    focus_inc = tasks[ft].get("incident") if ft is not None else None
    # P0～P3 四張卡常駐（2026-11-20 使用者）：每個優先級一格，放這段裡那個優先級最相關的事件
    # （還開著的 → 目前在看的 → 最新的）；沒有就畫灰色的空卡
    slots = {0: [], 1: [], 2: [], 3: []}
    for inc, members in groups.items():
        slots[min(3, min(f["priority"] for _, f in members))].append((inc, members))
    y = 0.805
    h = 0.103
    for level in (0, 1, 2, 3):
        y -= h
        cand = sorted(slots[level], key=lambda kv: (all(tid in st.closed for tid, _ in kv[1]), kv[0] != focus_inc,
                                                    -max(f["t"] for _, f in kv[1])))
        if not cand:
            box(side, 0.04, y, 0.92, h - 0.012, fc="#f8fafc", ec=CARD_EDGE)
            side.text(0.07, y + h - 0.032, f"P{level} · no active task", fontsize=10.5, va="center", color="#9ca3af")
            side.text(0.07, y + h - 0.057, "waiting for tasks of this priority", fontsize=7.5, va="center",
                      color="#b6bcc6")
            continue
        inc, members = cand[0]
        sel = inc == focus_inc
        box(side, 0.04, y, 0.92, h - 0.012, fc="#eff6ff" if sel else "white", ec=NAVY if sel else CARD_EDGE,
            lw=1.6 if sel else 1.0)
        prio = min(f["priority"] for _, f in members)
        side.text(0.07, y + h - 0.032, f"P{prio} · {incident_title(inc, members)}", fontsize=10.5, va="center",
                  color="#111827")
        n_ok = sum(1 for tid, _ in members if holder_of[tid] or st.closed.get(tid, (0,))[0] == 1)
        side.text(0.07, y + h - 0.057, f"{short_id(members[0][0])} · {n_ok}/{len(members)} roles assigned",
                  fontsize=7.5, va="center", color=GREY)
        states = []
        cx = 0.07
        for tid, f in members:
            hs = holder_of[tid]
            closed = st.closed.get(tid)
            if closed:
                txt, c = f"{f['label']} " + ("DONE" if closed[0] == 1 else "SKIPPED" if f["optional"] else "CANCELLED"), \
                    (GREY if closed[0] == 1 else FAIL_RED)
                states.append("done" if closed[0] == 1 else "cancel")
            elif hs:
                vid = hs[0]
                g = st.gate_text(vid, tid)
                ex = (vid, tid) in st.arrived
                c = "#d69e00" if g == "WAITING" else PASS_GREEN if ex else BLUE
                txt = f"{st.vehicles[vid]['name']} {f['label']}"
                states.append("wait" if g == "WAITING" else "exec" if ex else "route")
            else:
                txt, c = f"{f['label']} —", GREY
                states.append("open")
            txt = txt[:26]
            w = 0.02 + 0.0152 * len(txt)
            if cx + w > 0.94:
                break
            box(side, cx, y + 0.01, w, 0.028, fc=c, ec="none", r=0.007)
            side.text(cx + w / 2, y + 0.024, txt, fontsize=7.2, ha="center", va="center", color="white")
            cx += w + 0.012
        if all(s in ("done", "cancel") for s in states):
            badge, bc = ("Done", GREY) if "done" in states else ("Cancelled", FAIL_RED)
        elif "exec" in states:
            badge, bc = "Executing", ORANGE
        elif "wait" in states:
            badge, bc = "Waiting", "#d69e00"
        elif "route" in states:
            badge, bc = "En route", BLUE
        else:
            badge, bc = "Unassigned", FAIL_RED
        box(side, 0.74, y + h - 0.046, 0.19, 0.028, fc="white", ec=bc, r=0.011)
        side.text(0.835, y + h - 0.032, badge, fontsize=8, ha="center", va="center", color=bc)

    # ---------- 右：相關 Agent ----------
    # 表格接在任務卡下面，列高撐滿到最下面的提示列（2026-11-20：原本中間空一大塊）
    y -= 0.032
    title = f"{short_id(ft)}  related agents  ({tasks[ft]['label']})" if ft is not None else "Agents"
    side.text(0.04, y, title[:48], fontsize=10.5, va="center", color="#111827")
    y -= 0.034
    for x, h_ in ((0.04, "Agent"), (0.16, "Role"), (0.40, "Status"), (0.59, "Battery"), (0.76, "Reason / progress")):
        side.text(x, y, h_, fontsize=7.5, color=GREY, va="center")
    group = {tid for tid, f in tasks.items() if focus_inc and f.get("incident") == focus_inc}
    row_h = min(0.07, (y - 0.085) / max(1, len(st.vehicles)))
    for vid in sorted(st.vehicles, key=lambda v: (st.vehicles[v]["platform"] != "uav", v)):
        y -= row_h
        side.plot([0.04, 0.96], [y + row_h / 2, y + row_h / 2], color="#eef1f5", lw=0.8)
        v = st.vehicles[vid]
        role, status, reason, red = agent_view(st, tasks, vid, ft, group, latest, t)
        name = v["name"].upper() if v["platform"] == "uav" else v["name"].capitalize()
        side.text(0.04, y, name, fontsize=8.5, va="center", color="#111827" if v["platform"] == "uav" else DOG_COLOR)
        side.text(0.16, y, role[:18], fontsize=7.5, va="center", color="#374151")
        sc = STATUS_COLOR.get(status, GREY)
        side.scatter([0.405], [y], s=26, color=sc)
        side.text(0.425, y, status, fontsize=8, va="center", color=sc)
        side.add_patch(plt.Rectangle((0.59, y - 0.006), 0.095, 0.012, color="#e5e7eb"))
        side.add_patch(plt.Rectangle((0.59, y - 0.006), 0.095 * FAKE_BATTERY / 100, 0.012, color="#16a34a"))
        side.text(0.695, y, f"{FAKE_BATTERY:.0f}%", fontsize=7.5, va="center", color=GREY)
        side.text(0.76, y, reason[:22], fontsize=7.5, va="center", color=FAIL_RED if red else "#374151")

    # ---------- 右下：預期／證據 ----------
    key = info.get("text") if info["kind"] == "key" else \
        (st.key[1] if st.key and st.key[2] == seg["id"] and 0 <= t - st.key[0] < KEY_SHOW else None)
    if key:
        ok = result in ("PASS", "")
        fc, tc, txt = (PASS_GREEN if ok else FAIL_RED), "white", (result + "  " if result else "") + key
    else:
        fc, tc, txt = "#e0ecfb", "#1e3a8a", "Expected: " + seg.get("expected", "")
    # 一行小字（2026-11-20 使用者：照範本只佔一小塊）
    box(side, 0.04, 0.014, 0.92, 0.04, fc=fc, ec="none", r=0.008)
    side.text(0.06, 0.034, wrap_lines(txt, 84, 1), fontsize=7.5, va="center", color=tc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--frames", required=True)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--slow", type=float, default=SLOW, help="全部時間的倍數（1 = 原速；越大越慢）")
    args = ap.parse_args()
    plan, events = load(args.out)
    if not plan["segments"]:
        raise SystemExit("group_plan.json 沒有測試段落")
    for i, s in enumerate(plan["segments"]):
        s.setdefault("t1", plan["segments"][i + 1]["t0"] if i + 1 < len(plan["segments"]) else events[-1]["t"])
    tasks = {int(k): v for k, v in plan["tasks"].items()}
    report, bids, zones = load_report(args.out), load_bids(args.out), load_zones()
    # 2026-11-20 使用者：地圖固定看整個 yard（Z_YARD 的範圍），位置和 Gazebo 一樣（無人機起降板在左下）；
    # 原本每段照任務放大，載具在畫面上的位置每段都不一樣
    # 讀不到 yard 時也不分段放大：用全部段落的範圍，整支影片同一個視角（2026-11-20 使用者：放大地圖影響觀看）
    yard_lim = yard_limits(zones) or bounds(plan, events, [x["id"] for x in plan["segments"]])
    frames = timeline(plan, events, args.fps, args.slow)
    os.makedirs(args.frames, exist_ok=True)
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    segs = segments_by_id(plan)
    lims = {}
    st, cur_seg, ei = None, None, 0
    for k, (scene, stage, sid, t, info) in enumerate(frames):
        if sid is not None and sid != cur_seg:
            # 換段落：從頭重播到這段開始（段落照場景重排，時間會往回跳）；地圖範圍照這一段
            st, ei, cur_seg = State(plan), 0, sid
            LABEL_OFFSETS.clear()
            lims[sid] = yard_lim
        if sid is not None:
            while ei < len(events) and events[ei].get("t", 0.0) <= t:
                st.apply(events[ei])
                ei += 1
        draw(fig, plan, report, bids, zones, st, tasks, scene, stage, segs.get(sid), t, info, lims.get(sid))
        fig.savefig(os.path.join(args.frames, f"{k:05d}.png"))
    with open(os.path.join(args.frames, "fps.txt"), "w") as f:
        f.write(str(args.fps))
    print(f"{len(frames)} 格（{len(frames) / args.fps:.0f} s、{args.fps} fps）→ {args.frames}")


if __name__ == "__main__":
    main()
