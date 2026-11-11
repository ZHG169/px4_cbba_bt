#!/usr/bin/env python3
"""狗的日常巡檢點的進度（2026-11-07）：讀伯宇 BT 的 log（dog_bt_<ns>.log），itest.py、animate_demo.py、label_gazebo.py 共用。

log 的行（牆上時間）：「[INFO] [1794060535.61] [task_manager]: … GoTo │ → navigate_to_pose  去 A2  (…)」、
「… GoTo │ ✓ 抵達 A2  goal=…」。巡邏是一直繞圈（loop），每個點會一再重複做：
  - 做完：抵達這個點之後，下一個「去 …」是**別的巡檢點**（雲台檢查做完、往下一點走）。下一個是 cbba_／manual_ 時不算——
    伯宇的 BT 被插隊後會接回被打斷的那個點，回來才做完
  - 再出現：狗再出發去這個點（「去 A2」）
使用者 2026-11-07：做完的點 2D 動畫變灰、Gazebo 刪掉，下一圈再出現。
"""
import re

LINE = re.compile(r"\[(\d+\.\d+)\] .*(去|✓ 抵達) (\S+)")


def parse(lines):
    """[(牆上時間, "go" | "arrive", 目標名稱)]"""
    out = []
    for row in lines:
        m = LINE.search(row)
        if m:
            out.append((float(m.group(1)), "go" if m.group(2) == "去" else "arrive", m.group(3)))
    return out


def done_intervals(events, names):
    """每個巡檢點做完（變灰）的時段：{名稱: [(做完的時刻, 再出發去它的時刻 或 None)]}"""
    names = set(names)
    out = {n: [] for n in names}
    arrived = None              # 最近抵達、還沒做完的巡檢點
    for t, kind, target in events:
        if kind == "arrive":
            arrived = target if target in names else arrived
            continue
        if target not in names:
            continue            # 去 cbba_／manual_：被插隊，arrived 留著（之後會接回）
        if arrived is not None and target != arrived:
            out[arrived].append([t, None])
        arrived = None if target != arrived else arrived
        if out[target] and out[target][-1][1] is None:
            out[target][-1][1] = t
    return {n: [tuple(iv) for iv in ivs] for n, ivs in out.items()}


def is_done(intervals, t):
    """這個點在 t 時是不是做完（灰色、Gazebo 沒有標記）"""
    return any(a <= t and (b is None or t < b) for a, b in intervals)
