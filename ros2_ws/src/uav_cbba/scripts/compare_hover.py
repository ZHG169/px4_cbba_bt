#!/usr/bin/env python3
"""並排比較「未考慮電量」與「考慮電量」：確認火情後就地懸停監看，電量到返航門檻才返航。

用法：
    cbba_sim run scenarios/fire_hover.csv          --out <考慮電量 run>
    cbba_sim run scenarios/fire_hover_noenergy.csv --out <未考慮電量 run>
    compare_hover.py <未考慮電量 run> <考慮電量 run> scenarios/fire_hover.csv <輸出 GIF>

兩邊的耗電都用真實參數場景檔計算（未考慮電量的那組只是出價時忽略電量）。
返航門檻 = 安全存量 + 從目前位置返航所需電量。
另外輸出 <輸出 GIF 去掉副檔名>_final.png（最後一格）。
需要 matplotlib、Pillow 和中文字型（Noto Sans CJK TC）。
"""
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from animate_results import load_run, read_scenario  # noqa: E402
from plot_results import (FIRE, GRID, INK, INK_2, MUTED, SURFACE, agent_color_map,  # noqa: E402
                          draw_fire_halo, read_task_tags)

for f in font_manager.findSystemFonts(["/usr/share/fonts/opentype/noto"]):
    font_manager.fontManager.addfont(f)
plt.rcParams["font.family"] = ["Noto Sans CJK TC", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

no_dir, yes_dir, scenario, out = sys.argv[1:5]
params = read_scenario(scenario)
tags = read_task_tags(scenario)
FPS, SPEEDUP, TAIL = 12, 3.0, 12.0   # TAIL：所有返航結束後再多播放的模擬秒數（顯示懸停耗電）


def plan(agent, legs, epm, hover):
    """回傳 dict：segs=[(t0,t1,p0,p1,e0,e1)]、mode（monitor/rtl/home）與相關數值。"""
    home = (float(agent["x"]), float(agent["y"]))
    battery, reserve = float(agent["battery"]), float(agent["safety_reserve"])
    pos, t, e, speed = home, 0.0, 0.0, None
    segs, fire_tid = [], None
    for leg in legs:
        nxt = (float(leg["x"]), float(leg["y"]))
        arr, comp = float(leg["arrival"]), float(leg["completion"])
        d = math.dist(pos, nxt)
        if arr > t and d > 0:
            speed = d / (arr - t)
        segs.append((t, arr, pos, nxt, e, e + d * epm))
        e += d * epm
        segs.append((arr, comp, nxt, nxt, e, e + (comp - arr) * hover))
        e += (comp - arr) * hover
        pos, t = nxt, comp
        if tags.get(int(leg["task_id"])) == "fire":
            fire_tid = int(leg["task_id"])
            break
    speed = speed or 4.0
    info = dict(home=home, battery=battery, reserve=reserve, epm=epm, hover=hover, segs=segs,
                fire=fire_tid, t_done=t, end_pos=pos)
    ret = math.dist(pos, home) * epm
    left = battery - e
    if fire_tid is not None and left > reserve + ret:
        info.update(mode="monitor", monitor_s=(left - reserve - ret) / hover)
        return info
    if segs:
        dt = math.dist(pos, home) / speed
        segs.append((t, t + dt, pos, home, e, e + ret))
    info.update(mode="rtl" if fire_tid is not None else "home", final=battery - e - ret)
    return info


def state(info, now):
    """目前位置、已耗電量。"""
    segs = info["segs"]
    if not segs:
        return info["home"], 0.0
    for t0, t1, p0, p1, e0, e1 in segs:
        if now <= t1:
            f = 0.0 if t1 <= t0 else max(0.0, (now - t0) / (t1 - t0))
            return ((p0[0] + (p1[0] - p0[0]) * f, p0[1] + (p1[1] - p0[1]) * f),
                    e0 + (e1 - e0) * f)
    used = segs[-1][5]
    if info["mode"] == "monitor":
        used += (now - segs[-1][1]) * info["hover"]
    return segs[-1][3], used


panels = []
for d, title in ((no_dir, "未考慮電量"), (yes_dir, "考慮電量（本研究）")):
    agents, tasks, legs = load_run(d)
    infos = {int(a["agent_id"]): plan(a, legs.get(int(a["agent_id"]), []),
                                      *params[int(a["agent_id"])]) for a in agents}
    panels.append(dict(title=title, agents=agents, tasks=tasks, legs=legs, infos=infos))

ids = sorted(int(a["agent_id"]) for a in panels[0]["agents"])
colors = agent_color_map(ids)
t_end = max(i["segs"][-1][1] for p in panels for i in p["infos"].values() if i["segs"]) + TAIL
xs = [float(t["x"]) for t in panels[0]["tasks"]] + [float(a["x"]) for a in panels[0]["agents"]]
ys = [float(t["y"]) for t in panels[0]["tasks"]] + [float(a["y"]) for a in panels[0]["agents"]]
pad = 5.0

fig = plt.figure(figsize=(12, 6.8), facecolor=SURFACE)
gs = fig.add_gridspec(2, 2, height_ratios=[2.3, 1], hspace=0.38, wspace=0.18,
                      left=0.06, right=0.97, top=0.87, bottom=0.07)
fig.suptitle("電量成本驗證：確認火情後就地懸停監看", fontsize=15, fontweight="bold", color=INK)
fig.text(0.5, 0.925, "uav1 電量 34%（離火點 T1 近）　uav2 電量 100%　uav3 電量 60%　"
         "返航門檻 = 安全存量 20% + 返航所需電量", ha="center", fontsize=10.5, color=INK_2)
clock = fig.text(0.97, 0.965, "", ha="right", fontsize=11, color=INK, family="monospace")

for col, p in enumerate(panels):
    ax = fig.add_subplot(gs[0, col])
    axb = fig.add_subplot(gs[1, col])
    p["ax"], p["axb"] = ax, axb
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad - 4, max(ys) + pad)
    ax.set_title(p["title"], fontsize=13, fontweight="bold", color=INK)
    ax.set_xlabel("x (m)", fontsize=9)
    ax.tick_params(labelsize=8)
    for aid in ids:
        h = p["infos"][aid]["home"]
        ax.scatter([h[0]], [h[1]], marker="s", s=90, color="none", edgecolors=colors[aid],
                   linewidths=1.4, zorder=2)
        ax.annotate(f"uav{aid}", h, xytext=(13, 2), textcoords="offset points", ha="left",
                    fontsize=9, color=colors[aid], fontweight="bold")
    p["completion"] = {int(r["task_id"]): float(r["completion"])
                       for rows in p["legs"].values() for r in rows}
    p["task_art"], p["halos"] = {}, {}
    for t in p["tasks"]:
        tid, w = int(t["task_id"]), int(t["winner"])
        x, y = float(t["x"]), float(t["y"])
        fire = tags.get(tid) == "fire"
        if fire:
            p["halos"][tid] = draw_fire_halo(ax, x, y)
        p["task_art"][tid] = ax.scatter([x], [y], marker="o", s=170, color=colors.get(w, MUTED),
                                        edgecolors=SURFACE, linewidths=1.8, zorder=4)
        ax.annotate(("火點 " if fire else "") + f"T{tid}", (x, y),
                    xytext=(0, 17) if fire else ((8, -15) if y < -6 else (8, 6)),
                    textcoords="offset points", fontsize=9.5 if fire else 9,
                    ha="center" if fire else "left",
                    color=FIRE if fire else INK, fontweight="bold" if fire else "normal")
    p["trails"] = {aid: ax.plot([], [], color=colors[aid], lw=2.2, zorder=3)[0] for aid in ids}
    p["marks"] = {aid: ax.scatter([p["infos"][aid]["home"][0]], [p["infos"][aid]["home"][1]],
                                  marker="^", s=230, color=colors[aid], edgecolors=INK,
                                  linewidths=1.0, zorder=6) for aid in ids}

    ypos = {aid: i for i, aid in enumerate(reversed(ids))}
    reserve = p["infos"][ids[0]]["reserve"]
    axb.barh(list(ypos.values()), [100] * len(ids), color=GRID, height=0.55, zorder=1)
    axb.axvspan(0, reserve, color=FIRE, alpha=0.08, zorder=0)
    axb.axvline(reserve, color=FIRE, lw=1.2, ls=(0, (4, 2)), zorder=3)
    axb.text(reserve, len(ids) - 0.35, " 安全存量 20%", fontsize=8.5, color=FIRE, va="bottom")
    axb.text(100, len(ids) - 0.35, "▌= 返航門檻", fontsize=8.5, color=INK, va="bottom", ha="right")
    p["bars"] = {aid: axb.barh([ypos[aid]], [p["infos"][aid]["battery"]], color=colors[aid],
                               height=0.55, zorder=2)[0] for aid in ids}
    p["ticks"] = {aid: axb.plot([], [], color=INK, lw=3.0, zorder=4)[0] for aid in ids}
    p["vals"] = {aid: axb.text(0, ypos[aid], "", va="center", fontsize=9, color=INK,
                               fontweight="bold") for aid in ids}
    p["ypos"] = ypos
    axb.set_yticks(list(ypos.values()), [f"uav{aid}" for aid in ypos])
    axb.set_xlim(0, 100)
    axb.set_ylim(-0.6, len(ids) - 0.1)
    axb.set_xlabel("電量（%）", fontsize=9)
    axb.tick_params(labelsize=8.5)
    for s in ("top", "right"):
        axb.spines[s].set_visible(False)

n_frames = int(math.ceil(t_end / SPEEDUP * FPS)) + 1 + 3 * FPS
verdict_art = []


def status(info, now, left):
    last = info["segs"][-1][1] if info["segs"] else 0.0
    if info["mode"] == "monitor" and now >= info["t_done"]:
        remain = info["monitor_s"] - (now - info["t_done"])
        return f"{left:.1f}%  監看 T{info['fire']}，剩 {remain / 60:.1f} 分", False
    if info["mode"] == "rtl" and now >= info["t_done"]:
        if now >= last:
            return f"{left:.1f}%  電量不足無法監看，返航後低於 20%", True
        return f"{left:.1f}%  已達返航門檻，立刻返航", True
    if info["mode"] == "home" and info["segs"] and now >= last:
        return f"{left:.1f}%  已返航", False
    return f"{left:.1f}%", False


def update(frame):
    now = min(t_end, frame * SPEEDUP / FPS)
    clock.set_text(f"t = {now:5.1f} s")
    for p in panels:
        for aid in ids:
            info = p["infos"][aid]
            (x, y), used = state(info, now)
            p["marks"][aid].set_offsets([[x, y]])
            pts = [info["home"]]
            for t0, t1, p0, p1, _, _ in info["segs"]:
                if now >= t1:
                    pts.append(p1)
                elif now > t0:
                    pts.append((x, y))
                    break
            p["trails"][aid].set_data([q[0] for q in pts], [q[1] for q in pts])
            left = info["battery"] - used
            thr = info["reserve"] + math.dist((x, y), info["home"]) * info["epm"]
            text, bad = status(info, now, left)
            bad = bad or (left < thr - 1e-6 and math.dist((x, y), info["home"]) > 1e-6)
            p["bars"][aid].set_width(left)
            p["bars"][aid].set_color(FIRE if bad else colors[aid])
            yy = p["ypos"][aid]
            p["ticks"][aid].set_data([thr, thr], [yy - 0.36, yy + 0.36])
            p["vals"][aid].set_text(text)
            inside = max(left, thr) > 55
            p["vals"][aid].set_x(left - 1.5 if inside else max(left, thr) + 1.5)
            p["vals"][aid].set_ha("right" if inside else "left")
            p["vals"][aid].set_color(SURFACE if inside else (FIRE if bad else INK))
        for tid, art in p["task_art"].items():
            if now >= p["completion"].get(tid, math.inf):
                art.set_alpha(0.35)
        for tid, halo in p["halos"].items():
            halo.set_sizes([520 + 260 * (0.5 + 0.5 * math.sin(frame * 0.9))])
    if now >= t_end and not verdict_art:
        for p in panels:
            rtl = [aid for aid in ids if p["infos"][aid]["mode"] == "rtl"]
            mon = [aid for aid in ids if p["infos"][aid]["mode"] == "monitor"]
            if rtl:
                aid = rtl[0]
                txt = (f"✘ uav{aid} 到火點 T{p['infos'][aid]['fire']} 就沒電監看，"
                       f"返航剩 {p['infos'][aid]['final']:.1f}%")
            else:
                txt = "✔ 兩個火點都有人監看（可監看 " + "、".join(
                    f"uav{aid} {p['infos'][aid]['monitor_s'] / 60:.1f} 分" for aid in mon) + "）"
            verdict_art.append(p["ax"].text(0.5, 0.04, txt, transform=p["ax"].transAxes,
                                            ha="center", fontsize=11, fontweight="bold",
                                            color=FIRE if rtl else INK,
                                            bbox=dict(facecolor=SURFACE, edgecolor=GRID,
                                                      pad=4)))
    return []


anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
anim.save(out, writer=PillowWriter(fps=FPS), dpi=90)
fig.savefig(os.path.splitext(out)[0] + "_final.png", dpi=150, facecolor=SURFACE)
for p in panels:
    print(p["title"])
    for aid in ids:
        i = p["infos"][aid]
        print(f"  uav{aid} mode={i['mode']} fire={i['fire']}",
              f"monitor={i.get('monitor_s', 0):.0f}s" if i["mode"] == "monitor"
              else f"final={i.get('final', 0):.1f}%")
print("wrote", out)
