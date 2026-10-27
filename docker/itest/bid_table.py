#!/usr/bin/env python3
"""把 cbba_node 的出價 log 整理成表格（2026-10-20，使用者：每個任務、每一台無人機與狗的出價，當參考資料）。

cbba_node（無人機、狗同一支）在這些時刻印一行「出價表 task=… type=… x=… y=… agent=… bid=… why=… winner=… winner_bid=…」：
開著的任務第一次知道、得標者改變、這台的出價狀態（why）改變。飛行中出價隨位置變動不印。

用法（主機，只用標準庫）：
  python3 docker/itest/bid_table.py [docker/itest/out/<時間>_demo]
不給資料夾時用最新的 *_demo。讀資料夾裡的 stack_uavN.log、node_uavN.log（無人機）、dog_cbba_<ns>.log（狗），輸出：
  bids.md   一、初次出價（每個任務 × 每一台，第一次知道那個任務時的出價）＋最後得標者（做這個任務的）
            二、得標過程（每台自己得到、失去任務的時刻，例如被別台出價更高搶走、失敗交回、被接管）
  bids.svg  長條圖：橫軸任務、縱軸出價，每一台一根（表一的初次出價；黑框＋★ = 最後得標）；瀏覽器開
  bids.csv  全部紀錄（時間、任務、名稱、類型、哪一台、出價、狀態、當時的得標者與得標價），用試算表看
任務名稱：demo_plan.json 的點（1.5 m 內）、火點（0.5 m 內），否則用座標。
"""
import argparse
import csv
import json
import math
import re
from html import escape
from pathlib import Path

LINE = re.compile(r"\[(\d+\.\d+)\].*?出價表 task=(\S+) type=(\d+) x=(\S+) y=(\S+) agent=(\d+) bid=(\S+) why=(\S+) "
                  r"winner=(\d+) winner_bid=(\S+)")
TYPE = {1: "空中偵察", 2: "地面處置", 3: "巡檢"}
AIR = 1   # 空中任務只有無人機做得了，其他（地面處置、巡檢）只有狗（types.hpp 的能力檢查）
WHY = {
    "in_bundle": "得標",
    "bid": "有出價、沒贏",
    "infeasible": "不能做",
    "excluded": "失敗後排除中",
    "locked": "別台已接受",
    "suspended": "BT 暫時拒絕",
    "busy": "不接新任務",
    "inactive": "不參與出價",
    "closed": "已結束",
    "unknown": "不認得",
}


# 和 2D 動畫同一組顏色（plot_results.py、animate_demo.py 的 DOG_SHADES；這裡不 import，主機沒有 matplotlib）
UAV_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7"]
DOG_COLORS = ["#e07000", "#9c4f00", "#f4a259"]
INK, INK_2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1", "#fcfcfb"
FONT = "'Noto Sans CJK TC', 'Microsoft JhengHei', 'PingFang TC', sans-serif"


def bar_chart(title, groups, agents, values):
    """分組長條圖（SVG，只用標準庫）：橫軸任務、縱軸出價，每個任務一組、做得了的每一台一根
    （空中任務只畫無人機、地面任務只畫狗，2026-10-20 使用者）。groups = [(任務, 名稱, 類型, [載具…])]。
    values[(任務, 載具)] = (出價, 得標?, 沒出價的原因)；得標的那根加黑框＋★，沒出價的畫 × 並在滑鼠移上去時顯示原因"""
    bar_w, gap, left, right, top, plot_h = 26, 34, 64, 24, 100, 300
    # 每組只畫做得了這種任務的載具（groups 的第 4 項），寬度跟著每組的根數
    width = max(left + right + sum(bar_w * len(g[3]) + gap for g in groups), left + 70 * len(agents) + 40, 680)
    height = top + plot_h + 70
    vmax = max([v[0] for v in values.values()] + [1.0])
    step = next(s for s in (1, 2, 5, 10, 20, 25, 50, 100, 200, 500, 1000) if vmax / s <= 6)
    ymax = step * math.ceil(vmax / step)

    def y(v):
        return top + plot_h * (1 - v / ymax)

    colors = {}
    for a in agents:
        pool = DOG_COLORS if a.startswith("dog") else UAV_COLORS
        colors[a] = pool[sum(1 for b in colors if b.startswith(a[:3])) % len(pool)]

    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" '
         f'font-family="{FONT}">',
         f'<rect width="100%" height="100%" fill="{SURFACE}"/>',
         f'<text x="{left}" y="26" font-size="16" font-weight="bold" fill="{INK}">{escape(title)}</text>']
    lx = left
    for a in agents:   # 圖例
        o.append(f'<rect x="{lx}" y="40" width="12" height="12" fill="{colors[a]}"/>'
                 f'<text x="{lx + 16}" y="50" font-size="12" fill="{INK_2}">{a}</text>')
        lx += 70
    o.append(f'<text x="{left}" y="72" font-size="12" fill="{MUTED}">★ 黑框 = 最後得標的那台　× = 沒出價'
             f'（滑鼠移上去看原因）　數字 = 第一次知道這個任務時的出價</text>')
    v = 0
    while v <= ymax + 1e-9:   # 格線、縱軸刻度
        o.append(f'<line x1="{left}" x2="{width - right}" y1="{y(v):.1f}" y2="{y(v):.1f}" stroke="{GRID}"/>'
                 f'<text x="{left - 8}" y="{y(v) + 4:.1f}" font-size="11" fill="{MUTED}" text-anchor="end">{v:g}</text>')
        v += step
    o.append(f'<text transform="translate(16 {top + plot_h / 2}) rotate(-90)" font-size="12" fill="{INK_2}" '
             f'text-anchor="middle">出價（score）</text>')
    gx = left + gap / 2
    for key, label, sub, members in groups:
        group_w = bar_w * len(members)
        for ai, a in enumerate(members):
            bid, won, why = values.get((key, a), (0.0, False, "這台沒有這個任務的紀錄"))
            x = gx + ai * bar_w
            if bid > 0:
                stroke = f' stroke="{INK}" stroke-width="2"' if won else ""
                o.append(f'<rect x="{x + 2}" y="{y(bid):.1f}" width="{bar_w - 4}" height="{top + plot_h - y(bid):.1f}" '
                         f'fill="{colors[a]}"{stroke}><title>{a}：{bid:.2f}{"（得標）" if won else ""}</title></rect>')
                o.append(f'<text x="{x + bar_w / 2}" y="{y(bid) - 4:.1f}" font-size="9" fill="{INK_2}" '
                         f'text-anchor="middle">{"★" if won else ""}{bid:.1f}</text>')
            else:
                o.append(f'<text x="{x + bar_w / 2}" y="{top + plot_h - 4}" font-size="12" fill="{colors[a]}" '
                         f'text-anchor="middle">×<title>{a}：{escape(why)}</title></text>')
        cx = gx + group_w / 2
        gx += group_w + gap
        o.append(f'<text x="{cx}" y="{top + plot_h + 20}" font-size="13" font-weight="bold" fill="{INK}" '
                 f'text-anchor="middle">{escape(label)}</text>'
                 f'<text x="{cx}" y="{top + plot_h + 37}" font-size="11" fill="{MUTED}" text-anchor="middle">'
                 f'{escape(sub)}</text>')
    o.append(f'<line x1="{left}" x2="{width - right}" y1="{top + plot_h}" y2="{top + plot_h}" stroke="{INK_2}"/>')
    o.append("</svg>")
    return "\n".join(o)


def agent_logs(out):
    """[(載具名稱前綴, log 檔)]：無人機 uavN、狗 dogN（N 是機號，從每一行的 agent= 取）"""
    files = [("uav", p) for p in sorted(out.glob("stack_uav*.log")) + sorted(out.glob("node_uav*.log"))]
    files += [("dog", p) for p in sorted(out.glob("dog_cbba_*.log"))]
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?")
    args = ap.parse_args()
    if args.out:
        out = Path(args.out)
    else:
        runs = sorted(Path(__file__).resolve().parent.glob("out/*_demo"))
        if not runs:
            raise SystemExit("找不到 docker/itest/out/*_demo")
        out = runs[-1]

    rows = []
    for kind, path in agent_logs(out):
        for line in open(path, encoding="utf-8", errors="replace"):
            m = LINE.search(line)
            if not m:
                continue
            t, task, typ, x, y, agent, bid, why, winner, wbid = m.groups()
            rows.append({"t": float(t), "task": task, "type": int(typ), "x": float(x), "y": float(y),
                         "agent": f"{kind}{agent}", "agent_id": int(agent), "bid": float(bid), "why": why,
                         "winner": int(winner), "winner_bid": float(wbid)})
    if not rows:
        raise SystemExit(f"{out} 的 log 裡沒有「出價表」（cbba_node 要是 2026-10-20 之後編譯的）")
    rows.sort(key=lambda r: r["t"])

    plan_path = out / "demo_plan.json"
    plan = json.load(open(plan_path, encoding="utf-8")) if plan_path.exists() else {}
    points = plan.get("points", {})
    fires = [f for f in plan.get("fires", []) if "spawned" in f]
    t0 = plan.get("t0") or rows[0]["t"]

    def name_of(x, y):
        p = min(points.items(), key=lambda kv: math.hypot(kv[1]["x"] - x, kv[1]["y"] - y), default=None)
        p = p[0] if p and math.hypot(p[1]["x"] - x, p[1]["y"] - y) < 1.5 else None
        f = next((f["name"] for f in fires if math.hypot(f["x"] - x, f["y"] - y) <= 0.5), None)
        return f"{p}（火）" if p and f else p or f or f"({x:g}, {y:g})"

    # 機號 → 名稱（winner 欄用）
    who = {r["agent_id"]: r["agent"] for r in rows}
    agents = sorted(who.values(), key=lambda a: (a.startswith("dog"), int(re.sub(r"\D", "", a))))
    tasks = {}
    for r in rows:
        tasks.setdefault(r["task"], {"t": r["t"], "type": r["type"], "name": name_of(r["x"], r["y"])})
    order = sorted(tasks, key=lambda k: tasks[k]["t"])

    def cell(r):
        if r is None:
            return "—"
        if r["why"] in ("in_bundle", "bid"):
            return f"{r['bid']:.2f}" + (" ★" if r["why"] == "in_bundle" else "")
        return WHY.get(r["why"], r["why"])

    def name_id(a):
        return "無" if a == 0 else who.get(a, f"agent{a}")

    # 得標過程：每台自己「得到」（why 變成 in_bundle）、「失去」（從 in_bundle 變成別的）的時刻。
    # 不用 winner 欄：各台看到的得標者有時間差，合起來會出現假的「得標 → 無 → 得標」
    changes = []
    holding = {}
    for r in rows:
        key = (r["agent"], r["task"])
        now_hold = r["why"] == "in_bundle"
        if now_hold != holding.get(key, False):
            changes.append((r, "得到" if now_hold else "失去"))
            holding[key] = now_hold
    final = {}
    for r, what in changes:
        if what == "得到":
            final[r["task"]] = r
        elif final.get(r["task"]) is not None and final[r["task"]]["agent"] == r["agent"]:
            final[r["task"]] = None
    # 任務結束後就不再印（只印開著的），最後一次「得到」而沒有「失去」的就是做這個任務的

    md = [f"# 出價表：{out.name}", "",
          "時間從 demo 開始算（秒）。出價 = score = p·value／(1 + ΔC／c_ref)（公式見 `docs/cbba_cbf_formulas.html`）。",
          "★ = 那一刻這台已得標；文字 = 沒有出價的原因。", "",
          "## 一、初次出價（每一台第一次知道這個任務時；空中任務只列無人機、地面任務只列狗）"]
    for title, is_air in (("空中任務（無人機）", True), ("地面任務（狗）", False)):
        ks = [k for k in order if (tasks[k]["type"] == AIR) == is_air]
        cols = [a for a in agents if a.startswith("uav") == is_air]
        if not ks or not cols:
            continue
        md += ["", f"### {title}", "",
               "| 任務 | 類型 | 出現 (s) | " + " | ".join(cols) + " | 最後得標 | 得標價 |",
               "|---|---|---:|" + "---:|" * len(cols) + "---|---:|"]
        for k in ks:
            first = {a: next((r for r in rows if r["task"] == k and r["agent"] == a), None) for a in cols}
            f = final.get(k)
            md.append(f"| {tasks[k]['name']} `{k}` | {TYPE.get(tasks[k]['type'], tasks[k]['type'])} | "
                      f"{tasks[k]['t'] - t0:.1f} | " + " | ".join(cell(first[a]) for a in cols) +
                      (f" | {f['agent']} | {f['bid']:.2f} |" if f else " | 無 | — |"))

    md += ["", "## 二、得標過程（每台自己得到、失去任務的時刻）", "",
           "| 時間 (s) | 任務 | 載具 | 得到／失去 | 出價 | 失去後的狀態 |", "|---:|---|---|---|---:|---|"]
    prev_bid = {}
    for r, what in changes:
        key = (r["agent"], r["task"])
        bid = r["bid"] if what == "得到" else prev_bid.get(key, 0.0)
        prev_bid[key] = r["bid"]
        md.append(f"| {r['t'] - t0:.1f} | {tasks[r['task']]['name']} | {r['agent']} | {what} | {bid:.2f} | "
                  f"{'' if what == '得到' else cell(r)} |")
    (out / "bids.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    # 長條圖（2026-10-20，使用者：橫軸任務、縱軸每一台的出價）：用表一的初次出價
    values = {}
    for k in order:
        f = final.get(k)
        for a in agents:
            r = next((r for r in rows if r["task"] == k and r["agent"] == a), None)
            if r is None:
                continue
            has = r["why"] in ("in_bundle", "bid")
            values[(k, a)] = (r["bid"] if has else 0.0, bool(f and f["agent"] == a),
                              "" if has else WHY.get(r["why"], r["why"]))
    groups = [(k, tasks[k]["name"], TYPE.get(tasks[k]["type"], str(tasks[k]["type"])),
               [a for a in agents if a.startswith("uav") == (tasks[k]["type"] == AIR)]) for k in order]
    (out / "bids.svg").write_text(bar_chart(f"各台對每個任務的初次出價（{out.name}）", groups, agents, values),
                                  encoding="utf-8")

    with open(out / "bids.csv", "w", newline="", encoding="utf-8-sig") as f:   # utf-8-sig：Excel 開中文不亂碼
        w = csv.writer(f)
        w.writerow(["時間(s)", "任務", "名稱", "類型", "載具", "出價", "狀態", "得標者", "得標價"])
        for r in rows:
            w.writerow([f"{r['t'] - t0:.2f}", r["task"], tasks[r["task"]]["name"], TYPE.get(r["type"], r["type"]),
                        r["agent"], f"{r['bid']:.3f}", WHY.get(r["why"], r["why"]), name_id(r["winner"]),
                        f"{r['winner_bid']:.3f}"])
    print(f"{len(tasks)} 個任務、{len(agents)} 台（{', '.join(agents)}）、{len(rows)} 筆 → {out / 'bids.md'}、bids.svg、bids.csv")


if __name__ == "__main__":
    main()
