#!/usr/bin/env python3
"""畫 itest loss（第 5 週弱網量測）的結果：掉包率對 5 s 收斂率、收斂時間、完成 → 全隊確認、mesh 頻寬
（四張圖、各自一個 y 軸）。

  python3 docker/itest/plot_loss.py docker/itest/out/<時間>_loss     → 同目錄的 loss_sweep.png
  python3 docker/itest/plot_loss.py <目錄 A> <目錄 B> <目錄 C>         → 第一個目錄的 loss_sweep.png、loss_params.png

多個目錄時：cbba_node 參數（itest 的 --node-param，csv 的 node_params）一樣的合併，不一樣的各畫一條線
（例如比較 resend_gap、completion_retry；最多 4 組），另外畫 loss_params.png（各組都有的掉包率上，
完成 → 全隊確認、mesh 頻寬的長條圖）。只有一組時收斂時間、確認時間畫中位數＋P90。

要 matplotlib（主機沒有時在 sim 容器裡跑：./enter.sh sim，路徑換成 /home/ncrl/CBBA_BT/...）。
有中文字型（Noto Sans/Serif CJK 等）時圖上用中文，沒有時（sim 容器）自動改用英文標籤。
讀 trials.csv（每次試驗一列）重算，不用 levels.csv，方便合併多次量測的結果（多個目錄一起給）。
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# 參考調色盤（dataviz 的 palette.md，已驗證色盲可分辨）：中位數 = 系列 1，P90 = 系列 2
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"

LABELS = {
    "zh": dict(loss="掉包率（%）", rate="5 s 內收斂率（沒有重複認領）", conv="收斂時間（建立 → 每個任務恰好一台）",
               close="完成 → 全隊確認（中位數）", close1="完成 → 全隊確認", default="預設參數",
               bar_close="（長條 = 中位數、誤差線到 P90）",
               bw="mesh 頻寬（每台，收＋送）", sec="秒", target="目標 95%（30% 掉包）", n="每個掉包率 {k} 次",
               median="中位數"),
    "en": dict(loss="packet loss (%)", rate="converged within 5 s (no duplicate claims)",
               close="completion -> confirmed by all (median)", close1="completion -> confirmed by all",
               default="default params", bar_close=" (bar = median, whisker to P90)",
               conv="convergence time (created -> one holder per task)", bw="mesh bandwidth per vehicle (rx+tx)",
               sec="s", target="target 95% (30% loss)", n="n = {k} per level", median="median"),
}


def pick_font():
    """有中文字型就用（回傳 "zh"），沒有就用英文標籤"""
    from matplotlib import font_manager
    for f in font_manager.fontManager.ttflist:
        if "CJK" in f.name or f.name in ("WenQuanYi Zen Hei", "AR PL UMing TW"):
            plt.rcParams["font.family"] = f.name
            return "zh"
    return "en"


def load(dirs):
    """{node_params: {掉包率: [試驗]}}"""
    groups = {}
    for d in dirs:
        with open(os.path.join(d, "trials.csv"), encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        # 開始時不乾淨、收不了尾的試驗不計（舊的 trials.csv 沒有這兩欄時全部計入）
        for r in rows:
            if r.get("clean", "1") == "1" and r.get("leftover", "0") in ("0", ""):
                g = groups.setdefault(r.get("node_params", ""), {})
                g.setdefault(int(r["loss"]), []).append(r)
    return {k: dict(sorted(v.items())) for k, v in groups.items()}


def quantile(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))] if xs else None


def bandwidth(dirs):
    """levels.csv 的 mesh_kBps：{node_params: {掉包率: 平均}}"""
    out = {}
    for d in dirs:
        path = os.path.join(d, "levels.csv")
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as f:
            for r in csv.DictReader(f):
                if r["mesh_kBps"]:
                    out.setdefault(r.get("node_params", ""), {}).setdefault(int(r["loss"]), []).append(
                        float(r["mesh_kBps"]))
    return {g: {k: sum(v) / len(v) for k, v in lv.items()} for g, lv in out.items()}


def style(ax, title, ylabel, xlabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=11, loc="left")
    ax.set_xlabel(xlabel, color=MUTED)
    ax.set_ylabel(ylabel, color=MUTED)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def stats(levels, key):
    """每個掉包率：(中位數, P90)；key 是 trials.csv 的欄位（秒）"""
    out = {}
    for l, rows in levels.items():
        xs = [float(r[key]) for r in rows if r.get(key)]
        out[l] = (quantile(xs, 0.5), quantile(xs, 0.9))
    return out


def line(ax, pts, color, label=None, style_="-"):
    pts = [(x, y) for x, y in pts if y is not None]
    if pts:
        ax.plot(*zip(*pts), color=color, linewidth=2, marker="o", markersize=8, label=label, linestyle=style_)


def bars(groups, bw, L, name, out):
    """多組參數時另畫長條圖（loss_params.png）：各組都有的掉包率上，完成 → 全隊確認（長條 = 中位數、誤差線到 P90）
    和 mesh 頻寬並排比較"""
    common = sorted(set.intersection(*(set(lv) for lv in groups.values())))
    if not common:
        return
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), facecolor=SURFACE)
    style(axes[0], L["close1"] + L["bar_close"], L["sec"], L["loss"])
    style(axes[1], L["bw"], "kB/s", L["loss"])
    k = len(groups)
    width = 0.8 / k
    for gi, (g, lv) in enumerate(groups.items()):
        pos = [i - 0.4 + width * (gi + 0.5) for i in range(len(common))]
        st = stats(lv, "close_s")
        med = [st[l][0] or 0.0 for l in common]
        p90 = [max(0.0, (st[l][1] or 0.0) - (st[l][0] or 0.0)) for l in common]
        axes[0].bar(pos, med, width, color=SERIES[gi], edgecolor=SURFACE, linewidth=2, label=name(g))
        axes[0].errorbar(pos, med, yerr=[[0] * len(common), p90], fmt="none", ecolor=INK, elinewidth=1, capsize=3)
        b = [bw.get(g, {}).get(l, 0.0) for l in common]
        axes[1].bar(pos, b, width, color=SERIES[gi], edgecolor=SURFACE, linewidth=2, label=name(g))
        for x, y in zip(pos, b):
            axes[1].annotate(f"{y:.1f}", (x, y), textcoords="offset points", xytext=(0, 3), ha="center",
                             color=INK, fontsize=8)
    for ax in axes:
        ax.set_xticks(range(len(common)))
        ax.set_xticklabels([str(l) for l in common])
        ax.set_ylim(bottom=0)
        ax.legend(frameon=False, labelcolor=INK, loc="upper left", fontsize=9)
    axes[1].set_ylim(0, max(v for g in bw.values() for v in g.values()) * 1.25)
    fig.tight_layout()
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(out)


def main():
    dirs = sys.argv[1:]
    if not dirs:
        sys.exit(__doc__)
    groups = load(dirs)
    if len(groups) > len(SERIES):
        sys.exit(f"參數組合太多（{len(groups)} 組，最多 {len(SERIES)} 組）：分開畫")
    bw = bandwidth(dirs)
    L = LABELS[pick_font()]
    single = len(groups) == 1
    name = lambda g: g or L["default"]
    loss = sorted({l for lv in groups.values() for l in lv})
    n = [len(rows) for lv in groups.values() for rows in lv.values()]

    fig, axes = plt.subplots(1, 4, figsize=(17, 4.2), facecolor=SURFACE)
    style(axes[0], L["rate"] + "\n" + L["n"].format(k=min(n) if min(n) == max(n) else f"{min(n)}-{max(n)}"),
          "%", L["loss"])
    style(axes[1], L["conv"], L["sec"], L["loss"])
    style(axes[2], L["close1"] if single else L["close"], L["sec"], L["loss"])
    style(axes[3], L["bw"], "kB/s", L["loss"])
    axes[0].axhline(95, color=MUTED, linewidth=1, linestyle="--")
    axes[0].text(loss[0], 94, L["target"], color=MUTED, fontsize=9, va="top", ha="left")
    axes[1].axhline(5, color=MUTED, linewidth=1, linestyle="--")

    table = []
    for gi, (g, lv) in enumerate(groups.items()):
        color = SERIES[gi]
        label = None if single else name(g)
        rate = {l: 100.0 * sum(int(r["converged_5s"]) for r in rows) / len(rows) for l, rows in lv.items()}
        conv, close = stats(lv, "conv_s"), stats(lv, "close_s")
        line(axes[0], [(l, rate[l]) for l in lv], color, label)
        for ax, st in ((axes[1], conv), (axes[2], close)):
            if single:
                line(ax, [(l, st[l][0]) for l in lv], SERIES[0], L["median"])
                line(ax, [(l, st[l][1]) for l in lv], SERIES[1], "P90")
            else:
                line(ax, [(l, st[l][0]) for l in lv], color, label)
        b = bw.get(g, {})
        line(axes[3], [(l, b.get(l)) for l in lv], color, label)
        if single:
            for x, y in b.items():
                axes[3].annotate(f"{y:.1f}", (x, y), textcoords="offset points", xytext=(0, 8), ha="center",
                                 color=INK, fontsize=8)
        for l, rows in lv.items():
            table.append((name(g), l, len(rows), rate[l], conv[l], close[l], b.get(l)))

    axes[0].set_ylim(0, 108)
    for ax in axes[1:]:
        ax.set_ylim(bottom=0)
    for ax in axes:
        ax.set_xticks(loss)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(frameon=False, labelcolor=INK, loc="best", fontsize=9)
    fig.tight_layout()
    out = os.path.join(dirs[0], "loss_sweep.png")
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(out)

    if not single:
        bars(groups, bw, L, name, os.path.join(dirs[0], "loss_params.png"))

    # 表格版（同一份數字，給報告用）
    fmt = lambda v: f"{v:.2f} s" if v is not None else "—"
    print("| 參數 | 掉包率 | 次數 | 5 s 收斂率 | 收斂中位數／P90 | 完成→全隊確認 中位數／P90 | mesh kB/s |")
    print("|---|---|---|---|---|---|---|")
    for g, l, k, r, c, cl, b in table:
        print(f"| {g} | {l}% | {k} | {r:.0f}% | {fmt(c[0])}／{fmt(c[1])} | {fmt(cl[0])}／{fmt(cl[1])} | "
              + (f"{b:.1f}" if b is not None else "—") + " |")


if __name__ == "__main__":
    main()
