#!/usr/bin/env python3
"""畫 itest loss（第 5 週弱網量測）的結果：掉包率對 5 s 收斂率、收斂時間、mesh 頻寬（三張圖、各自一個 y 軸）。

  python3 docker/itest/plot_loss.py docker/itest/out/<時間>_loss     → 同目錄的 loss_sweep.png

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
SERIES = ["#2a78d6", "#eb6834"]
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"

LABELS = {
    "zh": dict(loss="掉包率（%）", rate="5 s 內收斂率（沒有重複認領）", conv="收斂時間（建立 → 每個任務恰好一台）",
               bw="mesh 頻寬（每台，收＋送）", sec="秒", target="目標 95%（30% 掉包）", n="每個掉包率 {k} 次",
               median="中位數"),
    "en": dict(loss="packet loss (%)", rate="converged within 5 s (no duplicate claims)",
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
    rows = []
    for d in dirs:
        with open(os.path.join(d, "trials.csv"), encoding="utf-8") as f:
            rows += list(csv.DictReader(f))
    # 開始時不乾淨、收不了尾的試驗不計（舊的 trials.csv 沒有這兩欄時全部計入）
    rows = [r for r in rows if r.get("clean", "1") == "1" and r.get("leftover", "0") in ("0", "")]
    levels = {}
    for r in rows:
        levels.setdefault(int(r["loss"]), []).append(r)
    return dict(sorted(levels.items()))


def quantile(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))] if xs else None


def bandwidth(dirs):
    """levels.csv 的 mesh_kBps（每個目錄各自的；同一個掉包率有多筆時取平均）"""
    out = {}
    for d in dirs:
        path = os.path.join(d, "levels.csv")
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as f:
            for r in csv.DictReader(f):
                if r["mesh_kBps"]:
                    out.setdefault(int(r["loss"]), []).append(float(r["mesh_kBps"]))
    return {k: sum(v) / len(v) for k, v in out.items()}


def style(ax, title, ylabel, xlabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=11, loc="left")
    ax.set_xlabel(xlabel, color=MUTED)
    ax.set_ylabel(ylabel, color=MUTED)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.tick_params(colors=MUTED)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def main():
    dirs = sys.argv[1:]
    if not dirs:
        sys.exit(__doc__)
    levels = load(dirs)
    loss = list(levels)
    n = [len(levels[l]) for l in loss]
    rate = [100.0 * sum(int(r["converged_5s"]) for r in levels[l]) / len(levels[l]) for l in loss]
    convs = {l: [float(r["conv_s"]) for r in levels[l] if r["conv_s"]] for l in loss}
    med = [quantile(convs[l], 0.5) for l in loss]
    p90 = [quantile(convs[l], 0.9) for l in loss]
    bw = bandwidth(dirs)

    L = LABELS[pick_font()]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), facecolor=SURFACE)

    ax = axes[0]
    style(ax, L["rate"] + "\n" + L["n"].format(k=min(n) if min(n) == max(n) else f"{min(n)}～{max(n)}"),
          "%", L["loss"])
    ax.plot(loss, rate, color=SERIES[0], linewidth=2, marker="o", markersize=8)
    ax.axhline(95, color=MUTED, linewidth=1, linestyle="--")
    ax.text(loss[0], 94, L["target"], color=MUTED, fontsize=9, va="top", ha="left")
    ax.set_ylim(0, 108)

    ax = axes[1]
    style(ax, L["conv"], L["sec"], L["loss"])
    for ys, label, color in ((med, L["median"], SERIES[0]), (p90, "P90", SERIES[1])):
        pts = [(x, y) for x, y in zip(loss, ys) if y is not None]
        if pts:
            ax.plot(*zip(*pts), color=color, linewidth=2, marker="o", markersize=8, label=label)
    ax.axhline(5, color=MUTED, linewidth=1, linestyle="--")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(frameon=False, labelcolor=INK, loc="upper left")
    ax.set_ylim(bottom=0)

    ax = axes[2]
    style(ax, L["bw"], "kB/s", L["loss"])
    pts = [(x, bw[x]) for x in loss if x in bw]
    if pts:
        ax.plot(*zip(*pts), color=SERIES[0], linewidth=2, marker="o", markersize=8)
        for x, y in pts:
            ax.annotate(f"{y:.1f}", (x, y), textcoords="offset points", xytext=(0, 8), ha="center",
                        color=INK, fontsize=8)
    ax.set_ylim(bottom=0)

    for ax in axes:
        ax.set_xticks(loss)
    fig.tight_layout()
    out = os.path.join(dirs[0], "loss_sweep.png")
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(out)

    # 表格版（同一份數字，給報告用）
    print("| 掉包率 | 次數 | 5 s 收斂率 | 收斂中位數 | P90 | mesh kB/s |")
    print("|---|---|---|---|---|---|")
    for x, k, r, m, p in zip(loss, n, rate, med, p90):
        fmt = lambda v: f"{v:.2f} s" if v is not None else "—"
        print(f"| {x}% | {k} | {r:.0f}% | {fmt(m)} | {fmt(p)} | {bw.get(x, float('nan')):.1f} |")


if __name__ == "__main__":
    main()
