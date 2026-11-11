#!/usr/bin/env python3
"""畫 itest loss（第 5 週弱網量測）的「每台接到幾個任務」：兩張長條圖。

  python3 docker/itest/plot_tasks.py docker/itest/out/<時間>_loss     → 同目錄的 loss_tasks.png

  左：每個掉包率、每台平均分到幾個任務（長條），誤差線 = 這個掉包率裡最少～最多，虛線 = 理論值
  右：每個掉包率的試驗分成三類（100% 堆疊）：分配和理論相同、不平均（或 10 s 內還在調整）但全部做完、有任務沒做完
      （第三類就是實驗規劃的「任務完成覆蓋率」沒達到 100%）

要 matplotlib（主機沒有時在 sim 容器裡跑：docker exec cbba_sim python3 /home/ncrl/CBBA_BT/docker/itest/plot_tasks.py
/home/ncrl/CBBA_BT/docker/itest/out/<時間>_loss）。字型、顏色和 plot_loss.py 一樣：有中文字型才用中文標籤。
讀 trials.csv（每次試驗一列），多個目錄一起給時合併。
"""
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from plot_loss import GRID, INK, MUTED, SURFACE, pick_font, style  # noqa: E402

# 參考調色盤（dataviz 的 palette.md）的類別色，照順序：uav1、uav2、uav3；狗（2026-11-09，loss --dogs）接在後面：
# 紫、棕、灰藍（和無人機的藍、橘、綠分得開）
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#8e44ad", "#a8641f", "#5b6f86", "#c2185b", "#00838f", "#6d4c41"]

LABELS = {
    "zh": dict(loss="掉包率（%）", got="每台分到幾個任務（平均；誤差線 = 最少～最多）", tasks="任務數",
               expect="理論 {e} 個", agent="uav{v}", mix="試驗的分配結果（{k} 個任務）", pct="試驗比例（%）",
               same="和理論相同", uneven="不平均（或 10 s 內還在調整），全部做完", missed="有任務沒做完",
               n="每個掉包率 {k} 次"),
    "en": dict(loss="packet loss (%)", got="tasks per vehicle (mean; whisker = min to max)", tasks="tasks",
               expect="expected {e}", agent="uav{v}", mix="allocation outcome per trial ({k} tasks)",
               pct="share of trials (%)", same="matches expected split", uneven="uneven (or still settling at 10 s), all done",
               missed="some task not finished", n="n = {k} per level"),
}


def load(dirs):
    """每個掉包率的試驗（只去掉開始時不乾淨的；做不完的留著，算在「沒人接或做不完」）"""
    levels = {}
    for d in dirs:
        with open(os.path.join(d, "trials.csv"), encoding="utf-8") as f:
            for r in csv.DictReader(f):
                if r.get("clean", "1") == "1":
                    levels.setdefault(int(r["loss"]), []).append(r)
    return dict(sorted(levels.items()))


def counts(s):
    return [int(c) for c in s.split("/")]


def main():
    dirs = sys.argv[1:]
    if not dirs:
        sys.exit(__doc__)
    levels = load(dirs)
    loss = list(levels)
    first = next(iter(levels.values()))[0]
    n_agents = len(counts(first["got_per_agent"]))
    # trials.csv 有 agents 欄（2026-11-09 起）時用它的名稱（uav1、dog50…），舊的照順序叫 uav1、uav2…
    names = first["agents"].split("/") if first.get("agents") else [f"uav{a + 1}" for a in range(n_agents)]
    # 狗一律叫 dog1～dogN（2026-11-09 使用者）：舊的 trials.csv 記的是機號（dog50…），照順序改名
    dogs = [nm for nm in names if nm.startswith("dog")]
    names = [f"dog{dogs.index(nm) + 1}" if nm in dogs else nm for nm in names]
    expect = counts(next(iter(levels.values()))[0]["expected_per_agent"])
    n_tasks = int(next(iter(levels.values()))[0]["tasks"])

    L = LABELS[pick_font()]
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), facecolor=SURFACE)

    # ---- 左：每台平均分到幾個，誤差線 = 最少～最多
    ax = axes[0]
    n = [len(levels[l]) for l in loss]
    style(ax, L["got"] + "\n" + L["n"].format(k=min(n) if min(n) == max(n) else f"{min(n)}-{max(n)}"),
          L["tasks"], L["loss"])
    width = 0.8 / n_agents
    xs = range(len(loss))
    for a in range(n_agents):
        mean, lo, hi = [], [], []
        for l in loss:
            got = [counts(r["got_per_agent"])[a] for r in levels[l]]
            m = sum(got) / len(got)
            mean.append(m)
            lo.append(m - min(got))
            hi.append(max(got) - m)
        pos = [x - 0.4 + width * (a + 0.5) for x in xs]
        ax.bar(pos, mean, width, color=SERIES[a], edgecolor=SURFACE, linewidth=2, label=names[a])
        ax.errorbar(pos, mean, yerr=[lo, hi], fmt="none", ecolor=INK, elinewidth=1, capsize=3)
    if len(set(expect)) == 1:
        ax.axhline(expect[0], color=MUTED, linewidth=1, linestyle="--", label=L["expect"].format(e=expect[0]))
    ax.set_xticks(list(xs))
    ax.set_xticklabels([str(l) for l in loss])
    ax.set_ylim(0, max(6, max(max(counts(r["got_per_agent"])) for lv in levels.values() for r in lv) + 1))
    ax.legend(frameon=False, labelcolor=INK, loc="upper left", ncol=min(n_agents + 1, 4), fontsize=9)

    # ---- 右：試驗分成三類（100% 堆疊）
    ax = axes[1]
    style(ax, L["mix"].format(k=n_tasks), L["pct"], L["loss"])
    groups = {"same": [], "uneven": [], "missed": []}
    for l in loss:
        lv = levels[l]
        # 沒做完 = 收尾時限內有任務沒全隊確認結束（leftover）。10 s 內沒收斂但最後都做完的算「不平均」
        # （2026-11-04：70% 有一次路徑到 11 s 還在換，9 個任務都有人接、都做完）
        missed = sum(r.get("leftover", "0") not in ("0", "") for r in lv)
        same = sum(r["got_per_agent"] == r["expected_per_agent"] for r in lv
                   if r["conv_s"] != "" and r.get("leftover", "0") in ("0", ""))
        groups["missed"].append(100.0 * missed / len(lv))
        groups["same"].append(100.0 * same / len(lv))
        groups["uneven"].append(100.0 - groups["missed"][-1] - groups["same"][-1])
    bottom = [0.0] * len(loss)
    for key, color in (("same", SERIES[0]), ("uneven", SERIES[1]), ("missed", MUTED)):
        bars = ax.bar(list(xs), groups[key], 0.6, bottom=bottom, color=color, edgecolor=SURFACE, linewidth=2,
                      label=L[key])
        for b, v, y0 in zip(bars, groups[key], bottom):
            if v >= 8:                        # 太窄的段不標數字
                ax.text(b.get_x() + b.get_width() / 2, y0 + v / 2, f"{v:.0f}%", ha="center", va="center",
                        color=SURFACE, fontsize=8)
        bottom = [b + v for b, v in zip(bottom, groups[key])]
    ax.set_xticks(list(xs))
    ax.set_xticklabels([str(l) for l in loss])
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, labelcolor=INK, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3, fontsize=9)

    for a in axes:
        a.grid(axis="y", color=GRID, linewidth=0.8)
        a.set_axisbelow(True)                 # 格線在長條後面
    fig.tight_layout()
    out = os.path.join(dirs[0], "loss_tasks.png")
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(out)

    # 表格版（同一份數字，給報告用）
    print("| 掉包率 | 次數 | " + " | ".join(f"{names[a]} 平均（最少～最多）" for a in range(n_agents)) +
          " | 和理論相同 | 不平均 | 沒做完 |")
    print("|---" * (n_agents + 5) + "|")
    for i, l in enumerate(loss):
        cells = []
        for a in range(n_agents):
            got = [counts(r["got_per_agent"])[a] for r in levels[l]]
            cells.append(f"{sum(got) / len(got):.1f}（{min(got)}～{max(got)}）")
        print(f"| {l}% | {len(levels[l])} | " + " | ".join(cells) +
              f" | {groups['same'][i]:.0f}% | {groups['uneven'][i]:.0f}% | {groups['missed'][i]:.0f}% |")


if __name__ == "__main__":
    main()
