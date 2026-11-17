#!/usr/bin/env python3
"""TDR 參數實驗的報告（docs/tdr_experiment_plan.md 第五、六節）：讀 cbba_sim tdr 的輸出，照入選順序挑候選參數。

    python3 tdr_report.py <cbba_sim tdr 的輸出目錄> [--baseline 50,1] [--eta-tol 0.10] [--no-plots]

入選順序（計畫第四節）：
  1. 底線：期限違反 = 0、返航餘量 ≥ −margin_tol（0.5 %）、完成率不低於同一個 γ 的 w_E = 0 超過 1 個百分點、
     S1～S3 每一項選對比例 ≥ choice_min（90 %）、不掉包時 5 s 收斂率 ≥ 95 %
  2. P0／P1 到達時間：各參數組和基準組（預設 w_E 50、γ 1）配對比較；P0 中位數在最好的那組的 eta_tol 以內
  3. 在 2 的範圍內選耗電最低的（正式值），次低的當備選
  4. 弱網：列出候選組在每個掉包率的 5 s 收斂率、收斂時間、重複認領、非被迫換手、分數平手率
另外列 S1～S3 的「選對」比例（時間差、能量差、時能取捨）當判讀依據；實際折扣價值用固定的 half_life，不同 γ 可以比。
輸出 <目錄>/report.md 和圖（沒有 matplotlib 時略過）：
  tdr_heatmap.png   w_E × γ 的熱圖：P0 到達中位／P95、耗電與折扣價值（和基準組比）、完成率、最低返航餘量
  tdr_tradeoff.png  時間－耗電取捨：每組相對基準組的耗電 vs P0 到達時間
  tdr_choice.png    S1～S3 選對的比例
  tdr_loss.png      弱網（有多個掉包率時）：候選組＋基準組的 5 s 收斂率、收斂 P95、重複認領、非被迫換手
主機沒有 matplotlib 時用 cbba-sim 映像（有 matplotlib）：
  docker run --rm --user $(id -u):$(id -g) -e MPLCONFIGDIR=/tmp -v <repo>:/repo -v <輸出的上層>:/data \
    --entrypoint python3 cbba-sim:jazzy /repo/ros2_ws/src/cbba_core/scripts/tdr_report.py /data/<目錄>
"""
import argparse
import csv
import os
import statistics
from collections import defaultdict


def read(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def num(x):
    return float(x)


def median(values):
    return statistics.median(values) if values else None


def p95(values):
    if not values:
        return None
    v = sorted(values)
    pos = 0.95 * (len(v) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (pos - lo)


def fmt(x, digits=1, pct=False):
    if x is None:
        return '—'
    return f'{x * 100:.{digits}f}%' if pct else f'{x:.{digits}f}'


def rel(new, base):
    if new is None or base is None or base == 0:
        return None
    return (new - base) / base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--baseline', default='50,1', help='基準參數組 w_E,gamma（預設 50,1 = 目前的暫定值）')
    ap.add_argument('--eta-tol', type=float, default=0.10, help='P0 到達時間在最好的那組的多少比例以內算同一級')
    ap.add_argument('--margin-tol', type=float, default=0.5,
                    help='返航餘量容許的誤差（%%）：模型不含待命耗電、實際耗電有 ±10%% 誤差，略小於 0 不算違反')
    ap.add_argument('--choice-min', type=float, default=0.9, help='S1～S3 每一項選對比例的下限')
    ap.add_argument('--no-plots', action='store_true')
    args = ap.parse_args()

    runs = read(os.path.join(args.out, 'runs.csv'))
    tasks = read(os.path.join(args.out, 'tasks.csv'))
    summary = read(os.path.join(args.out, 'summary.csv'))
    base_key = tuple(float(x) for x in args.baseline.split(','))

    losses = sorted({num(r['loss_rate']) for r in runs})
    loss0 = losses[0]
    sets = sorted({(num(r['w_E']), num(r['gamma'])) for r in runs})
    scenarios = sorted({r['scenario'] for r in runs})
    label_hl = {(num(r['w_E']), num(r['gamma'])): r['half_life_set'] for r in runs}

    # 任務依 run 分組（算 S1～S3 的選對）
    tasks_by_run = defaultdict(list)
    for t in tasks:
        tasks_by_run[t['run_id']].append(t)

    def key_of(r):
        return (num(r['w_E']), num(r['gamma']))

    # ---------- 每個參數組在不掉包時的指標（各情境合併）----------
    stats = {}
    paired = defaultdict(dict)   # (scenario, trial) → set → run（不掉包）
    for r in runs:
        if num(r['loss_rate']) != loss0:
            continue
        paired[(r['scenario'], r['trial'])][key_of(r)] = r
    for s in sets:
        rows = [r for r in runs if key_of(r) == s and num(r['loss_rate']) == loss0]
        p0 = [num(r['P0_eta']) for r in rows if num(r['P0_eta']) >= 0]
        p1 = [num(r['P1_eta']) for r in rows if num(r['P1_eta']) >= 0]
        choice = defaultdict(lambda: [0, 0])
        for r in rows:
            for t in tasks_by_run[r['run_id']]:
                if t['expected'] in ('0', ''):
                    continue
                c = choice[t['label']]
                c[1] += 1
                c[0] += 1 if t['first_executor'] == t['expected'] else 0
        stats[s] = {
            'p0_med': median(p0), 'p0_p95': p95(p0), 'p1_med': median(p1), 'p1_p95': p95(p1),
            'energy': sum(num(r['energy_used']) for r in rows),
            'value': sum(num(r['discounted_value']) for r in rows),
            'completed': sum(int(r['completed']) for r in rows), 'tasks': sum(int(r['tasks']) for r in rows),
            'deadline_miss': sum(int(r['deadline_miss']) for r in rows),
            'margin': min((num(r['min_return_margin']) for r in rows), default=None),
            'conv5': sum(int(r['conv_5s']) for r in rows) / len(rows) if rows else None,
            'choice': {k: v[0] / v[1] for k, v in choice.items() if v[1] > 0},
            'unforced': statistics.mean(int(r['handoff_unforced']) for r in rows) if rows else None,
            'near_tie': sum(int(r['near_tie']) for r in rows),
        }
        stats[s]['completion'] = stats[s]['completed'] / stats[s]['tasks'] if stats[s]['tasks'] else None

    # 配對比較：同一個 (情境, trial) 的總和差（P0 到達、耗電、折扣價值）
    def paired_rel(s, ref, field, only_valid=False):
        a = b = 0.0
        for runs_of in paired.values():
            if s not in runs_of or ref not in runs_of:
                continue
            x, y = num(runs_of[s][field]), num(runs_of[ref][field])
            if only_valid and (x < 0 or y < 0):
                continue
            a += x
            b += y
        return rel(a, b)

    # ---------- 1. 底線 ----------
    reasons = {}
    for s in sets:
        st = stats[s]
        r = []
        if st['deadline_miss'] > 0:
            r.append(f"期限違反 {st['deadline_miss']}")
        if st['margin'] is not None and st['margin'] < -args.margin_tol:
            r.append(f"返航餘量 {st['margin']:.1f} %")
        zero = (0.0, s[1])
        if zero in stats and st['completion'] is not None and stats[zero]['completion'] is not None and \
                st['completion'] < stats[zero]['completion'] - 0.01:
            r.append(f"完成率 {fmt(st['completion'], pct=True)} < w_E=0 的 {fmt(stats[zero]['completion'], pct=True)}")
        wrong = [k for k, v in sorted(st['choice'].items()) if v < args.choice_min]
        if wrong:
            r.append('選錯 ' + '、'.join(f"{k} {v * 100:.0f}%" for k, v in ((k, st['choice'][k]) for k in wrong)))
        if st['conv5'] is not None and st['conv5'] < 0.95:
            r.append(f"5 s 收斂 {fmt(st['conv5'], pct=True)}")
        reasons[s] = r
    passed = [s for s in sets if not reasons[s]]

    # ---------- 2、3. P0 到達 → 耗電 ----------
    ranked = []
    if passed:
        best = min((stats[s]['p0_med'] for s in passed if stats[s]['p0_med'] is not None), default=None)
        within = [s for s in passed if best is None or stats[s]['p0_med'] is None or
                  stats[s]['p0_med'] <= best * (1 + args.eta_tol)]
        ranked = sorted(within, key=lambda s: stats[s]['energy'])

    # ---------- 報告 ----------
    lines = ['# TDR 參數實驗報告', '',
             f'輸出：`{args.out}`；情境 {", ".join(scenarios)}；參數組 {len(sets)}；掉包率 '
             f'{", ".join(f"{x * 100:.0f}%" for x in losses)}；基準組 w_E={base_key[0]:g}、γ={base_key[1]:g}', '',
             '## 1. 不掉包時各參數組（各情境合併）', '',
             '「Δ」是和基準組的配對比較（同一個情境、同一個 trial 的總和）。P0／P1 到達時間是秒（從插入起算）。', '',
             '| w_E | γ | half_life | 底線 | P0 到達 中位／P95 | ΔP0 | P1 到達 中位／P95 | 耗電 Δ | 折扣價值 Δ | '
             '完成率 | 5 s 收斂 | 非被迫換手 | 平手（<1 %） |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for s in sets:
        st = stats[s]
        lines.append(
            f"| {s[0]:g} | {s[1]:g} | {label_hl[s]} | {'；'.join(reasons[s]) if reasons[s] else '通過'} | "
            f"{fmt(st['p0_med'])}／{fmt(st['p0_p95'])} | {fmt(paired_rel(s, base_key, 'P0_eta', True), pct=True)} | "
            f"{fmt(st['p1_med'])}／{fmt(st['p1_p95'])} | {fmt(paired_rel(s, base_key, 'energy_used'), pct=True)} | "
            f"{fmt(paired_rel(s, base_key, 'discounted_value'), pct=True)} | {fmt(st['completion'], pct=True)} | "
            f"{fmt(st['conv5'], pct=True)} | {fmt(st['unforced'], 2)} | {st['near_tie']} |")

    labels = sorted({k for s in sets for k in stats[s]['choice']})
    if labels:
        lines += ['', '## 2. 情境的判讀（選對的比例，不掉包）', '',
                  'S1：緊急任務給快的；S2：給電量多的（w_E = 0 時平手比機號會給錯）；S3：P0 給快但耗電的、P3 給慢但省電的。', '',
                  '| w_E | γ | ' + ' | '.join(labels) + ' |', '|---|---|' + '---|' * len(labels)]
        for s in sets:
            lines.append(f'| {s[0]:g} | {s[1]:g} | ' +
                         ' | '.join(fmt(stats[s]['choice'].get(k), 0, pct=True) for k in labels) + ' |')

    lines += ['', '## 3. 候選參數（照入選順序）', '']
    if not ranked:
        lines.append('沒有參數組通過底線，見第 1 節的原因。')
    else:
        lines.append(f'通過底線 {len(passed)} 組；P0 到達中位數在最好的 {args.eta_tol * 100:.0f} % 以內、依耗電排序：')
        lines.append('')
        for i, s in enumerate(ranked[:3]):
            tag = '正式值' if i == 0 else '備選'
            lines.append(f'- **{tag}**：w_E = {s[0]:g}、half_life = {label_hl[s]} s'
                         f'（耗電 Δ {fmt(paired_rel(s, base_key, "energy_used"), pct=True)}、'
                         f'P0 到達 Δ {fmt(paired_rel(s, base_key, "P0_eta", True), pct=True)}）')

    # ---------- 4. 弱網 ----------
    if len(losses) > 1:
        show = list(ranked[:3])
        if base_key in stats and base_key not in show:
            show.append(base_key)
        lines += ['', '## 4. 弱網（候選組＋基準組，各情境分開）', '',
                  '| 情境 | w_E | γ | 掉包 | 5 s 收斂 | 收斂 中位／P95 (s) | 重複認領的試驗 | 非被迫換手 | 分數平手率 |',
                  '|---|---|---|---|---|---|---|---|---|']
        for row in summary:
            s = (num(row['w_E']), num(row['gamma']))
            if s not in show:
                continue
            lines.append(f"| {row['scenario']} | {s[0]:g} | {s[1]:g} | {num(row['loss_rate']) * 100:.0f}% | "
                         f"{fmt(num(row['conv_5s_rate']), pct=True)} | {fmt(num(row['converge_median']), 2)}／"
                         f"{fmt(num(row['converge_p95']), 2)} | {row['duplicate_runs']} | "
                         f"{fmt(num(row['handoff_unforced_mean']), 2)} | "
                         f"{fmt(num(row['near_tie_rate']), pct=True) if num(row['near_tie_rate']) >= 0 else '—'} |")

    if ranked:
        s = ranked[0]
        hi_loss = [row for row in summary if (num(row['w_E']), num(row['gamma'])) == s and
                   num(row['loss_rate']) == losses[-1]]
        conv_hi = statistics.mean(num(r['conv_5s_rate']) for r in hi_loss) if hi_loss else None
        lines += ['', '## 5. 結論句（照計畫 6.1，數字自動填入；請再確認）', '',
                  f'在不造成期限違反與返航存量不足的前提下，參數組（w_E = {s[0]:g}，P0～P3 half-life = {label_hl[s]} s）'
                  f'相較基準組使總耗電變化 {fmt(paired_rel(s, base_key, "energy_used"), pct=True)}，'
                  f'P0 到達時間變化 {fmt(paired_rel(s, base_key, "P0_eta", True), pct=True)}，'
                  f'且在 {losses[-1] * 100:.0f}% 掉包下 5 s 收斂率為 {fmt(conv_hi, pct=True)}（各情境平均）。']

    with open(os.path.join(args.out, 'report.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"報告：{os.path.join(args.out, 'report.md')}")

    if args.no_plots:
        return
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt  # noqa: F401
    except ImportError:
        print('沒有 matplotlib，略過圖（主機沒有時用 cbba-sim 映像跑，見檔頭）')
        return
    ctx = {'stats': stats, 'sets': sets, 'reasons': reasons, 'label_hl': label_hl, 'summary': summary,
           'losses': losses, 'loss0': loss0, 'base': base_key, 'ranked': ranked, 'labels': labels,
           'rel': paired_rel, 'out': args.out}
    for path in plot_all(ctx):
        print(f'圖：{path}')


# ===========================================================================
# 圖（英文標籤：容器裡沒有中文字型）
# ===========================================================================
def plot_all(c):
    import matplotlib.pyplot as plt
    out = []
    stats, sets, base = c['stats'], c['sets'], c['base']
    weights = sorted({s[0] for s in sets})
    gammas = sorted({s[1] for s in sets})

    def nan(x):
        return float('nan') if x is None else x

    # ---------- 1. 熱圖：w_E × γ（不掉包）----------
    panels = [
        ('P0 ETA median (s)', lambda s: stats[s]['p0_med'], '.0f', 'viridis_r'),
        ('P0 ETA P95 (s)', lambda s: stats[s]['p0_p95'], '.0f', 'viridis_r'),
        ('Energy vs baseline (%)', lambda s: None if c['rel'](s, base, 'energy_used') is None
         else 100 * c['rel'](s, base, 'energy_used'), '+.1f', 'RdYlGn_r'),
        ('Discounted value vs baseline (%)', lambda s: None if c['rel'](s, base, 'discounted_value') is None
         else 100 * c['rel'](s, base, 'discounted_value'), '+.1f', 'RdYlGn'),
        ('Completion rate (%)', lambda s: None if stats[s]['completion'] is None else 100 * stats[s]['completion'],
         '.1f', 'RdYlGn'),
        ('Min return margin (%)', lambda s: stats[s]['margin'], '.1f', 'RdYlGn'),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for ax, (title, get, f, cmap) in zip(axes.flat, panels):
        grid = [[nan(get((w, g))) if (w, g) in stats else float('nan') for w in weights] for g in gammas]
        im = ax.imshow(grid, aspect='auto', origin='lower', cmap=cmap)
        ax.set_xticks(range(len(weights)), [f'{w:g}' for w in weights])
        ax.set_yticks(range(len(gammas)), [f'{g:g}' for g in gammas])
        ax.set_xlabel('w_E')
        ax.set_ylabel('half-life x gamma')
        ax.set_title(title, fontsize=11)
        for i, g in enumerate(gammas):
            for j, w in enumerate(weights):
                v = grid[i][j]
                if v == v:
                    failed = bool(c['reasons'].get((w, g)))
                    ax.text(j, i, format(v, f) + ('\nX' if failed else ''), ha='center', va='center', fontsize=8,
                            color='black', fontweight='bold' if (w, g) == base else 'normal')
        fig.colorbar(im, ax=ax, shrink=0.85)
    fig.suptitle(f'TDR parameter screening, loss {c["loss0"] * 100:.0f}%   '
                 f'(X = fails baseline checks, bold = baseline w_E={base[0]:g}, gamma={base[1]:g})')
    fig.tight_layout()
    out.append(save(fig, c['out'], 'tdr_heatmap.png'))

    # ---------- 2. 時間－耗電取捨（配對比較）----------
    fig, ax = plt.subplots(figsize=(8, 6))
    markers = {g: m for g, m in zip(gammas, ['v', 'o', '^', 's', 'D'])}
    cmap = plt.get_cmap('plasma')
    for s in sets:
        x = c['rel'](s, base, 'energy_used')
        y = c['rel'](s, base, 'P0_eta', True)
        if x is None or y is None:
            continue
        color = cmap(weights.index(s[0]) / max(1, len(weights) - 1) * 0.85)
        failed = bool(c['reasons'][s])
        ax.scatter(100 * x, 100 * y, s=120, marker=markers[s[1]], facecolors='none' if failed else color,
                   edgecolors=color, linewidths=2, zorder=3)
        ax.annotate(f'{s[0]:g}/{s[1]:g}', (100 * x, 100 * y), textcoords='offset points', xytext=(6, 4), fontsize=8)
    for k, s in enumerate(c['ranked'][:3]):
        x, y = c['rel'](s, base, 'energy_used'), c['rel'](s, base, 'P0_eta', True)
        if x is not None and y is not None:
            ax.scatter(100 * x, 100 * y, s=380, marker='*' if k == 0 else 'P', facecolors='none',
                       edgecolors='green', linewidths=1.5, zorder=4)
    ax.axhline(0, color='gray', lw=0.8)
    ax.axvline(0, color='gray', lw=0.8)
    ax.set_xlabel('Total energy vs baseline (%)   <- saves energy')
    ax.set_ylabel('P0 ETA vs baseline (%)   <- faster')
    ax.set_title('Time-energy trade-off (labels w_E/gamma; hollow = fails checks; green = candidates)')
    for g, m in markers.items():
        ax.scatter([], [], marker=m, color='gray', label=f'gamma {g:g}')
    ax.legend(loc='best', fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out.append(save(fig, c['out'], 'tdr_tradeoff.png'))

    # ---------- 3. 情境判讀：選對的比例 ----------
    if c['labels']:
        labels = c['labels']
        grid = [[nan(stats[s]['choice'].get(k)) * 100 if stats[s]['choice'].get(k) is not None else float('nan')
                 for k in labels] for s in sets]
        fig, ax = plt.subplots(figsize=(1.4 * len(labels) + 3, 0.38 * len(sets) + 1.8))
        im = ax.imshow(grid, aspect='auto', cmap='RdYlGn', vmin=0, vmax=100)
        ax.set_xticks(range(len(labels)), labels)
        ax.set_yticks(range(len(sets)), [f'w_E {s[0]:g}, gamma {s[1]:g}' for s in sets])
        for i, row in enumerate(grid):
            for j, v in enumerate(row):
                if v == v:
                    ax.text(j, i, f'{v:.0f}', ha='center', va='center', fontsize=8)
        ax.set_title('Expected choice rate (%)  S1 faster, S2 more battery, S3 P0 fast / P3 frugal')
        fig.colorbar(im, ax=ax, shrink=0.8)
        fig.tight_layout()
        out.append(save(fig, c['out'], 'tdr_choice.png'))

    # ---------- 4. 弱網：候選組＋基準組 ----------
    if len(c['losses']) > 1:
        show = list(c['ranked'][:3])
        if base in stats and base not in show:
            show.append(base)
        if not show:
            show = sets
        metrics = [('conv_5s_rate', '5 s convergence rate (%)', 100, 'mean'),
                   ('converge_p95', 'Convergence P95 (s), worst scenario', 1, 'max'),
                   ('duplicate_runs', 'Runs with duplicate claims', 1, 'sum'),
                   ('handoff_unforced_mean', 'Unforced handoffs per run', 1, 'mean')]
        fig, axes = plt.subplots(1, len(metrics), figsize=(4.4 * len(metrics), 4))
        for ax, (field, title, scale, how) in zip(axes, metrics):
            for s in show:
                ys = []
                for loss in c['losses']:
                    vals = [float(r[field]) for r in c['summary'] if (float(r['w_E']), float(r['gamma'])) == s
                            and float(r['loss_rate']) == loss and float(r[field]) >= 0]
                    if not vals:
                        ys.append(float('nan'))
                        continue
                    agg = {'mean': sum(vals) / len(vals), 'max': max(vals), 'sum': sum(vals)}[how]
                    ys.append(agg * scale)
                ax.plot([x * 100 for x in c['losses']], ys, marker='o',
                        label=f'w_E {s[0]:g}, gamma {s[1]:g}' + (' (baseline)' if s == base else ''))
            if field == 'conv_5s_rate':
                ax.axhline(95, color='red', ls='--', lw=0.8)
            if field == 'converge_p95':
                ax.axhline(5, color='red', ls='--', lw=0.8)
            ax.set_xlabel('packet loss (%)')
            ax.set_title(title, fontsize=10)
            ax.grid(alpha=0.3)
        axes[0].legend(fontsize=8)
        fig.suptitle('Weak network: candidates and baseline (all scenarios)')
        fig.tight_layout()
        out.append(save(fig, c['out'], 'tdr_loss.png'))
    return out


def save(fig, out_dir, name):
    import matplotlib.pyplot as plt
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path

if __name__ == '__main__':
    main()
