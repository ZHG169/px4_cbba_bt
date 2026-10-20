#!/usr/bin/env python3
"""量 itest demo 每個任務的懸停誤差（2026-10-19）：從 events.jsonl 的位置紀錄算。

用法（主機）：python3 docker/itest/hover_error.py [docker/itest/out/<時間>_demo] [--reach 0.25]
不給資料夾時用最新的 *_demo；--reach 是「到達」的距離，和 uav_bt.yaml 的 reach_tolerance 一樣（2026-10-19 起 0.25 m，原本 0.5）。

每個指派：執行的那台第一次進到任務點 reach 內算到達，之後到回報完成為止：
  平均、最大   水平誤差（含剛到達、還在靠近的那段，所以最大值通常接近 reach）
  穩定         到達後多久進到 0.1 m 內（之後一直在 0.1 m 內）
  穩定後       穩定之後的最大水平誤差（真正的懸停晃動）
  過頭         沿著飛來的方向越過任務點的最大距離（0 = 沒有衝過頭）
  高度         最大高度誤差
懸停監控火點的那台會移到火點上空，誤差大是設計如此（看備註）。
2026-10-19 修正：原本的「衝過頭」是到達後 3 s 內的最大距離，其實就是到達那一刻的 ~0.45 m，不是衝過頭。
"""
import argparse
import json
import math
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?")
    ap.add_argument("--reach", type=float, default=0.25, help="到達的距離（m），和 reach_tolerance 一樣")
    args = ap.parse_args()
    reach = args.reach
    if args.out:
        out = Path(args.out)
    else:
        runs = sorted(Path(__file__).resolve().parent.glob("out/*_demo"))
        if not runs:
            raise SystemExit("找不到 docker/itest/out/*_demo")
        out = runs[-1]
    events = [json.loads(line) for line in open(out / "events.jsonl", encoding="utf-8")]
    t0 = events[0]["t"]
    pos = {}
    for e in events:
        if e["kind"] == "pos":
            pos.setdefault(e["uav"], []).append((e["t"], e["x"], e["y"], e["z"]))
    results = {e["msg"]["task_id"]: e["t"] for e in events if e["kind"] == "result"}
    fires = [e["msg"]["position"] for e in events if e["kind"] == "new_task" and e["msg"].get("type") == 2]

    print(f"{out.name}（到達 = 進到 {reach} m 內）")
    print(f"{'無人機':<6}{'任務':<10}{'位置':<16}{'到達':>7}{'懸停':>7}{'平均':>7}{'最大':>7}{'穩定':>7}{'穩定後':>7}"
          f"{'過頭':>7}{'高度':>7}  備註")
    for e in events:
        if e["kind"] != "assigned" or not e["msg"]["task_id"]:
            continue
        uav, msg = e["uav"], e["msg"]
        tx, ty, tid = msg["position"]["x"], msg["position"]["y"], msg["task_id"]
        end = results.get(tid, float("inf"))
        track = [p for p in pos.get(uav, []) if e["t"] <= p[0] <= end]
        arrive = next((p for p in track if math.hypot(p[1] - tx, p[2] - ty) < reach), None)
        name = f"{tid:08X}"
        where = f"({tx:.1f}, {ty:.1f})"
        if not arrive:
            print(f"uav{uav:<3}{name:<10}{where:<16}  沒有進到 {reach} m 內（被換掉、交回或還在飛）")
            continue
        hover = [p for p in track if p[0] >= arrive[0]]
        d = [math.hypot(p[1] - tx, p[2] - ty) for p in hover]
        dz = max(abs(p[3] - 5.0) for p in hover)
        # 穩定：之後一直在 0.1 m 內的第一個時刻
        settle_i = next((i for i in range(len(d)) if all(x < 0.1 for x in d[i:])), None)
        settle = f"{hover[settle_i][0] - arrive[0]:6.1f}s" if settle_i is not None else "     —"
        after = f"{max(d[settle_i:]):6.2f}m" if settle_i is not None else "     —"
        # 過頭：飛來的方向（到達前 1 s 的位置 → 任務點）上，越過任務點的距離
        before = [p for p in track if arrive[0] - 1.2 <= p[0] <= arrive[0] - 0.8]
        over = 0.0
        if before:
            ux, uy = tx - before[0][1], ty - before[0][2]
            n = math.hypot(ux, uy)
            if n > 1e-6:
                over = max(0.0, max(((p[1] - tx) * ux + (p[2] - ty) * uy) / n for p in hover))
        note = ""
        if max(d) > 1.0 and any(math.hypot(f["x"] - tx, f["y"] - ty) <= 3.0 for f in fires):
            note = "旁邊有火點：移到火點上空監控"
        print(f"uav{uav:<3}{name:<10}{where:<16}{arrive[0] - t0:6.1f}s{hover[-1][0] - arrive[0]:6.1f}s"
              f"{sum(d) / len(d):6.2f}m{max(d):6.2f}m{settle}{after}{over:6.2f}m{dz:6.2f}m  {note}")


if __name__ == "__main__":
    main()
