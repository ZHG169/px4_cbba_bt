#!/usr/bin/env python3
"""把 record_cameras.py 錄的各台下視相機畫面，照時間拼成「三台直排」的每一格（2026-10-19）。

在 sim 容器跑（有 PIL）：
  python3 compose_cameras.py <itest 輸出目錄> --start <牆上時間 s> --end <牆上時間 s> [--fps 5] [--height 720]
--start、--end 是 Gazebo 俯瞰錄影（gazebo.mp4）開始、結束的時間（itest 記的），所以拼出來的和 gazebo.mp4 從同一刻開始；
每一格用每台在那個時刻以前最新的一張。輸出 <輸出目錄>/camframes/%05d.jpg 和 fps.txt，主機的 ffmpeg 再和 gazebo.mp4 左右並排。
沒有畫面的那台畫灰底和「no image」。圖上的字用英文（容器裡沒有中文字型）。
"""
import argparse
import bisect
import os

from PIL import Image, ImageDraw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--start", type=float, required=True)
    ap.add_argument("--end", type=float, required=True)
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--height", type=int, default=576, help="整欄的高度（和縮小後的 gazebo.mp4 一樣；itest 給 576）")
    args = ap.parse_args()

    root = os.path.join(args.out_dir, "cameras")
    uavs = sorted(d for d in os.listdir(root) if d.startswith("uav")) if os.path.isdir(root) else []
    if not uavs:
        raise SystemExit(f"{root} 沒有相機畫面")
    shots = {}
    for u in uavs:
        files = sorted(f for f in os.listdir(os.path.join(root, u)) if f.endswith(".jpg"))
        shots[u] = ([int(f[:-4]) / 1000.0 for f in files], [os.path.join(root, u, f) for f in files])

    tile_h = args.height // len(uavs)
    sample = next((paths[0] for _, paths in shots.values() if paths), None)
    if sample is None:
        raise SystemExit("相機畫面都是空的")
    with Image.open(sample) as im:
        tile_w = round(im.width * tile_h / im.height)
    tile_w += tile_w % 2   # ffmpeg 的 yuv420p 要偶數寬

    out = os.path.join(args.out_dir, "camframes")
    os.makedirs(out, exist_ok=True)
    cache = {}
    n = 0
    t = args.start
    while t <= args.end:
        frame = Image.new("RGB", (tile_w, tile_h * len(uavs)), (60, 60, 60))
        draw = ImageDraw.Draw(frame)
        for i, u in enumerate(uavs):
            times, paths = shots[u]
            k = bisect.bisect_right(times, t) - 1
            y0 = i * tile_h
            if k >= 0:
                path = paths[k]
                if cache.get(u, (None,))[0] != path:
                    with Image.open(path) as im:
                        cache[u] = (path, im.convert("RGB").resize((tile_w, tile_h)))
                frame.paste(cache[u][1], (0, y0))
            else:
                draw.text((10, y0 + tile_h // 2), "no image", fill=(220, 220, 220))
            draw.rectangle([0, y0, tile_w - 1, y0 + tile_h - 1], outline=(255, 255, 255), width=2)
            draw.rectangle([0, y0, 118, y0 + 20], fill=(0, 0, 0))
            draw.text((6, y0 + 4), f"{u} down camera", fill=(255, 255, 255))
        frame.save(os.path.join(out, f"{n:05d}.jpg"), quality=85)
        n += 1
        t += 1.0 / args.fps
    with open(os.path.join(out, "fps.txt"), "w") as f:
        f.write(f"{args.fps:g}")
    print(f"[compose_cameras] {len(uavs)} 台、{n} 格（{n / args.fps:.0f} s，每台 {tile_w}×{tile_h}）→ {out}")


if __name__ == "__main__":
    main()
