#!/usr/bin/env python3
"""第二個 terminal：對正在跑的 itest demo --interactive 插入任務點、起火點（2026-10-15）。

  terminal 1：python3 docker/itest/itest.py demo --interactive --area -15 15 -12 8   開環境（Gazebo、PX4、CBBA、假狗）
  terminal 2：python3 docker/itest/insert.py                                       輸入指令（互動，Ctrl-D 或 exit 離開）
              python3 docker/itest/insert.py t 12 -4 d=10                          只送一個指令

指令（和 terminal 1 的說明相同）：
  t [x y] [d=秒]    任務點（藍色平台）；不給 x y = 範圍內隨機
  t G2 [d=秒]       用場景的點名
  tf [x y] [d=秒]   任務點、而且那裡起火
  f [x y]           只起火、不建任務
  m uav2 [x y] [d=秒]   人工指派（地面站／平板）：指定那台去那裡、期間不參與 CBBA；m dog51 …、m uav2 clear
  l 列出  a 範圍  h 說明  q 不再插入（做完就結束）

和 itest 之間只靠 docker/itest/out/interactive/ 的檔案：指令加到 commands.txt，itest 處理後回覆寫到 replies.txt，
這裡讀回來印；session.json 表示有一個 --interactive 在跑（itest 結束時刪掉）。只用 Python 標準庫，在主機執行。
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
IA_DIR = os.path.join(HERE, "out", "interactive")
SESSION = os.path.join(IA_DIR, "session.json")
COMMANDS = os.path.join(IA_DIR, "commands.txt")
REPLIES = os.path.join(IA_DIR, "replies.txt")


def session():
    """正在跑的 --interactive（session.json，而且那個程序還活著）；沒有就 None"""
    try:
        with open(SESSION, encoding="utf-8") as f:
            s = json.load(f)
        os.kill(s["pid"], 0)
        return s
    except (OSError, ValueError, KeyError):
        return None


def replies_size():
    try:
        return os.path.getsize(REPLIES)
    except OSError:
        return 0


def send(line, wait=3.0, quiet=0.6):
    """送一行指令，印出 itest 的回覆（有回覆後安靜 quiet 秒、或最多 wait 秒）"""
    s = session()
    if not s:
        print("沒有在跑的 itest demo --interactive（先在另一個 terminal 開：python3 docker/itest/itest.py demo --interactive）")
        return False
    if not s.get("ready"):
        print("（環境還在起飛、CBBA 還沒上線：指令先排著，上線後才處理）")
    pos = replies_size()
    with open(COMMANDS, "a", encoding="utf-8") as f:
        f.write(line.strip() + "\n")
    start, last = time.time(), None
    while time.time() - start < wait and (last is None or time.time() - last < quiet):
        size = replies_size()
        if size > pos:
            with open(REPLIES, encoding="utf-8") as f:
                f.seek(pos)
                text = f.read()
            pos = size
            sys.stdout.write(text)
            sys.stdout.flush()
            last = time.time()
        time.sleep(0.1)
    if last is None and s.get("ready"):
        print("（itest 沒有回覆：可能已經結束或在忙，用 l 看狀態）")
    return True


def main():
    if len(sys.argv) > 1:
        sys.exit(0 if send(" ".join(sys.argv[1:])) else 1)
    s = session()
    if not s:
        print("沒有在跑的 itest demo --interactive（先在另一個 terminal 開：python3 docker/itest/itest.py demo --interactive）")
        sys.exit(1)
    x0, x1, y0, y1 = s["area"]
    print(f"連到 itest（輸出 {s['out']}，場景 {s['world']}，範圍 x {x0:g}～{x1:g}、y {y0:g}～{y1:g}）")
    print("指令：t [x y] [d=秒]｜t G2｜tf [x y]｜f [x y]｜m uav2 [x y] [d=秒]（人工指派）｜l 列出｜a 範圍｜h 說明｜q 不再插入｜exit 離開這個 terminal")
    while True:
        try:
            line = input("insert> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if line.strip() in ("exit", "quit"):
            return
        if not line.strip():
            continue
        if not send(line):
            return


if __name__ == "__main__":
    main()
