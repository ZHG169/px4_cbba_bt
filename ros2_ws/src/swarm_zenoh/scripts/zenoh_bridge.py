#!/usr/bin/env python3
"""GCS ↔ 載具的白名單 bridge（2026-10-21）：依角色產生 zenoh-bridge-ros2dds 與 CycloneDDS 的設定，然後啟動 bridge。

  載具：ros2 run swarm_zenoh zenoh_bridge --role vehicle --ns uav1 --gcs-ip 172.30.0.100
  GCS： ros2 run swarm_zenoh zenoh_bridge --role gcs --listen-ip 172.30.0.100
  只看設定不啟動：加 --dry-run

連法：GCS 是 Zenoh router（聽 tcp/<listen-ip>:7447），每台載具是 client（連 GCS）。載具之間不經過 Zenoh
（CBBA、位置廣播走自訂 UDP），所以不用 peer 之間互連；關掉 multicast scouting（mesh 上不送 Zenoh 的 multicast）。

中繼（--relay，2026-10-25）：GCS 直接連不到某些載具、要經過某一台（當「內網」）時，那一台用 --relay：Zenoh router，
往上連 GCS、同時在 --listen-ip:--listen-port 聽，讓其他載具連進來。**client 不會替別人轉送**——那一台是 client 時，
連到它的載具會變成單向或不通（使用者 10/25：「依某一台 agent 當內網會有 A 看到 B、B 看不到 A」）；router 之間雙向轉送：
  GCS（router）←── uav1（--relay，router）←── uav2（client，--gcs-ip 指到 uav1）
  ros2 run swarm_zenoh zenoh_bridge --role vehicle --ns uav1 --relay --gcs-ip <GCS> --listen-ip <uav1 的 mesh IP>
  ros2 run swarm_zenoh zenoh_bridge --role vehicle --ns uav2 --gcs-ip <uav1 的 mesh IP>

白名單（config/whitelist.yaml）→ bridge 的 allow 規則：
  載具：只允許自己的 namespace（--ns，可以給多個，例如模擬時 uav1 容器也替假狗 v60… 轉）
  GCS： 任何 namespace（^/[^/]+/名稱$）
  話題同時放進 publishers、subscribers（方向由實際的發布者、訂閱者決定），service 同時放進 servers、clients。

DDS 那一側：bridge 內部是 CycloneDDS，和機內的 Fast DDS 用標準 RTPS 互通。--dds-ip 指定它用哪張網卡（預設 WIRE_IP，
沒有就用 LOCAL_IPS 的第一個），**不能是 mesh 的網卡**，否則 DDS 又漏到 mesh 上。探索用 unicast（--dds-peers，預設
LOCAL_IPS＋PEER_IPS，和 docker/fastdds 的 Fast DDS 設定一樣），不用 multicast。
"""
import argparse
import json
import os
import re
import shutil
import sys

import yaml


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("swarm_zenoh")
    except Exception:   # 還沒安裝（原始碼直接跑、測試）
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def load_whitelist(path):
    with open(path, encoding="utf-8") as f:
        w = yaml.safe_load(f) or {}
    topics = list((w.get("to_gcs") or {}).get("topics") or []) + list((w.get("to_vehicle") or {}).get("topics") or [])
    services = list((w.get("to_gcs") or {}).get("services") or []) + \
        list((w.get("to_vehicle") or {}).get("services") or [])
    for n in topics + services:
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_/]*", n) or n.startswith("/"):
            raise ValueError(f"白名單的名稱要是 namespace 底下的相對名稱：{n!r}")
    freq = {k: float(v) for k, v in (w.get("max_frequency") or {}).items()}
    return topics, services, freq


def allow_rules(role, namespaces, topics, services):
    """白名單 → zenoh-bridge-ros2dds 的 allow（regex，比對完整的 ROS 名稱）"""
    if role == "vehicle":
        if not namespaces:
            raise ValueError("--role vehicle 要給 --ns")
        ns = "(" + "|".join(re.escape(n.strip("/")) for n in namespaces) + ")"
    else:
        ns = "[^/]+"

    def rule(names):
        return [f"^/{ns}/({'|'.join(re.escape(n) for n in names)})$"] if names else []
    return {
        "publishers": rule(topics),
        "subscribers": rule(topics),
        "service_servers": rule(services),
        "service_clients": rule(services),
        "action_servers": [],
        "action_clients": [],
    }


def zenoh_config(args, topics, services, freq):
    up = f"{args.transport}/{args.gcs_ip}:{args.port}"                       # 載具往上連（GCS 或中繼）
    listen_port = args.listen_port or args.port
    here = f"{args.transport}/{args.listen_ip}:{listen_port}"                # GCS、中繼在這裡聽
    router = args.role == "gcs" or args.relay
    cfg = {
        "mode": "router" if router else "client",
        # 載具（含中繼）：連不上時一直重試、不退出（GCS 晚開、斷線恢復）
        "connect": {"endpoints": [up] if args.role == "vehicle" else [],
                    "timeout_ms": -1, "exit_on_failure": False},
        "listen": {"endpoints": [here] if router else []},
        "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
        "plugins": {
            "ros2dds": {
                "domain": args.domain,
                "ros_localhost_only": False,
                "allow": allow_rules(args.role, args.ns, topics, services),
                "pub_max_frequencies": [f".*/{re.escape(n)}$={hz:g}" for n, hz in sorted(freq.items())],
            },
        },
    }
    return cfg


def cyclone_xml(dds_ip, peers):
    peer_xml = "".join(f'<Peer address="{p}"/>' for p in peers)
    return ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any">'
            f'<General><Interfaces><NetworkInterface address="{dds_ip}"/></Interfaces>'
            '<AllowMulticast>false</AllowMulticast></General>'
            '<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>63</MaxAutoParticipantIndex>'
            f'<Peers>{peer_xml}</Peers></Discovery>'
            '</Domain></CycloneDDS>\n')


def parse(argv):
    env = os.environ
    local = env.get("LOCAL_IPS", "").split()
    ap = argparse.ArgumentParser(description="GCS ↔ 載具的白名單 Zenoh bridge")
    ap.add_argument("--role", choices=["vehicle", "gcs"], required=True)
    ap.add_argument("--ns", nargs="*", default=[], help="載具：要轉的 namespace（uav1、v60…），可以多個")
    ap.add_argument("--gcs-ip", default=env.get("GCS_IP", "172.30.0.100"), help="載具：GCS 在 mesh 上的 IP")
    ap.add_argument("--listen-ip", default=env.get("MESH_IP", "0.0.0.0"), help="GCS：在哪個 IP 聽（mesh 的網卡）")
    ap.add_argument("--port", type=int, default=7447, help="往上連的埠（載具）、聽的埠（GCS）")
    ap.add_argument("--relay", action="store_true",
                    help="載具當中繼（router）：往上連 --gcs-ip、在 --listen-ip:--listen-port 聽，替連進來的載具轉送")
    ap.add_argument("--listen-port", type=int, default=None, help="中繼聽的埠（預設同 --port）")
    ap.add_argument("--transport", choices=["tcp", "udp", "quic"], default="tcp",
                    help="Zenoh 的連線（tcp 可靠；udp 不保證送達；quic 要憑證）")
    ap.add_argument("--domain", type=int, default=int(env.get("ROS_DOMAIN_ID", "0")))
    ap.add_argument("--dds-ip", default=env.get("WIRE_IP") or (local[0] if local else "127.0.0.1"),
                    help="bridge 的 DDS 用哪張網卡（機內那張，不能是 mesh）")
    ap.add_argument("--dds-peers", nargs="*", default=None,
                    help="DDS 探索的對象（預設 LOCAL_IPS＋PEER_IPS，沒有就是 --dds-ip）")
    ap.add_argument("--whitelist", default=None, help="預設 share/swarm_zenoh/config/whitelist.yaml")
    ap.add_argument("--out-dir", default=None, help="產生的設定放哪裡（預設 /tmp/swarm_zenoh_<角色>）")
    ap.add_argument("--dry-run", action="store_true", help="只印設定，不啟動")
    args = ap.parse_args(argv)
    if args.relay and args.role != "vehicle":
        ap.error("--relay 只用在 --role vehicle（GCS 本來就是 router）")
    if args.dds_peers is None:
        args.dds_peers = (local + env.get("PEER_IPS", "").split()) or [args.dds_ip]
    if (args.role == "gcs" or args.relay) and args.listen_ip == "0.0.0.0":
        print("[zenoh_bridge] 警告：GCS 在所有網卡上聽（沒有 MESH_IP，也沒給 --listen-ip）", file=sys.stderr)
    return args


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    topics, services, freq = load_whitelist(args.whitelist or os.path.join(share_dir(), "config", "whitelist.yaml"))
    cfg = zenoh_config(args, topics, services, freq)
    xml = cyclone_xml(args.dds_ip, args.dds_peers)
    out = args.out_dir or f"/tmp/swarm_zenoh_{args.role}"
    os.makedirs(out, exist_ok=True)
    cfg_path, xml_path = os.path.join(out, "zenoh.json5"), os.path.join(out, "cyclonedds.xml")
    with open(cfg_path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
    with open(xml_path, "w", encoding="utf-8") as f:
        f.write(xml)
    where = "、".join(([f"連 {cfg['connect']['endpoints'][0]}"] if cfg["connect"]["endpoints"] else []) +
                     ([f"聽 {cfg['listen']['endpoints'][0]}"] if cfg["listen"]["endpoints"] else []))
    print(f"[zenoh_bridge] {args.role}" + ("（中繼）" if args.relay else "") +
          (f"（{', '.join(args.ns)}）" if args.ns else "") +
          f"：{where}；DDS 網卡 {args.dds_ip}、探索 {' '.join(args.dds_peers)}、domain {args.domain}；"
          f"話題 {', '.join(topics)}；service {', '.join(services) or '無'}；設定 {cfg_path}", flush=True)
    if args.dry_run:
        print(json.dumps(cfg, ensure_ascii=False, indent=2))
        print(xml)
        return 0
    exe = shutil.which("zenoh-bridge-ros2dds")
    if not exe:
        print("[zenoh_bridge] 找不到 zenoh-bridge-ros2dds（要重建映像：docker/robot.Dockerfile；實機從 Eclipse 的套件庫裝）",
              file=sys.stderr)
        return 1
    os.environ["CYCLONEDDS_URI"] = "file://" + xml_path
    # 外掛的 domain 以環境變數 ROS_DOMAIN_ID 為準，設定檔的 domain 會被蓋掉（2026-10-21 selftest：容器的 42 蓋掉 90／91，
    # 兩個 bridge 都在 42、看不到測試節點）
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.execv(exe, [exe, "-c", cfg_path])


if __name__ == "__main__":
    sys.exit(main())
