#!/usr/bin/env python3
# ============================================================
#  依 .env 的 UAV_COUNT 產生 docker-compose.yaml
#  start.sh / build.sh 會自動呼叫，一般不需要手動執行
#
#  每台無人機有自己的線材網路 sim_uavN_net，彼此之間「沒有線」，
#  只能透過 mesh_net 溝通 —— 這樣 tc 才會作用在無人機之間的所有流量上
# ============================================================
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
ENV_FILE = HERE / ".env"
OUT_FILE = HERE / "docker-compose.yaml"


def load_env(path):
    env = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, val = line.split("=", 1)
        env[key.strip()] = val.split("#", 1)[0].strip().strip('"').strip("'")
    return env


env = load_env(ENV_FILE)
try:
    n = int(env.get("UAV_COUNT", "1"))
except ValueError:
    sys.exit("[gen_compose] UAV_COUNT 必須是數字")
if not 1 <= n <= 9:
    sys.exit("[gen_compose] UAV_COUNT 必須在 1~9 之間")

base = env.get("NET_BASE", "172.30")
uavs = range(1, n + 1)

mesh_subnet = f"{base}.0.0/24"
gcs_mesh = f"{base}.0.100"


def wire_subnet(i):
    return f"{base}.{i}.0/24"


def sim_wire(i):
    return f"{base}.{i}.2"


def uav_wire(i):
    return f"{base}.{i}.11"


def uav_mesh(i):
    return f"{base}.0.{10 + i}"


def ips(lst):
    return '"' + " ".join(lst) + '"'


out = []
w = out.append

w(f"""# ============================================================
#  !!! 自動產生的檔案，請勿手動修改 !!!
#  來源：gen_compose.py + .env（UAV_COUNT={n}）
#  改 .env 後執行 ./start.sh 就會重新產生
#
#  sim ─┬─ sim_uav1_net ─ uav1 ─┐
#       ├─ sim_uav2_net ─ uav2 ─┼─ mesh_net (tc) ─ gcs
#       └─ ...                  ┘
# ============================================================
name: ${{COMPOSE_PROJECT_NAME:-cbba_bt}}

x-common: &common
  env_file: .env
  stdin_open: true
  tty: true
  init: true
  working_dir: /home/ncrl/CBBA_BT
  command: ["sleep", "infinity"]

x-common-env: &common-env
  HOST_UID: ${{HOST_UID:-1000}}
  HOST_GID: ${{HOST_GID:-1000}}
  ROS_DOMAIN_ID: ${{ROS_DOMAIN_ID:-42}}
  RMW_IMPLEMENTATION: rmw_fastrtps_cpp
  FASTRTPS_DEFAULT_PROFILES_FILE: /tmp/cbba_fastdds.xml
  FASTDDS_DEFAULT_PROFILES_FILE: /tmp/cbba_fastdds.xml
  # cbba_node 的程序鎖（同一機號只能跑一個）：所有容器共用主機的 /tmp/cbba_locks
  CBBA_LOCK_DIR: /run/cbba

x-gui-env: &gui-env
  DISPLAY: ${{DISPLAY:-:0}}
  QT_X11_NO_MITSHM: "1"

services:
  # ---------------------------------------------------------- sim
  sim:
    <<: *common
    build:
      context: ..
      dockerfile: docker/sim.Dockerfile
      args:
        BASE_IMAGE: cbba-base:jazzy
        PX4_VERSION: ${{PX4_VERSION}}
    image: cbba-sim:jazzy
    container_name: cbba_sim
    hostname: sim
    environment:
      <<: [*common-env, *gui-env]
      CBBA_ROLE: sim
      GZ_IP: 127.0.0.1
      LOCAL_IPS: {ips(sim_wire(i) for i in uavs)}
      PEER_IPS: {ips(uav_wire(i) for i in uavs)}
      UAV_AGENT_IPS: {ips(uav_wire(i) for i in uavs)}
    volumes:
      - ..:/home/ncrl/CBBA_BT
      - /tmp/cbba_locks:/run/cbba
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    networks:
""")
for i in uavs:
    w(f"""      sim_uav{i}_net:
        ipv4_address: {sim_wire(i)}
""")

for i in uavs:
    others = [uav_mesh(j) for j in uavs if j != i]
    build = """    build:
      context: ..
      dockerfile: docker/robot.Dockerfile
      args:
        BASE_IMAGE: cbba-base:jazzy
        XRCE_AGENT_VERSION: ${XRCE_AGENT_VERSION}
        WITH_GUI: "0"
""" if i == 1 else ""
    w(f"""
  # ---------------------------------------------------------- uav{i}
  uav{i}:
    <<: *common
{build}    image: cbba-robot:jazzy
    container_name: cbba_uav{i}
    hostname: uav{i}
    cap_add:
      - NET_ADMIN
    environment:
      <<: *common-env
      CBBA_ROLE: uav{i}
      CBBA_WS_TAG: uav
      UAV_ID: "{i}"
      UAV_NS: uav{i}
      WIRE_IP: {uav_wire(i)}
      MESH_IP: {uav_mesh(i)}
      LOCAL_IPS: {ips([uav_wire(i), uav_mesh(i)])}
      PEER_IPS: {ips([sim_wire(i)] + others + [gcs_mesh])}
    volumes:
      - ..:/home/ncrl/CBBA_BT
      - /tmp/cbba_locks:/run/cbba
    networks:
      sim_uav{i}_net:
        ipv4_address: {uav_wire(i)}
      mesh_net:
        ipv4_address: {uav_mesh(i)}
""")

w(f"""
  # ---------------------------------------------------------- gcs
  gcs:
    <<: *common
    build:
      context: ..
      dockerfile: docker/robot.Dockerfile
      args:
        BASE_IMAGE: cbba-base:jazzy
        XRCE_AGENT_VERSION: ${{XRCE_AGENT_VERSION}}
        WITH_GUI: "1"
    image: cbba-gcs:jazzy
    container_name: cbba_gcs
    hostname: gcs
    cap_add:
      - NET_ADMIN
    environment:
      <<: [*common-env, *gui-env]
      CBBA_ROLE: gcs
      MESH_IP: {gcs_mesh}
      LOCAL_IPS: {ips([gcs_mesh])}
      PEER_IPS: {ips(uav_mesh(i) for i in uavs)}
    volumes:
      - ..:/home/ncrl/CBBA_BT
      - /tmp/cbba_locks:/run/cbba
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    networks:
      mesh_net:
        ipv4_address: {gcs_mesh}

networks:
  mesh_net:
    driver: bridge
    ipam:
      config:
        - subnet: {mesh_subnet}
""")
for i in uavs:
    w(f"""  sim_uav{i}_net:
    driver: bridge
    ipam:
      config:
        - subnet: {wire_subnet(i)}
""")

OUT_FILE.write_text("".join(out), encoding="utf-8")
print(f"[gen_compose] UAV_COUNT={n} -> {OUT_FILE.name}")
for i in uavs:
    print(f"[gen_compose]   uav{i}: 線 {uav_wire(i)}  mesh {uav_mesh(i)}")
print(f"[gen_compose]   gcs : mesh {gcs_mesh}")
