#!/usr/bin/env python3
# ============================================================
#  依 .env 的 UAV_COUNT、DOG_COUNT 產生 docker-compose.yaml
#  start.sh / build.sh 會自動呼叫，一般不需要手動執行
#
#  每台無人機有自己的線材網路 sim_uavN_net，彼此之間「沒有線」，
#  只能透過 mesh_net 溝通 —— 這樣 tc 才會作用在無人機之間的所有流量上
#
#  狗（2026-11-09，DOG_COUNT）：每隻狗也有自己的容器 dogK（同一個 cbba-robot 映像，沒有 PX4，只跑狗的 cbba_node，
#  demo 時也可以跑伯宇的 BT），自己的 mesh 網卡；線材是**所有狗共用一條** sim_dog_net（給 sim 的 ros_bridge、方塊身體）。
#  為什麼共用：PX4 的 XRCE client 用自己的一段 XML 請 agent 建 DDS 參與者，不吃 fastdds/profile.xml.template，
#  遠端位址照 Fast DDS 預設只記 4 個；sim 每多一張線材網卡就多一個位址，超過 4 張時 agent 可能丟掉自己那條線，
#  sim 收不到那台的 PX4 話題（2026-11-09 demo：uav1、uav2 沒有位置）。所以 sim 的線材網卡 ≤ 4：無人機 N 張＋狗 1 張。
#  狗之間在這條線上不會互相探索（每隻的 DDS 只綁自己的線材 IP、只找 sim），CBBA 照舊走 mesh。
#  原本狗的 cbba_node 借 uav1 的容器跑，uav1 ↔ 狗走本機、不經過 tc，掉包量測要另外註明；有自己的容器後空地之間的
#  每一條連線都經過 tc。機號 50、51、52…，namespace v60、v60_1、v60_2（照伯宇的命名）
#
#  DDS（ROS 2）只在機內：uavN 的 Fast DDS 只綁線材網卡、只探索 sim（PX4 SITL、Gazebo 的相機），
#  mesh 上只有 cbba_node 的 UDP（協定版本 2）和 BT 的避碰廣播，tc 只影響機間協商。
#  DDS_OVER_MESH=1（.env）恢復 2026-10-10 以前的行為：uav、gcs 的 DDS 也經過 mesh（從 gcs 用 ros2 指令監看各台時用，
#  但 PX4 的話題也會被 tc 影響、吃 mesh 頻寬）
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
try:
    n_dog = int(env.get("DOG_COUNT", "0"))
except ValueError:
    sys.exit("[gen_compose] DOG_COUNT 必須是數字")
if not 0 <= n_dog <= 9:
    sys.exit("[gen_compose] DOG_COUNT 必須在 0~9 之間")
if n + (1 if n_dog else 0) > 4:
    sys.exit(f"[gen_compose] sim 的線材網卡會有 {n + (1 if n_dog else 0)} 張（無人機 {n}＋狗 {1 if n_dog else 0}），超過 4 張時"
             "PX4 的 XRCE agent 可能連不到 sim（Fast DDS 預設只記 4 個位址）：UAV_COUNT 有狗時最多 3")

base = env.get("NET_BASE", "172.30")
dds_over_mesh = env.get("DDS_OVER_MESH", "0") == "1"
uavs = range(1, n + 1)
dogs = range(1, n_dog + 1)

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


# 狗：線材全部共用 <base>.20.0/24（sim 是 .20.2、dogK 是 .20.(10 + k)），mesh <base>.0.(49 + k) → 機號 50 的狗是 .0.50
DOG_NET = f"{base}.20.0/24"
DOG_SIM_WIRE = f"{base}.20.2"


def dog_wire(k):
    return f"{base}.20.{10 + k}"


def dog_mesh(k):
    return f"{base}.0.{49 + k}"


def dog_id(k):
    return 49 + k


def dog_ns(k):
    return "v60" if k == 1 else f"v60_{k - 1}"


def ips(lst):
    return '"' + " ".join(lst) + '"'


out = []
w = out.append

w(f"""# ============================================================
#  !!! 自動產生的檔案，請勿手動修改 !!!
#  來源：gen_compose.py + .env（UAV_COUNT={n}、DOG_COUNT={n_dog}）
#  改 .env 後執行 ./start.sh 就會重新產生
#
#  sim ─┬─ sim_uav1_net ─ uav1 ─┐
#       ├─ sim_uav2_net ─ uav2 ─┼─ mesh_net (tc) ─ gcs
#       ├─ ...                  ┤
#       └─ sim_dog_net ── dog1…K ┘  （DOG_COUNT > 0 時，所有狗共用一條線）
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
      LOCAL_IPS: {ips([sim_wire(i) for i in uavs] + ([DOG_SIM_WIRE] if n_dog else []))}
      PEER_IPS: {ips([uav_wire(i) for i in uavs] + [dog_wire(k) for k in dogs])}
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
if n_dog:
    w(f"""      sim_dog_net:
        ipv4_address: {DOG_SIM_WIRE}
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
      LOCAL_IPS: {ips([uav_wire(i), uav_mesh(i)] if dds_over_mesh else [uav_wire(i)])}
      PEER_IPS: {ips([sim_wire(i)] + others + [gcs_mesh] if dds_over_mesh else [sim_wire(i)])}
    volumes:
      - ..:/home/ncrl/CBBA_BT
      - /tmp/cbba_locks:/run/cbba
    networks:
      sim_uav{i}_net:
        ipv4_address: {uav_wire(i)}
      mesh_net:
        ipv4_address: {uav_mesh(i)}
""")

for k in dogs:
    w(f"""
  # ---------------------------------------------------------- dog{k}（機號 {dog_id(k)}、/{dog_ns(k)}）
  dog{k}:
    <<: *common
    image: cbba-robot:jazzy
    container_name: cbba_dog{k}
    hostname: dog{k}
    cap_add:
      - NET_ADMIN
    environment:
      <<: *common-env
      CBBA_ROLE: dog{k}
      DOG_ID: "{dog_id(k)}"
      DOG_NS: {dog_ns(k)}
      WIRE_IP: {dog_wire(k)}
      MESH_IP: {dog_mesh(k)}
      LOCAL_IPS: {ips([dog_wire(k)])}
      PEER_IPS: {ips([DOG_SIM_WIRE])}
    volumes:
      - ..:/home/ncrl/CBBA_BT
      - /tmp/cbba_locks:/run/cbba
    networks:
      sim_dog_net:
        ipv4_address: {dog_wire(k)}
      mesh_net:
        ipv4_address: {dog_mesh(k)}
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
      PEER_IPS: {ips([uav_mesh(i) for i in uavs] if dds_over_mesh else [])}
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

if n_dog:
    w(f"""  sim_dog_net:
    driver: bridge
    ipam:
      config:
        - subnet: {DOG_NET}
""")

OUT_FILE.write_text("".join(out), encoding="utf-8")
print(f"[gen_compose] UAV_COUNT={n}，DDS {'也經過 mesh' if dds_over_mesh else '只在機內（不經過 mesh）'} -> {OUT_FILE.name}")
for i in uavs:
    print(f"[gen_compose]   uav{i}: 線 {uav_wire(i)}  mesh {uav_mesh(i)}")
for k in dogs:
    print(f"[gen_compose]   dog{k}: 線 {dog_wire(k)}  mesh {dog_mesh(k)}  機號 {dog_id(k)}  /{dog_ns(k)}")
print(f"[gen_compose]   gcs : mesh {gcs_mesh}")
