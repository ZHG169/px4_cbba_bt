# swarm_zenoh 的設定產生器：白名單 → allow 規則、角色 → 連法、DDS 不上 mesh（2026-10-21）
import json
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import zenoh_bridge as zb  # noqa: E402

WL = os.path.join(os.path.dirname(__file__), "..", "config", "whitelist.yaml")


def gen(tmp_path, argv):
    zb.main(argv + ["--whitelist", WL, "--out-dir", str(tmp_path), "--dry-run"])
    cfg = json.load(open(tmp_path / "zenoh.json5", encoding="utf-8"))
    xml = open(tmp_path / "cyclonedds.xml", encoding="utf-8").read()
    return cfg, xml


def matches(rules, name):
    return any(re.fullmatch(r, name) for r in rules)


def test_vehicle_only_routes_its_namespaces(tmp_path):
    # 載具只轉自己的 namespace 的白名單話題；別台的、白名單外的（PX4）不轉
    cfg, _ = gen(tmp_path, ["--role", "vehicle", "--ns", "uav1", "v60", "--gcs-ip", "10.0.0.1", "--dds-ip", "172.30.1.11"])
    allow = cfg["plugins"]["ros2dds"]["allow"]
    assert matches(allow["publishers"], "/uav1/robot_state")
    assert matches(allow["subscribers"], "/uav1/emergency_stop")
    assert matches(allow["publishers"], "/v60/current_task")
    assert not matches(allow["publishers"], "/uav2/robot_state")
    assert not matches(allow["publishers"], "/uav1/fmu/out/vehicle_status")
    assert not matches(allow["publishers"], "/uav1/robot_state_extra")
    assert matches(allow["service_servers"], "/uav1/manual_override")
    assert allow["action_servers"] == [] and allow["action_clients"] == []


def test_gcs_routes_any_namespace(tmp_path):
    cfg, _ = gen(tmp_path, ["--role", "gcs", "--listen-ip", "10.0.0.1", "--dds-ip", "10.0.0.1"])
    allow = cfg["plugins"]["ros2dds"]["allow"]
    assert matches(allow["subscribers"], "/uav2/robot_state")
    assert matches(allow["publishers"], "/v60_1/emergency_stop")
    assert not matches(allow["subscribers"], "/uav2/camera/image_raw")


def test_roles_connect_router_and_client(tmp_path):
    # GCS 是 router（聽），載具是 client（連 GCS、連不上一直重試）；都不用 multicast scouting
    g, _ = gen(tmp_path, ["--role", "gcs", "--listen-ip", "10.0.0.1", "--dds-ip", "10.0.0.1"])
    assert g["mode"] == "router" and g["listen"]["endpoints"] == ["tcp/10.0.0.1:7447"]
    v, _ = gen(tmp_path, ["--role", "vehicle", "--ns", "uav1", "--gcs-ip", "10.0.0.1", "--dds-ip", "127.0.0.1"])
    assert v["mode"] == "client" and v["connect"]["endpoints"] == ["tcp/10.0.0.1:7447"]
    assert v["connect"]["exit_on_failure"] is False
    for c in (g, v):
        assert c["scouting"]["multicast"]["enabled"] is False


def test_relay_is_router_that_connects_up_and_listens(tmp_path):
    # 中繼（2026-10-25）：某一台當內網時要是 router——往上連 GCS、自己也聽，替連進來的載具雙向轉送（client 不轉送）
    r, _ = gen(tmp_path, ["--role", "vehicle", "--ns", "uav1", "--relay", "--gcs-ip", "10.0.0.1",
                          "--listen-ip", "10.0.0.11", "--dds-ip", "127.0.0.1"])
    assert r["mode"] == "router"
    assert r["connect"]["endpoints"] == ["tcp/10.0.0.1:7447"]
    assert r["listen"]["endpoints"] == ["tcp/10.0.0.11:7447"]
    allow = r["plugins"]["ros2dds"]["allow"]
    assert matches(allow["publishers"], "/uav1/robot_state")
    assert not matches(allow["publishers"], "/uav2/robot_state")     # 別台的 DDS 不經過中繼的 DDS，走 Zenoh
    leaf, _ = gen(tmp_path, ["--role", "vehicle", "--ns", "uav2", "--gcs-ip", "10.0.0.11", "--dds-ip", "127.0.0.1"])
    assert leaf["mode"] == "client" and leaf["connect"]["endpoints"] == ["tcp/10.0.0.11:7447"]


def test_relay_only_for_vehicle(tmp_path):
    try:
        gen(tmp_path, ["--role", "gcs", "--relay", "--listen-ip", "10.0.0.1", "--dds-ip", "127.0.0.1"])
    except SystemExit:
        return
    raise AssertionError("GCS 加 --relay 應該拒絕")


def test_robot_state_rate_limited(tmp_path):
    cfg, _ = gen(tmp_path, ["--role", "vehicle", "--ns", "uav1", "--dds-ip", "127.0.0.1"])
    assert ".*/robot_state$=2" in cfg["plugins"]["ros2dds"]["pub_max_frequencies"]


def test_dds_side_stays_on_given_interface_without_multicast(tmp_path):
    # DDS 那一側只用指定的網卡（機內那張）、unicast 探索：DDS 不上 mesh
    _, xml = gen(tmp_path, ["--role", "vehicle", "--ns", "uav1", "--dds-ip", "172.30.1.11",
                            "--dds-peers", "172.30.1.11", "172.30.1.2"])
    assert '<NetworkInterface address="172.30.1.11"/>' in xml
    assert "<AllowMulticast>false</AllowMulticast>" in xml
    assert '<Peer address="172.30.1.2"/>' in xml


def test_vehicle_needs_namespace(tmp_path):
    try:
        gen(tmp_path, ["--role", "vehicle", "--dds-ip", "127.0.0.1"])
    except ValueError:
        return
    raise AssertionError("沒給 --ns 應該拒絕")
