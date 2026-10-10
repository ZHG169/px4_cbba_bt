// AprilTag 火情偵測的核心（2026-10-09）：不依賴 ROS，節點（fire_detector_node）把影像、相機參數、PX4 姿態接進來。
//
//   TagDetector        apriltag C 函式庫（tag36h11）：灰階影像 → 每個 tag 的編號、decision_margin、
//                      相機座標的位置（estimate_tag_pose，要知道 tag 的實際邊長）
//   cameraToMap        相機座標 → map ENU：相機（光學座標：x 右、y 下、z 往前）→ 機體 FRD → PX4 local NED → map ENU
//   ConfidenceTracker  每個 tag 的 confidence = 最近 window_sec 內處理過的影像中，有偵測到它（margin ≥ min_margin）的比例
//
// confidence 的定義：apriltag 本身沒有「信心度」，decision_margin 是單張的解碼品質、不是機率。
// 週計畫的「信心度 > 0.85」用「連續影像裡穩定看到」來表示：偶爾誤認一張、或飛過時只掃到幾張都不會成立。
//
// 相機的安裝：PX4 的 x500_mono_cam_down（mono_cam 繞機體 y 軸轉 90°、在機體上方 0.1 m）：
//   光學 x（影像右）= 機體右（FRD 的 +y）、光學 y（影像下）= 機體後（FRD 的 −x）、光學 z（視線）= 機體下（FRD 的 +z）
// 換相機或裝法時改 kCameraToBody（或之後改成參數）。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <utility>
#include <vector>

// apriltag 的型別（不在這裡 include C 標頭，避免它的巨集影響用到這個檔案的程式）
struct apriltag_detector;
struct apriltag_family;

namespace apriltag_fire_detector
{

struct Vec3d
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Hamilton (w, x, y, z)：機體 FRD → NED（PX4 vehicle_odometry 的 q）
struct Quat
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

struct TagDetection
{
  std::uint32_t id{0};
  double decision_margin{0.0};
  Vec3d camera;           // tag 中心，相機的光學座標（公尺）
  double distance{0.0};
};

// 相機的光學座標 → 機體 FRD（列 = FRD 的 x、y、z）
constexpr std::array<std::array<double, 3>, 3> kCameraToBody{{
  {{0.0, -1.0, 0.0}},
  {{1.0, 0.0, 0.0}},
  {{0.0, 0.0, 1.0}},
}};

class TagDetector
{
public:
  // tag_size：黑框的邊長（公尺）；quad_decimate：先縮小幾倍再找四邊形（快，但遠的 tag 會漏）
  TagDetector(double tag_size, float quad_decimate, int threads);
  ~TagDetector();
  TagDetector(const TagDetector &) = delete;
  TagDetector & operator=(const TagDetector &) = delete;

  std::vector<TagDetection> detect(const std::uint8_t * gray, int width, int height, int stride,
    const CameraIntrinsics & camera) const;

private:
  double tag_size_;
  apriltag_detector * detector_{nullptr};
  apriltag_family * family_{nullptr};
};

// tag 的位置：相機座標 → map ENU
//   p_ned：PX4 local 位置（NED，以出生點為原點）；spawn_enu：出生點（map ENU）；camera_offset_frd：相機在機體上的位置
Vec3d cameraToMap(const Vec3d & camera, const Quat & q_body_to_ned, const Vec3d & p_ned,
  const Vec3d & spawn_enu, const Vec3d & camera_offset_frd);

class ConfidenceTracker
{
public:
  explicit ConfidenceTracker(double window_sec)
  : window_(window_sec) {}

  // 每處理一張影像呼叫一次（沒看到任何 tag 也要呼叫，分母才對）：hits = 這張看到的 tag 與它的 map 位置
  void addFrame(double t, const std::map<std::uint32_t, Vec3d> & hits);

  struct Stat
  {
    double confidence{0.0};
    int frames{0};           // 視窗內處理過的影像數
    int hits{0};
    Vec3d mean;              // 視窗內看到時的平均位置
  };
  // 視窗內沒看到過這個 tag 時回傳空
  std::optional<Stat> stat(std::uint32_t tag) const;

private:
  struct Frame
  {
    double t{0.0};
    std::map<std::uint32_t, Vec3d> hits;
  };
  double window_;
  std::deque<Frame> frames_;
};

}  // namespace apriltag_fire_detector
