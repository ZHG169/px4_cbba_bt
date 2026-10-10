// AprilTag 火情偵測節點（2026-10-09）：每台無人機的機上電腦各跑一個。
//
//   <ns>/camera/image_raw、<ns>/camera/camera_info   下視相機（模擬由 sim 容器的 gz_bridge.sh 送來）
//   <px4_ns>/fmu/out/vehicle_odometry                姿態與位置（相機座標 → map 要用）
//   → <ns>/fire_detection（swarm_interfaces/FireDetection）：每張影像看到的每個 tag 各一則
//
// 只算 margin ≥ min_margin 的偵測；confidence 的定義見 fire_detector.hpp。火警與否由 BT 判斷（expected_tag_id、
// min_confidence），這裡不知道哪個 tag 是火警。
// 影像和姿態不同步（影像是模擬時間、PX4 是開機時間），用最新的姿態：BT 是在任務點懸停時才判斷，誤差很小。
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <swarm_interfaces/msg/fire_detection.hpp>

#include "apriltag_fire_detector/fire_detector.hpp"

using px4_msgs::msg::VehicleOdometry;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using FireDetectionMsg = swarm_interfaces::msg::FireDetection;

namespace apriltag_fire_detector
{

namespace
{
std::string envOr(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr && *v != '\0' ? v : fallback;
}

double steadyNow()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// 轉灰階；不支援的編碼回傳 false
bool toGray(const Image & img, std::vector<std::uint8_t> & gray)
{
  const std::size_t n = static_cast<std::size_t>(img.width) * img.height;
  gray.resize(n);
  int channels = 0;
  bool bgr = false;
  if (img.encoding == "mono8") {
    channels = 1;
  } else if (img.encoding == "rgb8") {
    channels = 3;
  } else if (img.encoding == "bgr8") {
    channels = 3;
    bgr = true;
  } else if (img.encoding == "rgba8") {
    channels = 4;
  } else if (img.encoding == "bgra8") {
    channels = 4;
    bgr = true;
  } else {
    return false;
  }
  if (img.step < img.width * static_cast<std::uint32_t>(channels) ||
    img.data.size() < static_cast<std::size_t>(img.step) * img.height)
  {
    return false;
  }
  for (std::uint32_t y = 0; y < img.height; ++y) {
    const std::uint8_t * row = img.data.data() + static_cast<std::size_t>(y) * img.step;
    for (std::uint32_t x = 0; x < img.width; ++x) {
      const std::uint8_t * p = row + static_cast<std::size_t>(x) * channels;
      if (channels == 1) {
        gray[y * img.width + x] = p[0];
        continue;
      }
      const int r = bgr ? p[2] : p[0];
      const int g = p[1];
      const int b = bgr ? p[0] : p[2];
      gray[y * img.width + x] = static_cast<std::uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
    }
  }
  return true;
}
}  // namespace

class FireDetectorNode : public rclcpp::Node
{
public:
  FireDetectorNode()
  : Node("apriltag_fire_detector")
  {
    const int agent_id = declare_parameter<int>("agent_id", std::stoi(envOr("UAV_ID", "1")));
    const std::string px4_ns = declare_parameter<std::string>("px4_ns", envOr("UAV_NS", ""));
    const std::string ns = declare_parameter<std::string>("ns", envOr("UAV_NS", "uav" + std::to_string(agent_id)));
    const double tag_size = declare_parameter<double>("tag_size", 0.8);   // docker/gz/models/tag36h11_*：黑框 0.8 m
    const double decimate = declare_parameter<double>("quad_decimate", 1.0);
    const int threads = declare_parameter<int>("threads", 2);
    min_margin_ = declare_parameter<double>("min_margin", 30.0);
    window_sec_ = declare_parameter<double>("window_sec", 1.0);
    odom_timeout_ = declare_parameter<double>("odometry_timeout", 0.5);
    const double spacing = std::stod(envOr("UAV_SPAWN_SPACING", "2.0"));
    const auto spawn = declare_parameter<std::vector<double>>("spawn_enu", {0.0, (agent_id - 1) * spacing, 0.0});
    const auto offset = declare_parameter<std::vector<double>>("camera_offset_frd", {0.0, 0.0, -0.1});
    if (spawn.size() != 3 || offset.size() != 3) {
      throw std::invalid_argument("spawn_enu, camera_offset_frd must have 3 elements");
    }
    spawn_ = {spawn[0], spawn[1], spawn[2]};
    offset_ = {offset[0], offset[1], offset[2]};

    detector_ = std::make_unique<TagDetector>(tag_size, static_cast<float>(decimate), threads);
    tracker_ = std::make_unique<ConfidenceTracker>(window_sec_);

    const std::string prefix = "/" + ns;
    const std::string fmu = (px4_ns.empty() ? "" : "/" + px4_ns) + "/fmu";
    pub_ = create_publisher<FireDetectionMsg>(prefix + "/fire_detection", rclcpp::QoS(20));
    info_sub_ = create_subscription<CameraInfo>(prefix + "/camera/camera_info", rclcpp::SensorDataQoS(),
        [this](CameraInfo::ConstSharedPtr m) {
          if (m->k[0] > 0.0) {
            camera_ = {m->k[0], m->k[4], m->k[2], m->k[5]};
            have_camera_ = true;
          }
        });
    odom_sub_ = create_subscription<VehicleOdometry>(fmu + "/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        [this](VehicleOdometry::ConstSharedPtr m) {
          if (std::isfinite(m->q[0]) && std::isfinite(m->position[0])) {
            odom_ = *m;
            odom_rx_ = steadyNow();
          }
        });
    image_sub_ = create_subscription<Image>(prefix + "/camera/image_raw", rclcpp::SensorDataQoS(),
        [this](Image::ConstSharedPtr m) {onImage(*m);});
    RCLCPP_INFO(get_logger(), "uav%d：影像 %s/camera/image_raw → %s/fire_detection；tag36h11、邊長 %.2f m、"
      "min_margin %.0f、視窗 %.1f s", agent_id, prefix.c_str(), prefix.c_str(), tag_size, min_margin_, window_sec_);
  }

private:
  void onImage(const Image & img)
  {
    const double now = steadyNow();
    if (!have_camera_ || now - odom_rx_ > odom_timeout_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "等待 camera_info、vehicle_odometry");
      return;
    }
    if (!toGray(img, gray_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "不支援的影像編碼 %s", img.encoding.c_str());
      return;
    }
    const Quat q{odom_.q[0], odom_.q[1], odom_.q[2], odom_.q[3]};
    const Vec3d p{odom_.position[0], odom_.position[1], odom_.position[2]};
    std::map<std::uint32_t, Vec3d> hits;
    std::map<std::uint32_t, TagDetection> best;
    for (const TagDetection & d : detector_->detect(gray_.data(), static_cast<int>(img.width),
      static_cast<int>(img.height), static_cast<int>(img.width), camera_))
    {
      if (d.decision_margin < min_margin_) {
        continue;
      }
      hits[d.id] = cameraToMap(d.camera, q, p, spawn_, offset_);
      best[d.id] = d;
    }
    tracker_->addFrame(now, hits);
    for (const auto & [id, d] : best) {
      const auto s = tracker_->stat(id);
      if (!s) {
        continue;
      }
      FireDetectionMsg msg;
      msg.header.stamp = img.header.stamp;
      msg.header.frame_id = "map";
      msg.tag_id = id;
      msg.confidence = static_cast<float>(s->confidence);
      msg.decision_margin = static_cast<float>(d.decision_margin);
      msg.frames = static_cast<std::uint16_t>(s->frames);
      msg.position.x = s->mean.x;
      msg.position.y = s->mean.y;
      msg.position.z = s->mean.z;
      msg.distance = static_cast<float>(d.distance);
      pub_->publish(msg);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "tag %u：confidence %.2f（%d/%d 張）、margin %.0f、距離 %.1f m、map (%.2f, %.2f, %.2f)",
        id, s->confidence, s->hits, s->frames, d.decision_margin, d.distance, s->mean.x, s->mean.y, s->mean.z);
    }
  }

  std::unique_ptr<TagDetector> detector_;
  std::unique_ptr<ConfidenceTracker> tracker_;
  double min_margin_{30.0};
  double window_sec_{1.0};
  double odom_timeout_{0.5};
  Vec3d spawn_;
  Vec3d offset_;
  CameraIntrinsics camera_;
  bool have_camera_{false};
  VehicleOdometry odom_{};
  double odom_rx_{-1e9};
  std::vector<std::uint8_t> gray_;

  rclcpp::Publisher<FireDetectionMsg>::SharedPtr pub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
};

}  // namespace apriltag_fire_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<apriltag_fire_detector::FireDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
