#include "apriltag_fire_detector/fire_detector.hpp"

#include <cmath>
#include <stdexcept>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>

namespace apriltag_fire_detector
{

TagDetector::TagDetector(double tag_size, float quad_decimate, int threads)
: tag_size_(tag_size)
{
  if (tag_size <= 0.0) {
    throw std::invalid_argument("tag_size must be > 0");
  }
  family_ = tag36h11_create();
  detector_ = apriltag_detector_create();
  if (family_ == nullptr || detector_ == nullptr) {
    throw std::runtime_error("apriltag detector create failed");
  }
  apriltag_detector_add_family(detector_, family_);
  detector_->quad_decimate = quad_decimate;
  detector_->nthreads = threads;
  detector_->refine_edges = true;
}

TagDetector::~TagDetector()
{
  if (detector_ != nullptr) {
    apriltag_detector_destroy(detector_);
  }
  if (family_ != nullptr) {
    tag36h11_destroy(family_);
  }
}

std::vector<TagDetection> TagDetector::detect(const std::uint8_t * gray, int width, int height,
  int stride, const CameraIntrinsics & camera) const
{
  // apriltag 只讀影像，image_u8_t 的 buf 不是 const 是 C 介面的限制
  image_u8_t im{width, height, stride, const_cast<std::uint8_t *>(gray)};
  zarray_t * detections = apriltag_detector_detect(detector_, &im);
  std::vector<TagDetection> out;
  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t * det = nullptr;
    zarray_get(detections, i, &det);
    apriltag_detection_info_t info{det, tag_size_, camera.fx, camera.fy, camera.cx, camera.cy};
    apriltag_pose_t pose{nullptr, nullptr};
    estimate_tag_pose(&info, &pose);
    TagDetection d;
    d.id = static_cast<std::uint32_t>(det->id);
    d.decision_margin = det->decision_margin;
    if (pose.t != nullptr) {
      d.camera = {MATD_EL(pose.t, 0, 0), MATD_EL(pose.t, 1, 0), MATD_EL(pose.t, 2, 0)};
      d.distance = std::sqrt(d.camera.x * d.camera.x + d.camera.y * d.camera.y + d.camera.z * d.camera.z);
      matd_destroy(pose.t);
    }
    if (pose.R != nullptr) {
      matd_destroy(pose.R);
    }
    out.push_back(d);
  }
  apriltag_detections_destroy(detections);
  return out;
}

Vec3d cameraToMap(const Vec3d & c, const Quat & q, const Vec3d & p_ned, const Vec3d & spawn_enu,
  const Vec3d & camera_offset_frd)
{
  const auto & M = kCameraToBody;
  const Vec3d b{
    M[0][0] * c.x + M[0][1] * c.y + M[0][2] * c.z + camera_offset_frd.x,
    M[1][0] * c.x + M[1][1] * c.y + M[1][2] * c.z + camera_offset_frd.y,
    M[2][0] * c.x + M[2][1] * c.y + M[2][2] * c.z + camera_offset_frd.z};
  // 機體 → NED：q 的旋轉矩陣
  const double w = q.w, x = q.x, y = q.y, z = q.z;
  const Vec3d n{
    (1 - 2 * (y * y + z * z)) * b.x + 2 * (x * y - w * z) * b.y + 2 * (x * z + w * y) * b.z,
    2 * (x * y + w * z) * b.x + (1 - 2 * (x * x + z * z)) * b.y + 2 * (y * z - w * x) * b.z,
    2 * (x * z - w * y) * b.x + 2 * (y * z + w * x) * b.y + (1 - 2 * (x * x + y * y)) * b.z};
  const Vec3d ned{p_ned.x + n.x, p_ned.y + n.y, p_ned.z + n.z};
  // NED → map ENU（以出生點為原點）
  return {spawn_enu.x + ned.y, spawn_enu.y + ned.x, spawn_enu.z - ned.z};
}

void ConfidenceTracker::addFrame(double t, const std::map<std::uint32_t, Vec3d> & hits)
{
  frames_.push_back(Frame{t, hits});
  while (!frames_.empty() && frames_.front().t < t - window_) {
    frames_.pop_front();
  }
}

std::optional<ConfidenceTracker::Stat> ConfidenceTracker::stat(std::uint32_t tag) const
{
  Stat s;
  for (const Frame & f : frames_) {
    ++s.frames;
    const auto it = f.hits.find(tag);
    if (it != f.hits.end()) {
      ++s.hits;
      s.mean.x += it->second.x;
      s.mean.y += it->second.y;
      s.mean.z += it->second.z;
    }
  }
  if (s.hits == 0) {
    return std::nullopt;
  }
  s.mean = {s.mean.x / s.hits, s.mean.y / s.hits, s.mean.z / s.hits};
  s.confidence = static_cast<double>(s.hits) / s.frames;
  return s;
}

}  // namespace apriltag_fire_detector
