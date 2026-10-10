// AprilTag 火情偵測的核心：合成影像偵測（編號、距離）、相機座標 → map、confidence 視窗
#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <vector>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include "apriltag_fire_detector/fire_detector.hpp"

using namespace apriltag_fire_detector;

namespace
{
// 正對相機（光軸垂直於 tag）的合成影像：tag 中心在像素 (u, v)，每格 cell 像素（含外圍 1 格白邊共 10 格）
std::vector<std::uint8_t> renderTag(int id, int width, int height, int u, int v, int cell)
{
  std::vector<std::uint8_t> img(static_cast<std::size_t>(width) * height, 180);   // 灰色地面
  apriltag_family_t * tf = tag36h11_create();
  image_u8_t * tag = apriltag_to_image(tf, id);
  const int size = tag->width * cell;
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      const int px = u - size / 2 + x;
      const int py = v - size / 2 + y;
      if (px >= 0 && px < width && py >= 0 && py < height) {
        img[py * width + px] = tag->buf[(y / cell) * tag->stride + x / cell];
      }
    }
  }
  image_u8_destroy(tag);
  tag36h11_destroy(tf);
  return img;
}
}  // namespace

TEST(FireDetector, DetectsSyntheticTagWithDistance)
{
  // fx = 300、tag 黑框 0.8 m、距離 5 m → 黑框 48 像素（每格 6 像素）；中心偏右 60 像素 → x = 60 × 5 / 300 = 1 m
  const CameraIntrinsics cam{300.0, 300.0, 320.0, 240.0};
  const auto img = renderTag(101, 640, 480, 380, 240, 6);
  TagDetector detector(0.8, 1.0f, 1);
  const auto dets = detector.detect(img.data(), 640, 480, 640, cam);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_EQ(dets[0].id, 101u);
  EXPECT_GT(dets[0].decision_margin, 30.0);
  EXPECT_NEAR(dets[0].camera.z, 5.0, 0.15);
  EXPECT_NEAR(dets[0].camera.x, 1.0, 0.1);
  EXPECT_NEAR(dets[0].camera.y, 0.0, 0.1);
}

TEST(FireDetector, NoTagNoDetection)
{
  const std::vector<std::uint8_t> img(640 * 480, 180);
  TagDetector detector(0.8, 1.0f, 1);
  EXPECT_TRUE(detector.detect(img.data(), 640, 480, 640, {300, 300, 320, 240}).empty());
}

TEST(FireDetector, CameraToMapLevelNorth)
{
  // 水平、機頭朝北：影像右 = 東、影像下 = 南。tag 在相機 (1, 2, 5)，飛機在 NED (10, 3, −5)，出生點 map (0, 2, 0)
  //   機體 FRD = (−2, 1, 5) + 相機位置 (0, 0, −0.1) → NED (8, 4, −0.1) → map (4, 10, 0.1)
  const Vec3d m = cameraToMap({1, 2, 5}, Quat{}, {10, 3, -5}, {0, 2, 0}, {0, 0, -0.1});
  EXPECT_NEAR(m.x, 4.0, 1e-9);
  EXPECT_NEAR(m.y, 10.0, 1e-9);
  EXPECT_NEAR(m.z, 0.1, 1e-9);
}

TEST(FireDetector, CameraToMapYawEast)
{
  // 機頭朝東（yaw 90°）：機體 FRD (−2, 1, 4.9) → NED (−1, −2, 4.9) → tag NED (9, 1, −0.1) → map (1, 11, 0.1)
  const double h = std::sqrt(0.5);
  const Vec3d m = cameraToMap({1, 2, 5}, Quat{h, 0, 0, h}, {10, 3, -5}, {0, 2, 0}, {0, 0, -0.1});
  EXPECT_NEAR(m.x, 1.0, 1e-9);
  EXPECT_NEAR(m.y, 11.0, 1e-9);
  EXPECT_NEAR(m.z, 0.1, 1e-9);
}

TEST(FireDetector, ConfidenceIsHitRatioInWindow)
{
  // 1 s 視窗、10 張裡 9 張看到 → 0.9；平均位置只算看到的那幾張；超過視窗的舊影像不算
  ConfidenceTracker tracker(1.0);
  for (int i = 0; i < 10; ++i) {
    std::map<std::uint32_t, Vec3d> hits;
    if (i != 3) {
      hits[101] = {10.0 + (i % 2 == 0 ? 0.1 : -0.1), 5.0, 0.3};
    }
    tracker.addFrame(i * 0.1, hits);
  }
  auto s = tracker.stat(101);
  ASSERT_TRUE(s);
  EXPECT_EQ(s->frames, 10);
  EXPECT_EQ(s->hits, 9);
  EXPECT_NEAR(s->confidence, 0.9, 1e-9);
  EXPECT_NEAR(s->mean.y, 5.0, 1e-9);
  EXPECT_FALSE(tracker.stat(7));

  // 之後 1.5 s 都沒看到：視窗內沒有 101
  for (int i = 10; i < 25; ++i) {
    tracker.addFrame(i * 0.1, {});
  }
  EXPECT_FALSE(tracker.stat(101));
}

TEST(FireDetector, BriefGlimpseHasLowConfidence)
{
  // 飛過時只掃到 2 張（15 Hz、1 s 視窗）→ confidence 約 0.13，不會成立
  ConfidenceTracker tracker(1.0);
  for (int i = 0; i < 15; ++i) {
    std::map<std::uint32_t, Vec3d> hits;
    if (i == 6 || i == 7) {
      hits[101] = {};
    }
    tracker.addFrame(i / 15.0, hits);
  }
  const auto s = tracker.stat(101);
  ASSERT_TRUE(s);
  EXPECT_LT(s->confidence, 0.2);
}
