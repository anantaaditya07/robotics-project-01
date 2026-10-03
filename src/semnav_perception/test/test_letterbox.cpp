// Unit tests for letterbox.hpp (architecture 7.1).
#include <gtest/gtest.h>

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "semnav_perception/letterbox.hpp"

namespace sp = semnav_perception;

namespace {

constexpr int kInputSize = 640;
constexpr float kRoundTripTolPx = 1.0F;

struct SizeCase {
  int w;
  int h;
};

class LetterboxSizes : public ::testing::TestWithParam<SizeCase> {};

// Forward-map a source box with the LetterboxInfo, unletterbox it, and expect
// the original box back within 1 px.
TEST_P(LetterboxSizes, AnalyticRoundTrip) {
  const auto [w, h] = GetParam();
  cv::Mat src(h, w, CV_8UC3, cv::Scalar(10, 20, 30));
  cv::Mat lb;
  const sp::LetterboxInfo info = sp::letterbox(src, kInputSize, lb);

  EXPECT_EQ(info.src_w, w);
  EXPECT_EQ(info.src_h, h);
  EXPECT_EQ(info.size, kInputSize);
  EXPECT_EQ(lb.rows, kInputSize);
  EXPECT_EQ(lb.cols, kInputSize);
  EXPECT_EQ(lb.type(), CV_8UC3);

  const cv::Rect2f src_box(0.2F * w, 0.3F * h, 0.25F * w, 0.4F * h);
  const cv::Rect2f in_box(src_box.x * info.scale + info.pad_x, src_box.y * info.scale + info.pad_y,
                          src_box.width * info.scale, src_box.height * info.scale);
  const cv::Rect2f back = sp::unletterbox(in_box, info);
  EXPECT_NEAR(back.x, src_box.x, kRoundTripTolPx);
  EXPECT_NEAR(back.y, src_box.y, kRoundTripTolPx);
  EXPECT_NEAR(back.x + back.width, src_box.x + src_box.width, kRoundTripTolPx);
  EXPECT_NEAR(back.y + back.height, src_box.y + src_box.height, kRoundTripTolPx);
}

// Paint a white box into the source, letterbox the real image, find the box in
// the letterboxed pixels and map it back. Edge error is at most one input pixel
// after resampling, i.e. 1 / scale source pixels, plus the 1 px budget.
TEST_P(LetterboxSizes, ImageRoundTrip) {
  const auto [w, h] = GetParam();
  cv::Mat src(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  const cv::Rect box(w / 5, h / 4, w / 3, h / 3);
  cv::rectangle(src, box, cv::Scalar(255, 255, 255), cv::FILLED);

  cv::Mat lb;
  const sp::LetterboxInfo info = sp::letterbox(src, kInputSize, lb);

  cv::Mat gray;
  cv::Mat mask;
  cv::cvtColor(lb, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, mask, 200, 255, cv::THRESH_BINARY);
  const cv::Rect found = cv::boundingRect(mask);
  ASSERT_GT(found.area(), 0);

  const cv::Rect2f back = sp::unletterbox(cv::Rect2f(found), info);
  const float tol = kRoundTripTolPx + 1.0F / info.scale;
  EXPECT_NEAR(back.x, box.x, tol);
  EXPECT_NEAR(back.y, box.y, tol);
  EXPECT_NEAR(back.x + back.width, box.x + box.width, tol);
  EXPECT_NEAR(back.y + back.height, box.y + box.height, tol);
}

// Geometry: content is centred, padding uses the pad value, rounding matches
// the Python reference (round(w * scale), (size - new) // 2).
TEST_P(LetterboxSizes, PaddingAndGeometry) {
  const auto [w, h] = GetParam();
  cv::Mat src(h, w, CV_8UC3, cv::Scalar(1, 2, 3));
  cv::Mat lb;
  const sp::LetterboxInfo info = sp::letterbox(src, kInputSize, lb);

  const double scale =
      std::min(static_cast<double>(kInputSize) / w, static_cast<double>(kInputSize) / h);
  const int new_w = static_cast<int>(std::nearbyint(w * scale));
  const int new_h = static_cast<int>(std::nearbyint(h * scale));
  EXPECT_FLOAT_EQ(info.scale, static_cast<float>(scale));
  EXPECT_EQ(info.pad_x, (kInputSize - new_w) / 2);
  EXPECT_EQ(info.pad_y, (kInputSize - new_h) / 2);

  // Content pixel in the centre.
  EXPECT_EQ(lb.at<cv::Vec3b>(kInputSize / 2, kInputSize / 2), cv::Vec3b(1, 2, 3));
  // Padding pixels (if any) use the default pad value.
  const cv::Vec3b pad(sp::kDefaultPadValue, sp::kDefaultPadValue, sp::kDefaultPadValue);
  if (info.pad_x > 0) {
    EXPECT_EQ(lb.at<cv::Vec3b>(kInputSize / 2, 0), pad);
    EXPECT_EQ(lb.at<cv::Vec3b>(kInputSize / 2, kInputSize - 1), pad);
  }
  if (info.pad_y > 0) {
    EXPECT_EQ(lb.at<cv::Vec3b>(0, kInputSize / 2), pad);
    EXPECT_EQ(lb.at<cv::Vec3b>(kInputSize - 1, kInputSize / 2), pad);
  }
}

INSTANTIATE_TEST_SUITE_P(Sizes, LetterboxSizes,
                         ::testing::Values(SizeCase{640, 480}, SizeCase{480, 640},
                                           SizeCase{1920, 1080}, SizeCase{640, 640}));

TEST(Letterbox, KnownGeometry640x480) {
  cv::Mat src(480, 640, CV_8UC3, cv::Scalar::all(0));
  cv::Mat lb;
  const sp::LetterboxInfo info = sp::letterbox(src, kInputSize, lb);
  EXPECT_FLOAT_EQ(info.scale, 1.0F);
  EXPECT_EQ(info.pad_x, 0);
  EXPECT_EQ(info.pad_y, 80);
}

TEST(Letterbox, CustomPadValue) {
  constexpr int kPad = 7;
  cv::Mat src(100, 200, CV_8UC3, cv::Scalar::all(0));
  cv::Mat lb;
  sp::letterbox(src, kInputSize, lb, kPad);
  EXPECT_EQ(lb.at<cv::Vec3b>(0, 0), cv::Vec3b(kPad, kPad, kPad));
}

TEST(Letterbox, RejectsBadInput) {
  cv::Mat lb;
  EXPECT_THROW(sp::letterbox(cv::Mat(), kInputSize, lb), std::invalid_argument);
  EXPECT_THROW(sp::letterbox(cv::Mat(10, 10, CV_8UC1), kInputSize, lb), std::invalid_argument);
  EXPECT_THROW(sp::letterbox(cv::Mat(10, 10, CV_8UC3), 0, lb), std::invalid_argument);
}

TEST(Unletterbox, ClipsToSourceImage) {
  const sp::LetterboxInfo info{1.0F, 0, 80, 640, 480, kInputSize};
  // Box spilling into the top padding and past the right edge.
  const cv::Rect2f back = sp::unletterbox(cv::Rect2f(600.0F, 40.0F, 100.0F, 100.0F), info);
  EXPECT_FLOAT_EQ(back.x, 600.0F);
  EXPECT_FLOAT_EQ(back.y, 0.0F);
  EXPECT_FLOAT_EQ(back.x + back.width, 639.0F);
  EXPECT_FLOAT_EQ(back.y + back.height, 60.0F);
}

TEST(Blob, LayoutAndNormalisation) {
  // 2x3 image (rows x cols) with distinct BGR values per pixel.
  constexpr int kRows = 2;
  constexpr int kCols = 3;
  cv::Mat img(kRows, kCols, CV_8UC3);
  for (int y = 0; y < kRows; ++y) {
    for (int x = 0; x < kCols; ++x) {
      const int i = y * kCols + x;
      img.at<cv::Vec3b>(y, x) = cv::Vec3b(static_cast<uchar>(10 * i),        // B
                                          static_cast<uchar>(100 + 10 * i),  // G
                                          static_cast<uchar>(200 + 10 * i)   // R (<=250)
      );
    }
  }
  std::vector<float> blob;
  sp::blob_from_letterboxed(img, blob);
  const int plane = kRows * kCols;
  ASSERT_EQ(blob.size(), static_cast<std::size_t>(3 * plane));
  for (int i = 0; i < plane; ++i) {
    EXPECT_FLOAT_EQ(blob[static_cast<std::size_t>(i)], (200.0F + 10.0F * i) / 255.0F);  // R
    EXPECT_FLOAT_EQ(blob[static_cast<std::size_t>(plane + i)], (100.0F + 10.0F * i) / 255.0F);
    EXPECT_FLOAT_EQ(blob[static_cast<std::size_t>(2 * plane + i)], (10.0F * i) / 255.0F);  // B
  }
}

TEST(Blob, ReusesBufferWithoutReallocation) {
  cv::Mat img(4, 4, CV_8UC3, cv::Scalar(255, 0, 0));
  std::vector<float> blob(3 * 4 * 4, -1.0F);
  const float* before = blob.data();
  sp::blob_from_letterboxed(img, blob);
  EXPECT_EQ(blob.data(), before);
  EXPECT_FLOAT_EQ(blob[0], 0.0F);          // R plane
  EXPECT_FLOAT_EQ(blob[2 * 4 * 4], 1.0F);  // B plane
}

TEST(Blob, MatchesCvtColorAndSplit) {
  cv::Mat img(8, 8, CV_8UC3);
  cv::randu(img, cv::Scalar::all(0), cv::Scalar::all(256));
  std::vector<float> blob;
  sp::blob_from_letterboxed(img, blob);
  cv::Mat rgb;
  cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
  std::vector<cv::Mat> planes;
  cv::split(rgb, planes);
  for (int c = 0; c < 3; ++c) {
    for (int y = 0; y < 8; ++y) {
      for (int x = 0; x < 8; ++x) {
        const auto idx = static_cast<std::size_t>(c * 64 + y * 8 + x);
        EXPECT_FLOAT_EQ(blob[idx], planes[static_cast<std::size_t>(c)].at<uchar>(y, x) / 255.0F);
      }
    }
  }
}

}  // namespace
