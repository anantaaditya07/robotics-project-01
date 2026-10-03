// Letterbox pre-processing for YOLOv8 (architecture 7.1). ROS-free, header-only.
//
// Mirrors letterbox(), preprocess() and unletterbox_boxes() in
// scripts/check_yolo_on_frames.py.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

namespace semnav_perception {

/// Letterbox grey, per architecture 7.1.
constexpr int kDefaultPadValue = 114;
/// Number of colour channels in the network input.
constexpr int kInputChannels = 3;
/// uint8 -> [0, 1] normalisation factor.
constexpr float kPixelNormalisation = 1.0F / 255.0F;

/// Geometry of one letterbox transform. A source pixel p maps to
/// p * scale + pad in the letterboxed (network input) image.
struct LetterboxInfo {
  float scale;
  int pad_x;
  int pad_y;
  int src_w;
  int src_h;
  int size;
};

/// Resize `bgr` keeping aspect ratio into a centred size x size CV_8UC3 canvas
/// padded with `pad_value`. Rounding matches the Python reference:
/// new_w = round(w * scale) with round-half-to-even (std::nearbyint under the
/// default FE_TONEAREST mode == Python's round()), pad = (size - new) / 2 floored.
/// `out` is reallocated only if it does not already have the right size/type.
inline LetterboxInfo letterbox(const cv::Mat& bgr, int size, cv::Mat& out,
                               int pad_value = kDefaultPadValue) {
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw std::invalid_argument("letterbox: input must be a non-empty CV_8UC3 image");
  }
  if (size <= 0) {
    throw std::invalid_argument("letterbox: size must be positive");
  }
  const int w = bgr.cols;
  const int h = bgr.rows;
  const double scale = std::min(static_cast<double>(size) / w, static_cast<double>(size) / h);
  const int new_w = std::clamp(static_cast<int>(std::nearbyint(w * scale)), 1, size);
  const int new_h = std::clamp(static_cast<int>(std::nearbyint(h * scale)), 1, size);
  const int pad_x = (size - new_w) / 2;
  const int pad_y = (size - new_h) / 2;

  out.create(size, size, CV_8UC3);
  out.setTo(cv::Scalar::all(pad_value));
  cv::Mat roi = out(cv::Rect(pad_x, pad_y, new_w, new_h));
  if (new_w == w && new_h == h) {
    bgr.copyTo(roi);
  } else {
    cv::resize(bgr, roi, cv::Size(new_w, new_h), 0.0, 0.0, cv::INTER_LINEAR);
  }
  return LetterboxInfo{static_cast<float>(scale), pad_x, pad_y, w, h, size};
}

/// BGR uint8 HWC -> RGB float32 /255 NCHW (batch 1) into a caller-owned buffer.
/// The buffer is resized only when its size differs, so a pre-allocated buffer
/// is reused across frames without reallocation.
inline void blob_from_letterboxed(const cv::Mat& lb_bgr, std::vector<float>& nchw) {
  if (lb_bgr.empty() || lb_bgr.type() != CV_8UC3) {
    throw std::invalid_argument("blob_from_letterboxed: input must be a non-empty CV_8UC3 image");
  }
  const int rows = lb_bgr.rows;
  const int cols = lb_bgr.cols;
  const std::size_t plane = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  const std::size_t needed = plane * kInputChannels;
  if (nchw.size() != needed) {
    nchw.resize(needed);
  }
  float* r_plane = nchw.data();
  float* g_plane = r_plane + plane;
  float* b_plane = g_plane + plane;
  for (int y = 0; y < rows; ++y) {
    const auto* src = lb_bgr.ptr<cv::Vec3b>(y);
    const std::size_t row_off = static_cast<std::size_t>(y) * static_cast<std::size_t>(cols);
    for (int x = 0; x < cols; ++x) {
      const std::size_t i = row_off + static_cast<std::size_t>(x);
      b_plane[i] = static_cast<float>(src[x][0]) * kPixelNormalisation;
      g_plane[i] = static_cast<float>(src[x][1]) * kPixelNormalisation;
      r_plane[i] = static_cast<float>(src[x][2]) * kPixelNormalisation;
    }
  }
}

/// Map an x,y,w,h box from letterboxed input pixels back to source pixels:
/// subtract pad, divide by scale, then clip corners to [0, src_w - 1] x
/// [0, src_h - 1] (same clip as the Python reference, which clips x1,y1,x2,y2).
inline cv::Rect2f unletterbox(const cv::Rect2f& box_in_input, const LetterboxInfo& info) {
  const float max_x = static_cast<float>(std::max(info.src_w - 1, 0));
  const float max_y = static_cast<float>(std::max(info.src_h - 1, 0));
  const float px = static_cast<float>(info.pad_x);
  const float py = static_cast<float>(info.pad_y);
  const float x1 = std::clamp((box_in_input.x - px) / info.scale, 0.0F, max_x);
  const float y1 = std::clamp((box_in_input.y - py) / info.scale, 0.0F, max_y);
  const float x2 = std::clamp((box_in_input.x + box_in_input.width - px) / info.scale, 0.0F, max_x);
  const float y2 =
      std::clamp((box_in_input.y + box_in_input.height - py) / info.scale, 0.0F, max_y);
  return cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
}

}  // namespace semnav_perception
