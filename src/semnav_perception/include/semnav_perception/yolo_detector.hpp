// YOLOv8 output decoding, NMS and class-name helpers (architecture 7.1).
//
// The free functions in this header are ROS-free and onnxruntime-free so they
// can be unit tested in isolation. They mirror decode() and nms() in
// scripts/check_yolo_on_frames.py. The ORT-backed YoloDetector class below them
// depends on ONNX Runtime and OpenCV only (still no ROS).
#pragma once

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <semnav_perception/letterbox.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace semnav_perception {

/// cx, cy, w, h precede the class scores in every YOLOv8 anchor column.
constexpr int kNumBoxValues = 4;
/// Guard for parse_class_names so std::stoi cannot overflow.
constexpr std::size_t kMaxClassIdDigits = 6;

struct Detection {
  int class_id;
  float score;
  cv::Rect2f box;  // x, y, w, h; frame stated by the producing function
};

/// Decode a YOLOv8 [1, num_attrs, num_anchors] output (channel-major: attribute
/// a of anchor i is out[a * num_anchors + i]). Attributes are cx, cy, w, h
/// followed by num_attrs - 4 class scores; there is no objectness.
///
/// For each anchor the best class score is taken over the allowed classes only
/// (class_mask[c] == true), so a masked class can never win and hide a lower
/// scoring allowed class. Anchors with best score >= conf_threshold are kept
/// (>= matches the Python reference). class_mask == nullptr allows all classes.
///
/// Returned boxes are x, y, w, h (top-left based) in network-input pixels.
inline std::vector<Detection> decode(const float* out, int num_attrs, int num_anchors,
                                     float conf_threshold,
                                     const std::vector<bool>* class_mask = nullptr) {
  if (out == nullptr) {
    throw std::invalid_argument("decode: null output tensor");
  }
  if (num_attrs <= kNumBoxValues || num_anchors < 0) {
    throw std::invalid_argument("decode: num_attrs must exceed 4 and num_anchors be >= 0");
  }
  const int num_classes = num_attrs - kNumBoxValues;
  if (class_mask != nullptr && static_cast<int>(class_mask->size()) != num_classes) {
    throw std::invalid_argument("decode: class_mask size " + std::to_string(class_mask->size()) +
                                " != number of classes " + std::to_string(num_classes));
  }
  const auto n = static_cast<std::size_t>(num_anchors);
  const auto attr = [out, n](int a, std::size_t i) {
    return out[static_cast<std::size_t>(a) * n + i];
  };

  std::vector<Detection> dets;
  for (std::size_t i = 0; i < n; ++i) {
    int best_c = -1;
    float best_s = 0.0F;
    for (int c = 0; c < num_classes; ++c) {
      if (class_mask != nullptr && !(*class_mask)[static_cast<std::size_t>(c)]) {
        continue;
      }
      const float s = attr(kNumBoxValues + c, i);
      if (best_c < 0 || s > best_s) {  // strict >: first max wins, like numpy argmax
        best_c = c;
        best_s = s;
      }
    }
    if (best_c < 0 || best_s < conf_threshold) {
      continue;
    }
    const float cx = attr(0, i);
    const float cy = attr(1, i);
    const float w = attr(2, i);
    const float h = attr(3, i);
    dets.push_back(Detection{best_c, best_s, cv::Rect2f(cx - w / 2.0F, cy - h / 2.0F, w, h)});
  }
  return dets;
}

/// Per-class NMS with cv::dnn::NMSBoxes.
///
/// Uses the class-offset trick from the Python reference: every box is shifted
/// diagonally by class_id * offset so boxes of different classes can never
/// overlap, and one NMSBoxes call suppresses only within a class. The offset is
/// the full coordinate span (max(x2, y2) - min(x, y)) + 1, which also stays
/// correct for boxes with negative coordinates (the Python offset assumes
/// non-negative x, y). Note NMSBoxes keeps scores strictly > score_threshold.
///
/// Returns the kept detections (boxes unshifted) sorted by score descending.
inline std::vector<Detection> nms(const std::vector<Detection>& dets, float score_threshold,
                                  float iou_threshold) {
  if (dets.empty()) {
    return {};
  }
  double lo = dets.front().box.x;
  double hi = lo;
  for (const auto& d : dets) {
    lo = std::min({lo, static_cast<double>(d.box.x), static_cast<double>(d.box.y)});
    hi = std::max({hi, static_cast<double>(d.box.x) + d.box.width,
                   static_cast<double>(d.box.y) + d.box.height});
  }
  const double offset = (hi - lo) + 1.0;

  std::vector<cv::Rect2d> shifted;
  std::vector<float> scores;
  shifted.reserve(dets.size());
  scores.reserve(dets.size());
  for (const auto& d : dets) {
    const double shift = static_cast<double>(d.class_id) * offset;
    shifted.emplace_back(d.box.x + shift, d.box.y + shift, d.box.width, d.box.height);
    scores.push_back(d.score);
  }
  std::vector<int> keep;
  cv::dnn::NMSBoxes(shifted, scores, score_threshold, iou_threshold, keep);

  std::vector<Detection> result;
  result.reserve(keep.size());
  for (int idx : keep) {
    result.push_back(dets[static_cast<std::size_t>(idx)]);
  }
  std::stable_sort(result.begin(), result.end(),
                   [](const Detection& a, const Detection& b) { return a.score > b.score; });
  return result;
}

/// Parse the Ultralytics ONNX "names" metadata, a Python dict literal such as
/// "{0: 'person', 1: 'bicycle'}", into a vector indexed by class id.
/// Accepts single or double quotes and backslash escapes inside names.
/// Throws std::invalid_argument on malformed input, duplicate ids, or ids that
/// are not exactly 0..N-1.
inline std::vector<std::string> parse_class_names(const std::string& text) {
  std::size_t p = 0;
  const auto fail = [&p](const std::string& what) {
    throw std::invalid_argument("parse_class_names: " + what + " at offset " + std::to_string(p));
  };
  const auto skip_ws = [&]() {
    while (p < text.size() && std::isspace(static_cast<unsigned char>(text[p])) != 0) {
      ++p;
    }
  };
  const auto expect = [&](char c) {
    skip_ws();
    if (p >= text.size() || text[p] != c) {
      fail(std::string("expected '") + c + "'");
    }
    ++p;
  };

  std::unordered_map<int, std::string> by_id;
  expect('{');
  skip_ws();
  if (p < text.size() && text[p] == '}') {
    ++p;
  } else {
    for (;;) {
      skip_ws();
      const std::size_t start = p;
      while (p < text.size() && std::isdigit(static_cast<unsigned char>(text[p])) != 0) {
        ++p;
      }
      if (p == start) {
        fail("expected class id");
      }
      if (p - start > kMaxClassIdDigits) {
        fail("class id too large");
      }
      const int id = std::stoi(text.substr(start, p - start));
      expect(':');
      skip_ws();
      if (p >= text.size() || (text[p] != '\'' && text[p] != '"')) {
        fail("expected quoted class name");
      }
      const char quote = text[p++];
      std::string name;
      while (p < text.size() && text[p] != quote) {
        if (text[p] == '\\' && p + 1 < text.size()) {
          ++p;
        }
        name.push_back(text[p++]);
      }
      if (p >= text.size()) {
        fail("unterminated class name");
      }
      ++p;  // closing quote
      if (!by_id.emplace(id, name).second) {
        fail("duplicate class id " + std::to_string(id));
      }
      skip_ws();
      if (p < text.size() && text[p] == ',') {
        ++p;
        skip_ws();
        if (p < text.size() && text[p] == '}') {  // trailing comma
          ++p;
          break;
        }
        continue;
      }
      expect('}');
      break;
    }
  }
  skip_ws();
  if (p != text.size()) {
    fail("trailing characters");
  }

  std::vector<std::string> names(by_id.size());
  for (auto& [id, name] : by_id) {
    if (id < 0 || static_cast<std::size_t>(id) >= names.size()) {
      throw std::invalid_argument("parse_class_names: class ids are not contiguous from 0");
    }
    names[static_cast<std::size_t>(id)] = std::move(name);
  }
  return names;
}

/// Build a decode() class mask (true = keep) from class names to keep.
/// Throws std::invalid_argument listing every name not found in `names`.
/// An empty `keep` yields an all-false mask; callers that want no filtering
/// should pass class_mask = nullptr to decode() instead.
inline std::vector<bool> build_class_mask(const std::vector<std::string>& names,
                                          const std::vector<std::string>& keep) {
  std::vector<bool> mask(names.size(), false);
  std::string unknown;
  for (const auto& k : keep) {
    const auto it = std::find(names.begin(), names.end(), k);
    if (it == names.end()) {
      unknown += (unknown.empty() ? "" : ", ") + k;
      continue;
    }
    mask[static_cast<std::size_t>(it - names.begin())] = true;
  }
  if (!unknown.empty()) {
    throw std::invalid_argument("build_class_mask: unknown class name(s): " + unknown);
  }
  return mask;
}

struct YoloOptions {
  std::string model_path;
  int intra_op_num_threads;
  bool use_cuda;
};

/// Per-stage wall-clock timing of one detect() call, milliseconds.
struct StageTiming {
  double preprocess_ms;
  double infer_ms;
  double postprocess_ms;
};

/// YOLOv8 ONNX detector (architecture 7.1): letterbox -> BGR to RGB -> /255 ->
/// NCHW -> Ort::Session::Run -> decode -> NMS -> undo letterbox.
/// Input/output names, the square input size and the class names are read from
/// the model. The input buffer and its Ort::Value are allocated once and reused.
class YoloDetector {
 public:
  explicit YoloDetector(const YoloOptions& opt)
      : env_(ORT_LOGGING_LEVEL_WARNING, "semnav_yolo"),
        memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(opt.intra_op_num_threads);
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    if (opt.use_cuda) {
      try {
        OrtCUDAProviderOptions cuda{};
        so.AppendExecutionProvider_CUDA(cuda);
        cuda_active_ = true;
      } catch (const Ort::Exception& e) {
        warnings_.push_back(std::string("CUDA execution provider unavailable, using CPU: ") +
                            e.what());
      }
    }
    session_ = std::make_unique<Ort::Session>(env_, opt.model_path.c_str(), so);

    Ort::AllocatorWithDefaultOptions alloc;
    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
      throw std::runtime_error("YoloDetector: expected 1 input and 1 output");
    }
    input_name_ = session_->GetInputNameAllocated(0, alloc).get();
    output_name_ = session_->GetOutputNameAllocated(0, alloc).get();
    const auto in_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.size() != 4 || in_shape[1] != kInputChannels || in_shape[2] <= 0 ||
        in_shape[2] != in_shape[3]) {
      throw std::runtime_error("YoloDetector: expected static input [1,3,S,S]");
    }
    input_size_ = static_cast<int>(in_shape[2]);
    const auto out_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (out_shape.size() != 3 || out_shape[1] <= kNumBoxValues || out_shape[2] <= 0) {
      throw std::runtime_error("YoloDetector: expected output [1, 4+C, N]");
    }
    num_attrs_ = static_cast<int>(out_shape[1]);
    num_anchors_ = static_cast<int>(out_shape[2]);

    const auto meta = session_->GetModelMetadata();
    const auto names = meta.LookupCustomMetadataMapAllocated("names", alloc);
    if (!names) {
      throw std::runtime_error("YoloDetector: model metadata has no 'names'");
    }
    class_names_ = parse_class_names(names.get());
    if (static_cast<int>(class_names_.size()) != num_attrs_ - kNumBoxValues) {
      throw std::runtime_error("YoloDetector: class name count does not match output shape");
    }

    input_buffer_.assign(static_cast<std::size_t>(kInputChannels) * input_size_ * input_size_, 0.F);
    input_shape_ = {1, kInputChannels, input_size_, input_size_};
    input_tensor_ =
        Ort::Value::CreateTensor<float>(memory_info_, input_buffer_.data(), input_buffer_.size(),
                                        input_shape_.data(), input_shape_.size());
  }

  /// Detections with boxes in source-image pixels, sorted by score descending.
  std::vector<Detection> detect(const cv::Mat& bgr, float conf_threshold, float iou_threshold,
                                const std::vector<bool>* class_mask, StageTiming* timing) {
    using Clock = std::chrono::steady_clock;
    const auto ms = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    const auto t0 = Clock::now();
    const LetterboxInfo info = letterbox(bgr, input_size_, letterboxed_);
    blob_from_letterboxed(letterboxed_, input_buffer_);  // same size: no reallocation
    const auto t1 = Clock::now();

    const char* in_names[] = {input_name_.c_str()};
    const char* out_names[] = {output_name_.c_str()};
    auto outputs =
        session_->Run(Ort::RunOptions{nullptr}, in_names, &input_tensor_, 1, out_names, 1);
    const auto t2 = Clock::now();

    const float* out = outputs.front().GetTensorData<float>();
    auto dets = nms(decode(out, num_attrs_, num_anchors_, conf_threshold, class_mask),
                    conf_threshold, iou_threshold);
    for (auto& d : dets) {
      d.box = unletterbox(d.box, info);
    }
    const auto t3 = Clock::now();
    if (timing != nullptr) {
      *timing = {ms(t0, t1), ms(t1, t2), ms(t2, t3)};
    }
    return dets;
  }

  int input_size() const { return input_size_; }
  bool cuda_active() const { return cuda_active_; }
  const std::vector<std::string>& class_names() const { return class_names_; }
  /// Non-fatal problems found while loading (e.g. CUDA fallback).
  const std::vector<std::string>& warnings() const { return warnings_; }

 private:
  Ort::Env env_;
  Ort::MemoryInfo memory_info_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  int input_size_{0};
  int num_attrs_{0};
  int num_anchors_{0};
  std::vector<std::string> class_names_;
  std::vector<std::string> warnings_;
  bool cuda_active_{false};
  std::vector<float> input_buffer_;
  std::array<int64_t, 4> input_shape_{};
  Ort::Value input_tensor_{nullptr};
  cv::Mat letterboxed_;
};

}  // namespace semnav_perception
