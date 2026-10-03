// Unit tests for the ROS-free parts of yolo_detector.hpp (architecture 7.1).
#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "semnav_perception/yolo_detector.hpp"

namespace sp = semnav_perception;

namespace {

constexpr int kNumClasses = 80;
constexpr int kNumAttrs = sp::kNumBoxValues + kNumClasses;  // 84
constexpr int kNumAnchors = 8400;
constexpr float kConf = 0.35F;
constexpr float kIou = 0.45F;
constexpr int kPerson = 0;
constexpr int kChair = 56;

// Synthetic [1, 84, 8400] channel-major YOLOv8 output, all zeros by default.
class Tensor {
 public:
  Tensor() : data_(static_cast<std::size_t>(kNumAttrs) * kNumAnchors, 0.0F) {}

  float& at(int attr, int anchor) {
    return data_[static_cast<std::size_t>(attr) * kNumAnchors + static_cast<std::size_t>(anchor)];
  }
  void plant(int anchor, float cx, float cy, float w, float h, int cls, float score) {
    at(0, anchor) = cx;
    at(1, anchor) = cy;
    at(2, anchor) = w;
    at(3, anchor) = h;
    at(sp::kNumBoxValues + cls, anchor) = score;
  }
  const float* data() const { return data_.data(); }

 private:
  std::vector<float> data_;
};

TEST(Decode, ReturnsPlantedBox) {
  Tensor t;
  t.plant(1234, 320.0F, 240.0F, 100.0F, 50.0F, kChair, 0.9F);
  // A noise score below threshold on another anchor must be ignored.
  t.plant(42, 10.0F, 10.0F, 5.0F, 5.0F, kPerson, 0.1F);

  const auto dets = sp::decode(t.data(), kNumAttrs, kNumAnchors, kConf);
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_EQ(dets[0].class_id, kChair);
  EXPECT_FLOAT_EQ(dets[0].score, 0.9F);
  EXPECT_FLOAT_EQ(dets[0].box.x, 270.0F);
  EXPECT_FLOAT_EQ(dets[0].box.y, 215.0F);
  EXPECT_FLOAT_EQ(dets[0].box.width, 100.0F);
  EXPECT_FLOAT_EQ(dets[0].box.height, 50.0F);
}

TEST(Decode, TakesMaxClassScore) {
  Tensor t;
  t.plant(7, 100.0F, 100.0F, 20.0F, 20.0F, kPerson, 0.5F);
  t.at(sp::kNumBoxValues + kChair, 7) = 0.8F;
  const auto dets = sp::decode(t.data(), kNumAttrs, kNumAnchors, kConf);
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_EQ(dets[0].class_id, kChair);
  EXPECT_FLOAT_EQ(dets[0].score, 0.8F);
}

TEST(Decode, ConfidenceFiltering) {
  Tensor t;
  t.plant(1, 50.0F, 50.0F, 10.0F, 10.0F, kPerson, kConf - 0.01F);
  t.plant(2, 60.0F, 60.0F, 10.0F, 10.0F, kPerson, kConf);  // == threshold is kept
  t.plant(3, 70.0F, 70.0F, 10.0F, 10.0F, kPerson, kConf + 0.2F);
  const auto dets = sp::decode(t.data(), kNumAttrs, kNumAnchors, kConf);
  ASSERT_EQ(dets.size(), 2U);
  EXPECT_FLOAT_EQ(dets[0].score, kConf);
  EXPECT_FLOAT_EQ(dets[1].score, kConf + 0.2F);
  EXPECT_TRUE(sp::decode(t.data(), kNumAttrs, kNumAnchors, 0.99F).empty());
}

TEST(Decode, ClassMaskExcludesClass) {
  Tensor t;
  t.plant(10, 100.0F, 100.0F, 20.0F, 20.0F, kChair, 0.9F);
  t.plant(20, 300.0F, 300.0F, 40.0F, 80.0F, kPerson, 0.7F);
  std::vector<bool> mask(kNumClasses, false);
  mask[kPerson] = true;
  const auto dets = sp::decode(t.data(), kNumAttrs, kNumAnchors, kConf, &mask);
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_EQ(dets[0].class_id, kPerson);
}

TEST(Decode, MaskedClassCannotHideAllowedClass) {
  // Anchor scores chair 0.9 (masked) and person 0.6 (allowed): person must win.
  Tensor t;
  t.plant(5, 100.0F, 100.0F, 20.0F, 20.0F, kChair, 0.9F);
  t.at(sp::kNumBoxValues + kPerson, 5) = 0.6F;
  std::vector<bool> mask(kNumClasses, true);
  mask[kChair] = false;
  const auto dets = sp::decode(t.data(), kNumAttrs, kNumAnchors, kConf, &mask);
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_EQ(dets[0].class_id, kPerson);
  EXPECT_FLOAT_EQ(dets[0].score, 0.6F);
}

TEST(Decode, RejectsBadArguments) {
  Tensor t;
  std::vector<bool> wrong_size(kNumClasses - 1, true);
  EXPECT_THROW(sp::decode(t.data(), kNumAttrs, kNumAnchors, kConf, &wrong_size),
               std::invalid_argument);
  EXPECT_THROW(sp::decode(nullptr, kNumAttrs, kNumAnchors, kConf), std::invalid_argument);
  EXPECT_THROW(sp::decode(t.data(), sp::kNumBoxValues, kNumAnchors, kConf), std::invalid_argument);
}

TEST(Nms, RemovesDuplicateKeepsOtherClass) {
  const std::vector<sp::Detection> dets = {
      {kPerson, 0.80F, cv::Rect2f(100.0F, 100.0F, 50.0F, 100.0F)},
      {kPerson, 0.90F, cv::Rect2f(102.0F, 101.0F, 50.0F, 100.0F)},  // duplicate, higher score
      {kChair, 0.60F, cv::Rect2f(101.0F, 100.0F, 50.0F, 100.0F)},   // overlaps, other class
      {kPerson, 0.50F, cv::Rect2f(400.0F, 300.0F, 40.0F, 80.0F)},   // separate person
  };
  const auto kept = sp::nms(dets, kConf, kIou);
  ASSERT_EQ(kept.size(), 3U);
  // Sorted by score descending, boxes unshifted.
  EXPECT_EQ(kept[0].class_id, kPerson);
  EXPECT_FLOAT_EQ(kept[0].score, 0.90F);
  EXPECT_FLOAT_EQ(kept[0].box.x, 102.0F);
  EXPECT_EQ(kept[1].class_id, kChair);
  EXPECT_FLOAT_EQ(kept[1].box.x, 101.0F);
  EXPECT_FLOAT_EQ(kept[1].box.y, 100.0F);
  EXPECT_EQ(kept[2].class_id, kPerson);
  EXPECT_FLOAT_EQ(kept[2].score, 0.50F);
}

TEST(Nms, ClassOffsetHandlesNegativeCoordinates) {
  // Boxes partly outside the input (negative x/y) of different classes must
  // still not suppress each other.
  const std::vector<sp::Detection> dets = {
      {kPerson, 0.9F, cv::Rect2f(-30.0F, -30.0F, 60.0F, 60.0F)},
      {1, 0.8F, cv::Rect2f(-30.0F, -30.0F, 60.0F, 60.0F)},
      {2, 0.7F, cv::Rect2f(-30.0F, -30.0F, 60.0F, 60.0F)},
  };
  EXPECT_EQ(sp::nms(dets, kConf, kIou).size(), 3U);
}

TEST(Nms, EndToEndWithDecode) {
  Tensor t;
  t.plant(100, 320.0F, 240.0F, 100.0F, 200.0F, kPerson, 0.85F);
  t.plant(101, 322.0F, 241.0F, 100.0F, 200.0F, kPerson, 0.80F);
  t.plant(102, 321.0F, 240.0F, 100.0F, 200.0F, kChair, 0.55F);
  const auto kept = sp::nms(sp::decode(t.data(), kNumAttrs, kNumAnchors, kConf), kConf, kIou);
  ASSERT_EQ(kept.size(), 2U);
  EXPECT_EQ(kept[0].class_id, kPerson);
  EXPECT_FLOAT_EQ(kept[0].score, 0.85F);
  EXPECT_EQ(kept[1].class_id, kChair);
}

TEST(Nms, EmptyInput) { EXPECT_TRUE(sp::nms({}, kConf, kIou).empty()); }

TEST(ClassNames, ParsesUltralyticsMetadata) {
  const auto names =
      sp::parse_class_names("{0: 'person', 1: 'bicycle', 2: \"traffic light\", 3: 'it\\'s'}");
  ASSERT_EQ(names.size(), 4U);
  EXPECT_EQ(names[0], "person");
  EXPECT_EQ(names[1], "bicycle");
  EXPECT_EQ(names[2], "traffic light");
  EXPECT_EQ(names[3], "it's");
}

TEST(ClassNames, OutOfOrderAndWhitespace) {
  const auto names = sp::parse_class_names("  {\n 1 : 'b' ,\n 0:'a', }  ");
  ASSERT_EQ(names.size(), 2U);
  EXPECT_EQ(names[0], "a");
  EXPECT_EQ(names[1], "b");
}

TEST(ClassNames, Empty) { EXPECT_TRUE(sp::parse_class_names("{}").empty()); }

TEST(ClassNames, RejectsMalformed) {
  EXPECT_THROW(sp::parse_class_names(""), std::invalid_argument);
  EXPECT_THROW(sp::parse_class_names("{0: person}"), std::invalid_argument);
  EXPECT_THROW(sp::parse_class_names("{0: 'person'"), std::invalid_argument);
  EXPECT_THROW(sp::parse_class_names("{0: 'person}"), std::invalid_argument);
  EXPECT_THROW(sp::parse_class_names("{0: 'a', 0: 'b'}"), std::invalid_argument);
  EXPECT_THROW(sp::parse_class_names("{0: 'a', 2: 'c'}"), std::invalid_argument);
  EXPECT_THROW(sp::parse_class_names("{0: 'a'} x"), std::invalid_argument);
  EXPECT_THROW(sp::parse_class_names("{99999999999: 'a'}"), std::invalid_argument);
}

TEST(ClassMask, BuildsFromNames) {
  const std::vector<std::string> names = {"person", "bicycle", "chair"};
  const auto mask = sp::build_class_mask(names, {"person", "chair"});
  EXPECT_EQ(mask, (std::vector<bool>{true, false, true}));
}

TEST(ClassMask, UnknownNameThrows) {
  const std::vector<std::string> names = {"person", "chair"};
  try {
    sp::build_class_mask(names, {"person", "sofa", "unicorn"});
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("sofa"), std::string::npos);
    EXPECT_NE(msg.find("unicorn"), std::string::npos);
  }
}

}  // namespace
