// Hello-world Ort::Session test (architecture section 11): proves the pinned
// ONNX Runtime (scripts/setup_ort.sh) loads models/yolov8n.onnx before any ROS
// code exists. Deliberately includes no ROS header.

#include <gtest/gtest.h>
#include <onnxruntime_cxx_api.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#ifndef SEMNAV_TEST_MODEL_PATH
#error "SEMNAV_TEST_MODEL_PATH must be defined (path to models/yolov8n.onnx)"
#endif

namespace {

constexpr int64_t kInputChannels = 3;
constexpr int64_t kInputSize = 640;
constexpr int64_t kOutputAttributes = 84;  // 4 box + 80 COCO class scores
constexpr int64_t kOutputAnchors = 8400;

class OrtSessionTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "semnav_test_ort_session");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(*env_, SEMNAV_TEST_MODEL_PATH, options);
  }

  static void TearDownTestSuite() {
    session_.reset();
    env_.reset();
  }

  static std::unique_ptr<Ort::Env> env_;
  static std::unique_ptr<Ort::Session> session_;
};

std::unique_ptr<Ort::Env> OrtSessionTest::env_;
std::unique_ptr<Ort::Session> OrtSessionTest::session_;

TEST_F(OrtSessionTest, InputOutputSignatureMatchesYolov8n) {
  ASSERT_NE(session_, nullptr);
  Ort::AllocatorWithDefaultOptions allocator;

  ASSERT_EQ(session_->GetInputCount(), 1U);
  Ort::AllocatedStringPtr input_name = session_->GetInputNameAllocated(0, allocator);
  EXPECT_EQ(std::string(input_name.get()), "images");
  const std::vector<int64_t> input_shape =
      session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  EXPECT_EQ(input_shape, (std::vector<int64_t>{1, kInputChannels, kInputSize, kInputSize}));

  ASSERT_EQ(session_->GetOutputCount(), 1U);
  const std::vector<int64_t> output_shape =
      session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  EXPECT_EQ(output_shape, (std::vector<int64_t>{1, kOutputAttributes, kOutputAnchors}));
}

TEST_F(OrtSessionTest, RunOnZeroTensorGivesFiniteOutput) {
  ASSERT_NE(session_, nullptr);
  Ort::AllocatorWithDefaultOptions allocator;
  Ort::AllocatedStringPtr input_name = session_->GetInputNameAllocated(0, allocator);
  Ort::AllocatedStringPtr output_name = session_->GetOutputNameAllocated(0, allocator);

  const std::array<int64_t, 4> shape{1, kInputChannels, kInputSize, kInputSize};
  std::vector<float> input(static_cast<std::size_t>(kInputChannels * kInputSize * kInputSize),
                           0.0F);
  const Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input.data(), input.size(),
                                                            shape.data(), shape.size());

  const char *input_names[] = {input_name.get()};
  const char *output_names[] = {output_name.get()};
  std::vector<Ort::Value> outputs =
      session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

  ASSERT_EQ(outputs.size(), 1U);
  ASSERT_TRUE(outputs[0].IsTensor());
  const Ort::TensorTypeAndShapeInfo info = outputs[0].GetTensorTypeAndShapeInfo();
  ASSERT_EQ(info.GetElementType(), ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT);
  const std::size_t count = info.GetElementCount();
  ASSERT_EQ(count, static_cast<std::size_t>(kOutputAttributes * kOutputAnchors));

  const float *data = outputs[0].GetTensorData<float>();
  std::size_t non_finite = 0;
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(data[i])) {
      ++non_finite;
    }
  }
  EXPECT_EQ(non_finite, 0U);
}

TEST_F(OrtSessionTest, MetadataNamesContainPerson) {
  ASSERT_NE(session_, nullptr);
  Ort::AllocatorWithDefaultOptions allocator;
  const Ort::ModelMetadata metadata = session_->GetModelMetadata();
  Ort::AllocatedStringPtr names = metadata.LookupCustomMetadataMapAllocated("names", allocator);
  ASSERT_NE(names, nullptr) << "model has no 'names' custom metadata";
  EXPECT_NE(std::string(names.get()).find("0: 'person'"), std::string::npos) << names.get();
}

}  // namespace
