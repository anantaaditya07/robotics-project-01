// yolo_onnx_node (architecture 7.1): YOLOv8 ONNX inference on /camera/image_raw.
//
// The image callback only stores the newest frame under a mutex and signals a condition
// variable; a worker thread runs inference. Frames that arrive while the worker is busy replace
// the stored one, so old frames are dropped instead of queued and latency never grows.
//
// Publishes:
//   /detections        vision_msgs/Detection2DArray, reliable depth 5, header copied from image
//   /detections/image  sensor_msgs/Image (annotated), best effort, if publish_annotated
//   /metrics           std_msgs/Float32MultiArray (D-09), every metrics_period seconds, layout:
//                      [0] preprocess p50  [1] preprocess p95   (ms)
//                      [2] infer p50       [3] infer p95        (ms)
//                      [4] postprocess p50 [5] postprocess p95  (ms)
//                      [6] total p50       [7] total p95        (ms)
//                      [8] processed fps over the period
//                      [9] frames dropped over the period (replaced before processing)
#include <cv_bridge/cv_bridge.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <semnav_perception/rolling_stats.hpp>
#include <semnav_perception/yolo_detector.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <string>
#include <thread>
#include <vector>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace semnav_perception {

namespace {
constexpr int kMetricsFields = 10;
constexpr int kAnnotationThickness = 2;
constexpr double kLabelFontScale = 0.5;
constexpr int kLabelMargin = 4;  // px between label baseline and box edge
const cv::Scalar kBoxColour(0, 255, 0);
}  // namespace

class YoloOnnxNode : public rclcpp::Node {
 public:
  YoloOnnxNode() : rclcpp::Node("yolo_onnx_node") {
    const auto model_path = declare_parameter<std::string>("model_path", SEMNAV_DEFAULT_MODEL_PATH);
    conf_threshold_ = static_cast<float>(declare_parameter<double>("conf_threshold", 0.35));
    iou_threshold_ = static_cast<float>(declare_parameter<double>("iou_threshold", 0.45));
    const auto input_size = declare_parameter<int>("input_size", 640);
    const auto class_filter = declare_parameter<std::vector<std::string>>(
        "class_filter", std::vector<std::string>{"person", "chair"});  // D-15
    const auto use_cuda = declare_parameter<bool>("use_cuda", false);
    publish_annotated_ = declare_parameter<bool>("publish_annotated", true);
    const auto threads = declare_parameter<int>("intra_op_num_threads", 4);
    const auto image_qos_depth = declare_parameter<int>("image_qos_depth", 1);  // D-07: 1-2
    const auto detections_qos_depth = declare_parameter<int>("detections_qos_depth", 5);
    const auto metrics_period = declare_parameter<double>("metrics_period", 5.0);
    const auto metrics_window = declare_parameter<int>("metrics_window", 200);

    detector_ = std::make_unique<YoloDetector>(
        YoloOptions{model_path, static_cast<int>(threads), use_cuda});
    for (const auto& w : detector_->warnings()) {
      RCLCPP_WARN(get_logger(), "%s", w.c_str());
    }
    if (input_size != detector_->input_size()) {
      throw std::invalid_argument("input_size " + std::to_string(input_size) +
                                  " does not match the model input " +
                                  std::to_string(detector_->input_size()));
    }
    // An empty class_filter keeps every class (decode() with no mask).
    if (!class_filter.empty()) {
      class_mask_ = build_class_mask(detector_->class_names(), class_filter);
    }

    for (auto* s : {&pre_ms_, &infer_ms_, &post_ms_, &total_ms_}) {
      *s = std::make_unique<RollingStats>(static_cast<std::size_t>(metrics_window));
    }

    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        "detections", rclcpp::QoS(static_cast<std::size_t>(detections_qos_depth)).reliable());
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("detections/image",
                                                           rclcpp::SensorDataQoS().keep_last(1));
    metrics_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("metrics", 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw",
        rclcpp::SensorDataQoS().keep_last(static_cast<std::size_t>(image_qos_depth)),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(std::move(msg)); });
    metrics_timer_ = create_wall_timer(std::chrono::duration<double>(metrics_period),
                                       [this]() { report_metrics(); });
    period_start_ = std::chrono::steady_clock::now();

    worker_ = std::thread([this]() { worker_loop(); });
    RCLCPP_INFO(get_logger(), "model %s: input %d, %zu classes, filter [%s], %s, %ld threads",
                model_path.c_str(), detector_->input_size(), detector_->class_names().size(),
                join(class_filter).c_str(), detector_->cuda_active() ? "CUDA" : "CPU", threads);
  }

  ~YoloOnnxNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  static std::string join(const std::vector<std::string>& v) {
    std::string out;
    for (const auto& s : v) {
      out += (out.empty() ? "" : ", ") + s;
    }
    return out;
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_) {
        ++dropped_;
      }
      pending_ = std::move(msg);
    }
    cv_.notify_one();
  }

  void worker_loop() {
    while (true) {
      sensor_msgs::msg::Image::ConstSharedPtr msg;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return stop_ || pending_ != nullptr; });
        if (stop_) {
          return;
        }
        msg = std::move(pending_);
        pending_.reset();
      }
      try {
        process(msg);
      } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "inference failed: %s", e.what());
      }
    }
  }

  void process(const sensor_msgs::msg::Image::ConstSharedPtr& msg_ptr) {
    const auto start = std::chrono::steady_clock::now();
    const auto& msg = *msg_ptr;
    const auto cv_img = cv_bridge::toCvShare(msg_ptr, "bgr8");
    StageTiming t{};
    const auto dets = detector_->detect(cv_img->image, conf_threshold_, iou_threshold_,
                                        class_mask_.empty() ? nullptr : &class_mask_, &t);
    const auto& names = detector_->class_names();

    vision_msgs::msg::Detection2DArray out;
    out.header = msg.header;  // stamp copied from the image (architecture section 6)
    out.detections.reserve(dets.size());
    for (const auto& d : dets) {
      vision_msgs::msg::Detection2D det;
      det.header = msg.header;
      det.bbox.center.position.x = d.box.x + d.box.width / 2.0;
      det.bbox.center.position.y = d.box.y + d.box.height / 2.0;
      det.bbox.size_x = d.box.width;
      det.bbox.size_y = d.box.height;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = names[static_cast<std::size_t>(d.class_id)];
      hyp.hypothesis.score = d.score;
      det.results.push_back(hyp);
      out.detections.push_back(std::move(det));
    }
    detections_pub_->publish(out);

    if (publish_annotated_ && image_pub_->get_subscription_count() > 0) {
      cv::Mat annotated = cv_img->image.clone();
      for (const auto& d : dets) {
        cv::rectangle(annotated, d.box, kBoxColour, kAnnotationThickness);
        const auto label = names[static_cast<std::size_t>(d.class_id)] + " " +
                           cv::format("%.2f", static_cast<double>(d.score));
        // Above the box if there is room, otherwise just inside its top edge.
        int baseline = 0;
        const auto text =
            cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, kLabelFontScale, 1, &baseline);
        const int top = static_cast<int>(d.box.y);
        const int y = top - kLabelMargin >= text.height ? top - kLabelMargin
                                                        : top + text.height + kLabelMargin;
        cv::putText(annotated, label, cv::Point(static_cast<int>(d.box.x) + kLabelMargin, y),
                    cv::FONT_HERSHEY_SIMPLEX, kLabelFontScale, kBoxColour, 1);
      }
      image_pub_->publish(*cv_bridge::CvImage(msg.header, "bgr8", annotated).toImageMsg());
    }

    const double total =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    std::lock_guard<std::mutex> lock(stats_mutex_);
    pre_ms_->add(t.preprocess_ms);
    infer_ms_->add(t.infer_ms);
    post_ms_->add(t.postprocess_ms);
    total_ms_->add(total);
    ++processed_;
  }

  void report_metrics() {
    std_msgs::msg::Float32MultiArray m;
    std::size_t processed = 0;
    std::size_t dropped = 0;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      for (const auto* s : {&pre_ms_, &infer_ms_, &post_ms_, &total_ms_}) {
        m.data.push_back(static_cast<float>((*s)->percentile(50)));
        m.data.push_back(static_cast<float>((*s)->percentile(95)));
      }
      processed = processed_;
      processed_ = 0;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      dropped = dropped_;
      dropped_ = 0;
    }
    const auto now = std::chrono::steady_clock::now();
    const double secs = std::chrono::duration<double>(now - period_start_).count();
    period_start_ = now;
    const double fps = secs > 0.0 ? static_cast<double>(processed) / secs : 0.0;
    m.data.push_back(static_cast<float>(fps));
    m.data.push_back(static_cast<float>(dropped));
    std_msgs::msg::MultiArrayDimension dim;
    dim.label =
        "pre_p50,pre_p95,infer_p50,infer_p95,post_p50,post_p95,total_p50,total_p95,fps,dropped";
    dim.size = kMetricsFields;
    dim.stride = kMetricsFields;
    m.layout.dim.push_back(dim);
    metrics_pub_->publish(m);
    RCLCPP_INFO(get_logger(),
                "ms p50/p95  pre %.1f/%.1f  infer %.1f/%.1f  post %.1f/%.1f  total %.1f/%.1f  "
                "| %.1f fps, %zu dropped",
                m.data[0], m.data[1], m.data[2], m.data[3], m.data[4], m.data[5], m.data[6],
                m.data[7], fps, dropped);
  }

  std::unique_ptr<YoloDetector> detector_;
  float conf_threshold_;
  float iou_threshold_;
  bool publish_annotated_;
  std::vector<bool> class_mask_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr metrics_timer_;

  // Newest-frame hand-off between the callback and the worker.
  std::mutex mutex_;
  std::condition_variable cv_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_;
  bool stop_{false};
  std::size_t dropped_{0};
  std::thread worker_;

  std::mutex stats_mutex_;
  std::unique_ptr<RollingStats> pre_ms_, infer_ms_, post_ms_, total_ms_;
  std::size_t processed_{0};
  std::chrono::steady_clock::time_point period_start_;
};

}  // namespace semnav_perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<semnav_perception::YoloOnnxNode>());
  rclcpp::shutdown();
  return 0;
}
