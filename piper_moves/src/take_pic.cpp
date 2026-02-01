```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

// ============================================================================
// Configuration Constants
// ============================================================================

namespace config {
  // Contour detection parameters
  constexpr double kTabletContourMinArea = 25000.0;
  constexpr double kTabletContourMaxArea = 30000.0;
  constexpr int kMinShapeArea = 80;

  // Line detection parameters
  constexpr int kHoughLinesRho = 1;
  constexpr double kHoughLinesTheta = CV_PI / 180.0;
  constexpr int kVerticalLinesThreshold = 90;
  constexpr int kVerticalLinesMinLength = 60;
  constexpr int kVerticalLinesMaxGap = 60;
  constexpr int kHorizontalLinesThreshold = 60;
  constexpr int kHorizontalLinesMinLength = 30;
  constexpr int kHorizontalLinesMaxGap = 20;

  // Angle detection
  constexpr float kVerticalAngleTolerance = 20.0f;
  constexpr float kHorizontalAngleTolerance = 10.0f;

  // Shape detection parameters
  constexpr double kCrossSolidityThreshold = 0.85;
  constexpr double kCircleSolidityThreshold = 0.90;

  // Buffer parameters
  constexpr int kMaxBufferSize = 10;
  constexpr int kMinBufferSize = 5;
  constexpr double kVoteThresholdRatio = 0.6;

  // Clustering parameters
  constexpr int kGridSize = 3;
  constexpr int kTotalSquares = 9;

  // Camera parameters
  constexpr const char* kDefaultCameraTopic = "/camera/D435/color/image_raw";

  // Image processing
  constexpr int kMorphKernelSize = 5;
  constexpr int kMorphEllipseSize = 9;
  constexpr int kAdaptiveThreshBlockSize = 11;
  constexpr int kAdaptiveThreshConstant = -3;
  constexpr int kBinaryThreshold = 25;
  constexpr double kCannyMedianRatio1 = 0.66;
  constexpr double kCannyMedianRatio2 = 1.33;

  // Detection thresholds
  constexpr int kConsecutiveFramesThreshold = 10;
  constexpr int kMinShapesToPublish = 1;

  // Debug
  constexpr const char* kDebugImageDir = "/home/user/ros2_ws/src/helper_scripts/pics";
}  // namespace config

// Shape type enumeration
enum class ShapeType {
  kEmpty = 0,
  kCross = 1,
  kCircle = 2,
  kUncertain = -1
};

// ============================================================================
// CameraSubscriber Class
// ============================================================================

class CameraSubscriber : public rclcpp::Node {
 public:
  // ============================================================================
  // Constructor
  // ============================================================================

  explicit CameraSubscriber() : Node("camera_subscriber"),
                                order_done_(true),
                                robot_shape_(0),
                                nb_consecutive_good_frames_(0),
                                pipeline_success_rate_(0.0) {
    initialize_parameters_();
    initialize_subscriptions_();
    initialize_publishers_();
    squares_buffers_.resize(config::kTotalSquares);
  }

private:
  // ============================================================================
  // Initialization Methods
  // ============================================================================

  void initialize_parameters_() {
    this->declare_parameter<std::string>("camera_topic",
                                         config::kDefaultCameraTopic);
  }

  void initialize_subscriptions_() {
    const std::string camera_topic =
        this->get_parameter("camera_topic").as_string();

    auto callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group;

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 10,
        std::bind(&CameraSubscriber::on_image_received_, this, std::placeholders::_1),
        sub_options);

    order_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/take_pic_order", 10,
        std::bind(&CameraSubscriber::on_capture_order_, this,
                  std::placeholders::_1),
        sub_options);

    perception_parameter_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/perception_param", 10,
            std::bind(&CameraSubscriber::on_perception_params_, this,
                      std::placeholders::_1),
            sub_options);

    game_start_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/game_start", 10,
            std::bind(&CameraSubscriber::on_game_start_, this,
                      std::placeholders::_1));
  }

  void initialize_publishers_() {
    board_state_publisher_ =
        this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/take_pic_order_response", 10);
    
    // Debug publishers
    debug_processed_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/perception_server", 10);
  }

  // ============================================================================
  // Callback Methods
  // ============================================================================

  void on_image_received_(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (order_done_) return;

    try {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");
      process_frame_(cv_image->image);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void on_capture_order_(const std_msgs::msg::Bool::SharedPtr msg) {
    order_done_ = !msg->data;
    nb_consecutive_good_frames_ = 0;
    RCLCPP_INFO(this->get_logger(), "Capture order received: %s",
                order_done_ ? "idle" : "processing");
  }

  void on_perception_params_(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
      v_cluster_interval_ = msg->data[0];
      h_cluster_interval_ = msg->data[1];
      RCLCPP_DEBUG(this->get_logger(), 
                   "Updated clustering params: v=%d, h=%d",
                   v_cluster_interval_, h_cluster_interval_);
    }
  }

  void on_game_start_(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (!msg->data.empty()) {
      robot_shape_ = msg->data[0];
      RCLCPP_DEBUG(this->get_logger(), "Robot shape set to: %d", robot_shape_);
    }
  }

  // ============================================================================
  // Main Processing Pipeline
  // ============================================================================

  void process_frame_(const cv::Mat &image) {
    auto [processed_image, board_state] = perceive_game_board_(image);

    if (is_valid_board_(board_state)) {
      publish_board_state_(board_state);
      publish_debug_image_(processed_image);
      update_pipeline_stats_(true);
    } else {
      update_pipeline_stats_(false);
    }
  }

  // ============================================================================
  // Board Perception
  // ============================================================================

  std::pair<cv::Mat, std::vector<int>> perceive_game_board_(cv::Mat image) {
    std::vector<int> board_state(config::kTotalSquares, static_cast<int>(ShapeType::kUncertain));
    grid_vertices_.clear();

    // Step 1: Find tablet contour
    cv::Mat gray, binary;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    remove_shadows_(gray, gray);
    cv::threshold(gray, binary, config::kBinaryThreshold, 255, 0);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary.clone(), contours, cv::RETR_TREE,
                     cv::CHAIN_APPROX_SIMPLE);

    int tablet_idx = find_tablet_contour_(contours);
    if (tablet_idx < 0) {
      RCLCPP_ERROR(this->get_logger(), "No tablet contour found");
      return {image, board_state};
    }

    // Step 2: Perspective correction
    cv::Mat corrected = image.clone();
    apply_perspective_correction_(contours[tablet_idx], image, gray, corrected, gray);

    // Step 3: Detect grid lines and find board squares
    cv::Mat edges = compute_edges_(gray);
    if (!detect_grid_lines_(edges, corrected)) {
      return {corrected, board_state};
    }

    // Step 4: Detect shapes in each square
    detect_all_shapes_(edges, corrected, board_state);

    return {corrected, board_state};
  }

  void remove_shadows_(const cv::Mat &src, cv::Mat &dst) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(config::kMorphKernelSize,
                                                       config::kMorphKernelSize));
    cv::Mat background;
    cv::morphologyEx(src, background, cv::MORPH_DILATE, kernel);
    cv::subtract(background, src, dst);
  }

  int find_tablet_contour_(const std::vector<std::vector<cv::Point>> &contours) {
    for (size_t i = 0; i < contours.size(); ++i) {
      double area = cv::contourArea(contours[i]);
      if (area > config::kTabletContourMinArea && area < config::kTabletContourMaxArea) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  void apply_perspective_correction_(
      const std::vector<cv::Point> &tablet_contour,
      const cv::Mat &color_img, const cv::Mat &gray_img,
      cv::Mat &color_out, cv::Mat &gray_out) {
    // Get bounding box
    int xmin = 1000, ymin = 1000, xmax = 0, ymax = 0;
    for (const auto &pt : tablet_contour) {
      xmin = std::min(xmin, pt.x);
      ymin = std::min(ymin, pt.y);
      xmax = std::max(xmax, pt.x);
      ymax = std::max(ymax, pt.y);
    }

    // Find top-left corner for perspective transform
    int min_sum = INT_MAX;
    int top_left_x = 0, top_left_y = 0;
    for (const auto &pt : tablet_contour) {
      int sum = pt.x + pt.y;
      if (sum < min_sum) {
        min_sum = sum;
        top_left_x = pt.x;
        top_left_y = pt.y;
      }
    }

    std::vector<cv::Point2f> src_pts = {
        cv::Point2f(top_left_x, top_left_y),
        cv::Point2f(xmax - (top_left_x - xmin), top_left_y),
        cv::Point2f(xmax, ymax),
        cv::Point2f(xmin, ymax)};

    std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(xmin, ymin),
        cv::Point2f(xmax, ymin),
        cv::Point2f(xmax, ymax),
        cv::Point2f(xmin, ymax)};

    cv::Mat warp_matrix = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::warpPerspective(color_img, color_out, warp_matrix, color_img.size());
    cv::warpPerspective(gray_img, gray_out, warp_matrix, gray_img.size());

    // Crop to valid region
    int margin = 10;
    int width = xmax - xmin;
    int height = ymax - ymin;
    cv::Rect roi(xmin + margin, ymin + margin, width - 2 * margin,
                height - 2 * margin);
    if (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= color_out.cols &&
        roi.y + roi.height <= color_out.rows) {
      color_out = color_out(roi).clone();
      gray_out = gray_out(roi).clone();
      cv::resize(color_out, color_out, cv::Size(width, static_cast<int>(height * 1.7)));
      cv::resize(gray_out, gray_out, cv::Size(width, static_cast<int>(height * 1.7)));
    }
  }

  cv::Mat compute_edges_(const cv::Mat &gray) {
    cv::Mat processed;
    cv::adaptiveThreshold(gray, processed, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY, config::kAdaptiveThreshBlockSize,
                          config::kAdaptiveThreshConstant);

    cv::Mat hsv;
    cv::cvtColor(processed, hsv, cv::COLOR_GRAY2BGR);
    cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    processed = channels[2].clone();

    cv::GaussianBlur(processed, processed, cv::Size(5, 5), 0);

    // Adaptive Canny
    std::vector<uchar> pixels(processed.begin<uchar>(), processed.end<uchar>());
    std::sort(pixels.begin(), pixels.end());
    double median = pixels[pixels.size() / 2];
    int lower = std::max(0, static_cast<int>(config::kCannyMedianRatio1 * median));
    int upper = std::min(255, static_cast<int>(config::kCannyMedianRatio2 * median));

    cv::Mat edges;
    cv::Canny(processed, edges, lower, upper);
    return edges;
  }

  bool detect_grid_lines_(const cv::Mat &edges, cv::Mat &debug_img) {
    // Detect and cluster lines
    std::vector<cv::Vec4i> v_lines, h_lines;
    cv::HoughLinesP(edges, v_lines, config::kHoughLinesRho,
                    config::kHoughLinesTheta, config::kVerticalLinesThreshold,
                    config::kVerticalLinesMinLength, config::kVerticalLinesMaxGap);
    cv::HoughLinesP(edges, h_lines, config::kHoughLinesRho,
                    config::kHoughLinesTheta, config::kHorizontalLinesThreshold,
                    config::kHorizontalLinesMinLength, config::kHorizontalLinesMaxGap);

    auto v_positions = extract_and_filter_line_positions_(v_lines, true);
    auto h_positions = extract_and_filter_line_positions_(h_lines, false);

    if (v_positions.size() < 4 || h_positions.size() < 4) {
      RCLCPP_ERROR(this->get_logger(),
                   "Insufficient lines detected: v=%zu, h=%zu",
                   v_positions.size(), h_positions.size());
      return false;
    }

    auto v_clustered = cluster_positions_(v_positions, v_cluster_interval_);
    auto h_clustered = cluster_positions_(h_positions, h_cluster_interval_);

    if (v_clustered.size() != 4 || h_clustered.size() != 4) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid cluster counts: v=%zu, h=%zu (need 4 each)",
                   v_clustered.size(), h_clustered.size());
      return false;
    }

    compute_grid_vertices_(v_clustered, h_clustered, edges);
    return true;
  }

  std::vector<int> extract_and_filter_line_positions_(
      const std::vector<cv::Vec4i> &lines, bool is_vertical) {
    std::vector<int> positions;
    const float tolerance = is_vertical ? config::kVerticalAngleTolerance
                                        : config::kHorizontalAngleTolerance;
    const float target_angle = is_vertical ? 90.0f : 0.0f;

    for (const auto &line : lines) {
      float angle = atan2(line[3] - line[1], line[2] - line[0]) * 180.0f / CV_PI;
      if (std::abs(angle - target_angle) < tolerance ||
          std::abs(angle + target_angle) < tolerance) {
        int pos = is_vertical ? (line[0] + line[2]) / 2 : (line[1] + line[3]) / 2;
        positions.push_back(pos);
      }
    }

    std::sort(positions.begin(), positions.end());
    return positions;
  }

  std::vector<int> cluster_positions_(const std::vector<int> &positions,
                                      int cluster_distance) {
    if (positions.empty()) return {};

    std::vector<int> clustered;
    clustered.push_back(positions[0]);

    for (int pos : positions) {
      if (std::abs(pos - clustered.back()) > cluster_distance) {
        clustered.push_back(pos);
      }
    }

    return clustered;
  }

  void compute_grid_vertices_(const std::vector<int> &v_lines,
                              const std::vector<int> &h_lines,
                              const cv::Mat &edges) {
    int y_min = *std::min_element(edges.ptr(), edges.ptr() + edges.total());
    int y_max = *std::max_element(edges.ptr(), edges.ptr() + edges.total());

    grid_vertices_.clear();
    // Find 4 intersections
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        // Simplified: use line positions directly as grid vertices
        grid_vertices_.push_back(cv::Point(v_lines[i * 2 + 1], h_lines[j * 2 + 1]));
      }
    }
  }

  void detect_all_shapes_(const cv::Mat &edges, cv::Mat &color_img,
                          std::vector<int> &board_state) {
    for (int i = 0; i < config::kTotalSquares; ++i) {
      cv::Rect roi = compute_square_roi_(i);

      if (!clamp_rect_to_image_(edges, roi)) {
        board_state[i] = static_cast<int>(ShapeType::kUncertain);
        continue;
      }

      cv::Mat square_edges = edges(roi);
      cv::Mat square_color = color_img(roi);

      auto contour = find_largest_contour_(square_edges);
      if (contour.empty()) {
        board_state[i] = static_cast<int>(ShapeType::kEmpty);
      } else {
        int shape_id = classify_shape_(contour);
        squares_buffers_[i].push_front(shape_id);
        if (squares_buffers_[i].size() > config::kMaxBufferSize) {
          squares_buffers_[i].pop_back();
        }
        board_state[i] = get_consensus_shape_(squares_buffers_[i]);
      }
    }
  }

  cv::Rect compute_square_roi_(int square_index) {
    // Simplified ROI computation - adjust based on your grid geometry
    if (grid_vertices_.size() < 4) return cv::Rect(0, 0, 50, 50);

    int idx = square_index;
    cv::Point p1 = grid_vertices_[0];
    cv::Point p2 = grid_vertices_[3];

    int w = p2.x - p1.x;
    int h = p2.y - p1.y;
    int sq_w = w / config::kGridSize;
    int sq_h = h / config::kGridSize;

    int row = idx / config::kGridSize;
    int col = idx % config::kGridSize;

    return cv::Rect(p1.x + col * sq_w, p1.y + row * sq_h, sq_w, sq_h);
  }

  std::vector<cv::Point> find_largest_contour_(const cv::Mat &image) {
    cv::Mat morph_kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                  cv::Size(config::kMorphEllipseSize,
                                          config::kMorphEllipseSize));
    cv::Mat processed = image.clone();
    cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, morph_kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(processed, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return {};

    auto largest = std::max_element(
        contours.begin(), contours.end(),
        [](const auto &a, const auto &b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });

    return *largest;
  }

  int classify_shape_(const std::vector<cv::Point> &contour) {
    double area = cv::contourArea(contour);
    if (area < config::kMinShapeArea) {
      return static_cast<int>(ShapeType::kEmpty);
    }

    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    double hull_area = cv::contourArea(hull);
    double solidity = hull_area > 0 ? area / hull_area : 0;

    if (solidity < config::kCrossSolidityThreshold) {
      return static_cast<int>(ShapeType::kCross);
    } else if (solidity > config::kCircleSolidityThreshold) {
      return static_cast<int>(ShapeType::kCircle);
    }
    return static_cast<int>(ShapeType::kUncertain);
  }

  int get_consensus_shape_(const std::deque<int> &buffer) {
    if (static_cast<int>(buffer.size()) < config::kMinBufferSize) {
      return static_cast<int>(ShapeType::kUncertain);
    }

    std::map<int, int> votes;
    for (int shape : buffer) {
      if (shape != static_cast<int>(ShapeType::kUncertain)) {
        votes[shape]++;
      }
    }

    int required_votes =
        std::max(3, static_cast<int>(buffer.size() * config::kVoteThresholdRatio));
    for (const auto &[shape, count] : votes) {
      if (count >= required_votes) {
        return shape;
      }
    }
    return static_cast<int>(ShapeType::kUncertain);
  }

  // ============================================================================
  // Publishing & Metrics
  // ============================================================================

  bool is_valid_board_(const std::vector<int> &board_state) {
    for (int state : board_state) {
      if (state == static_cast<int>(ShapeType::kUncertain)) {
        return false;
      }
    }
    return true;
  }

  void publish_board_state_(const std::vector<int> &board_state) {
    int crosses = 0, circles = 0;
    for (int state : board_state) {
      if (state == static_cast<int>(ShapeType::kCross)) crosses++;
      if (state == static_cast<int>(ShapeType::kCircle)) circles++;
    }

    nb_consecutive_good_frames_++;

    bool should_publish = false;
    if (robot_shape_ == static_cast<int>(ShapeType::kCircle) &&
        crosses > circles && nb_consecutive_good_frames_ >= config::kConsecutiveFramesThreshold) {
      should_publish = true;
    } else if (robot_shape_ == static_cast<int>(ShapeType::kCross) &&
               crosses == circles && nb_consecutive_good_frames_ >= config::kConsecutiveFramesThreshold &&
               crosses + circles >= config::kMinShapesToPublish) {
      should_publish = true;
    }

    if (should_publish) {
      std_msgs::msg::Int32MultiArray msg;
      msg.data = board_state;
      board_state_publisher_->publish(msg);
      order_done_ = true;
      RCLCPP_INFO(this->get_logger(), "Board state published");
    }
  }

  void publish_debug_image_(const cv::Mat &image) {
    if (!debug_processed_publisher_) return;
    
    try {
      cv_bridge::CvImage cv_image;
      cv_image.header.stamp = this->now();
      cv_image.encoding = "bgr8";
      cv_image.image = image;
      debug_processed_publisher_->publish(*cv_image.toImageMsg());
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to publish debug image: %s",
                   e.what());
    }
  }

  void update_pipeline_stats_(bool success) {
    pipeline_total_frames_++;
    if (!success) pipeline_failures_++;
    pipeline_success_rate_ =
        static_cast<double>(pipeline_total_frames_ - pipeline_failures_) /
        pipeline_total_frames_;

    if (pipeline_total_frames_ % 30 == 0) {
      RCLCPP_INFO(this->get_logger(), "Pipeline success rate: %.2f%%",
                  pipeline_success_rate_ * 100.0);
    }
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  static bool clamp_rect_to_image_(const cv::Mat &image, cv::Rect &rect) {
    cv::Rect bounds(0, 0, image.cols, image.rows);
    rect &= bounds;
    return rect.width > 0 && rect.height > 0;
  }

  // ============================================================================
  // Member Variables
  // ============================================================================

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr order_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      perception_parameter_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      game_start_subscriber_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      board_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      debug_processed_publisher_;

  // State variables
  bool order_done_;
  int robot_shape_;
  int nb_consecutive_good_frames_;

  // Perception state
  std::vector<cv::Point> grid_vertices_;
  std::vector<std::deque<int>> squares_buffers_;

  // Parameters
  int v_cluster_interval_ = 1;
  int h_cluster_interval_ = 2;

  // Metrics
  int pipeline_total_frames_ = 0;
  int pipeline_failures_ = 0;
  double pipeline_success_rate_;
};  // class CameraSubscriber

// ============================================================================
// Main
// ============================================================================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraSubscriber>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}