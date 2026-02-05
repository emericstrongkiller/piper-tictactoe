#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/image__struct.hpp"
#include "std_msgs/msg/detail/int32__struct.hpp"
#include "std_msgs/msg/detail/int32_multi_array__struct.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstddef>
#include <iterator>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

const double TABLET_CONTOUR_AREA_INTERVAL[2] = {25000, 30000};
int VERTICAL_LINES_X_CLUSTERS_INTERVAL = 1;
int HORIZONTAL_LINES_Y_CLUSTERS_INTERVAL = 2;

// HoughLines Parameters
// vertical lines
const int VERTICAL_VOTES_THRESHOLD = 134;
const int VERTICAL_MINLINELENGTH = 138;
const int VERTICAL_MAXLINEGAP = 21;
// horizontal lines
const int HORIZONTAL_VOTES_THRESHOLD = 151;
const int HORIZONTAL_MINLINELENGTH = 138;
const int HORIZONTAL_MAXLINEGAP = 21;

class CameraSubscriber : public rclcpp::Node {
public:
  CameraSubscriber() : Node("camera_subscriber") {
    // variables init
    perception_params_.data = {3, 3, 0};
    squares_buffers_.resize(9);
    pipeline_occ_counter_ = 0;
    pipeline_success_rate_ = 0.0;
    pipeline_failures_ = 0;
    order_done_ = true;
    nb_right_since_last_piper_order_ = 0;
    grid_lines_done_ = false;

    // game vars
    robot_shape_ = 0;

    // camera topic parameter
    this->declare_parameter<std::string>("camera_topic",
                                         "/camera/D435/color/image_raw");
    std::string camera_topic = this->get_parameter("camera_topic").as_string();

    // Reentrant callback group
    auto callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group;

    // subscribers
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 10,
        std::bind(&CameraSubscriber::listener_callback, this,
                  std::placeholders::_1),
        sub_options);
    order_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/take_pic_order", 10,
        std::bind(&CameraSubscriber::order_subscriber_callback, this,
                  std::placeholders::_1),
        sub_options);
    perception_parameter_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/perception_param", 10,
            std::bind(
                &CameraSubscriber::perception_parameter_subscriber_callback,
                this, std::placeholders::_1),
            sub_options);

    // game start topic sub  to know when to publish to minmax (when
    // robot-shape=X=> publish when as many circles as crosses,   when
    // robot-shape=O=>publish when more cross than circle)
    game_start_subscriber_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/game_start", 10,
            std::bind(&CameraSubscriber::game_start_subscriber_callback, this,
                      std::placeholders::_1));

    // publishers
    order_response_publisher_ =
        this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/take_pic_order_response", 10);

    end_game_grid_publisher =
        this->create_publisher<std_msgs::msg::Int32MultiArray>("/end_game_grid",
                                                               10);

    video_server_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/perception_server", 10);
    cvtColor_server_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/cvtColor_perception_server", 10);
    no_shadow_server_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/no_shadow_perception_server", 10);
    grid_lines_server_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/grid_lines_perception_server", 10);
    roi_image_server_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/roi_image_perception_server", 10);

    test_gray_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/test_gray_standard", 10);
    vertical_filter_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/vertical_filter_perception_server", 10);
    horizontal_filter_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/horizontal_filter_perception_server", 10);

    detected_grid_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/detected_grid_perception_server", 10);
  }

private:
  void order_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "Receiving take Pic order. About to perceive game Grid..");
    order_done_ = !(msg->data);
    // init right counter
    nb_right_since_last_piper_order_ = 0;
  }

  void perception_parameter_subscriber_callback(
      const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    perception_params_.data = msg->data;
    RCLCPP_INFO(this->get_logger(), "perception_params_ are now: %d | %d | %d",
                static_cast<int>(perception_params_.data[0]),
                static_cast<int>(perception_params_.data[1]),
                static_cast<int>(perception_params_.data[2]));
  }

  void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Analyse + output grid state
    if (!order_done_) {
      RCLCPP_DEBUG(this->get_logger(), "Receiving image");

      try {
        cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
      } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      cv_ptr_->encoding = "bgr8";

      current_game_board_ = percieve_game_board(cv_ptr_->image);
      int game_board_quality = 1;

      // print perceived grid
      std::ostringstream oss;
      oss << "current_game_grid: |";
      for (const auto &el : current_game_board_.second) {
        oss << el << "|";
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      // check quality of game board analysis
      for (auto &el : current_game_board_.second) {
        if (el == -1) {
          RCLCPP_ERROR(
              this->get_logger(),
              "Undefined || partially undefined current_game_board => not "
              "publishing anything");
          pipeline_failures_++;
          game_board_quality = 0;
          break;
        }
      }
      if (game_board_quality == 1) {
        // feedback of current processed image Anyway
        cv_ptr_->image = current_game_board_.first;
        cv_ptr_->encoding = "bgr8";
        video_server_publisher_->publish(*cv_ptr_->toImageMsg());

        // PUBLISH TO MINMAX ONLY IF THERE IS A CHANGE TO THE GRID THAT
        // NECESSITATE THE ROBOT TO PLAY
        // get nb or circles &  crosses in the grid
        int nb_cross = 0, nb_circle = 0;
        for (auto &el : current_game_board_.second) {
          if (el == 2) {
            nb_circle++;
          } else if (el == 1) {
            nb_cross++;
          }
        }

        // add right counter and publish only if there is enough right counter
        // since last call
        nb_right_since_last_piper_order_++;

        // ALWAYS PUBLISH CURRENT GAME BOARD TO THE WEB APP
        int winner = check_winner();
        bool is_full = check_full_board();
        RCLCPP_INFO(this->get_logger(),
                    "publishing board game_grid status for webapp...");
        current_game_board_squares_.data = current_game_board_.second;
        end_game_grid_publisher->publish(current_game_board_squares_);
        if (winner != 0 || is_full) {
          RCLCPP_INFO(this->get_logger(),
                      "publishing board game_grid status for webapp...");
          if (current_game_board_.second.size() == 9) {
            current_game_board_.second.push_back(winner);
          }
          current_game_board_squares_.data = current_game_board_.second;
          end_game_grid_publisher->publish(current_game_board_squares_);
          order_done_ = true;
        }

        // if robot plays O this game, publish only if nb_cross>nb_circle ALSO
        // wait for at least 1 shape to be on the board
        if (robot_shape_ == 2 && nb_cross > nb_circle &&
            nb_right_since_last_piper_order_ >= 10 &&
            nb_cross + nb_circle >= 1) {
          RCLCPP_INFO(this->get_logger(), "publishing game_grid status...");
          current_game_board_squares_.data = current_game_board_.second;
          order_response_publisher_->publish(current_game_board_squares_);
          order_done_ = true;
        }

        // if robot plays X this game publish only if nb_cross==nb_circle ALSO
        // wait for at least 1 shape to be on the board
        else if (robot_shape_ == 1 && nb_cross == nb_circle &&
                 nb_right_since_last_piper_order_ >= 10 &&
                 nb_cross + nb_circle >= 1) {
          RCLCPP_INFO(this->get_logger(), "publishing game_grid status...");
          current_game_board_squares_.data = current_game_board_.second;
          order_response_publisher_->publish(current_game_board_squares_);
          order_done_ = true;
        }
      }

      pipeline_occ_counter_++;
      RCLCPP_INFO(this->get_logger(), "pipeline_occ_counter_: %d",
                  pipeline_occ_counter_);
      RCLCPP_INFO(this->get_logger(), "pipeline_failures_: %d",
                  pipeline_failures_);
      pipeline_success_rate_ =
          static_cast<double>(pipeline_occ_counter_ - pipeline_failures_) /
          pipeline_occ_counter_;
      RCLCPP_INFO(this->get_logger(), "Current Pipeline Success rate: %f",
                  pipeline_success_rate_);
    }
  }

  std::pair<cv::Mat, std::vector<int>> percieve_game_board(cv::Mat image) {
    cv::Mat processed_image;
    std::vector<int> game_board_grid_state(
        9, -1); // start grid with undefined board squares

    grid_center_vertices_.clear();

    // gray image
    cv::cvtColor(image, processed_image, cv::COLOR_BGR2GRAY);
    cv_ptr_->image = processed_image;
    cv_ptr_->encoding = "mono8";
    cvtColor_server_publisher_->publish(*cv_ptr_->toImageMsg());

    // shadow removal
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat background;
    cv::morphologyEx(processed_image, background, cv::MORPH_DILATE, kernel);
    cv::Mat shadow_removed;
    cv::subtract(background, processed_image, shadow_removed);
    cv_ptr_->image = shadow_removed;
    cv_ptr_->encoding = "mono8";
    no_shadow_server_publisher_->publish(*cv_ptr_->toImageMsg());

    // binary image
    cv::Mat binary_filtered;
    cv::threshold(shadow_removed, binary_filtered, 25, 255, 0);
    // contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_filtered, contours, hierarchy, cv::RETR_TREE,
                     cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_DEBUG(this->get_logger(), "Found %zu contours", contours.size());
    // find the tablet contour
    double tablet_contour_index = -1;
    for (size_t i = 0; i < contours.size(); i++) {
      if (cv::contourArea(contours[i]) > TABLET_CONTOUR_AREA_INTERVAL[0] &&
          cv::contourArea(contours[i]) < TABLET_CONTOUR_AREA_INTERVAL[1]) {
        tablet_contour_index = i;
      }
      RCLCPP_DEBUG(this->get_logger(), "contour[%zu].area: %f", i,
                   cv::contourArea(contours[i]));
    }
    if (tablet_contour_index == -1) {
      RCLCPP_ERROR(this->get_logger(), "No tablet Contour found, returning...");
      cv_ptr_->image = image;
      cv_ptr_->encoding = "bgr8";
      roi_image_server_publisher_->publish(*cv_ptr_->toImageMsg());

      save_image(image, "teeeeest.png");

      return std::pair<cv::Mat, std::vector<int>>(image, game_board_grid_state);
    }
    // find the two extremities of the tablet to crop the image accordingly
    int x_start = 1000, y_start = 1000, x_end = 0, y_end = 0, width = 0,
        height = 0;
    for (size_t i = 0; i < contours[tablet_contour_index].size(); i++) {
      if (contours[tablet_contour_index][i].x < x_start) {
        x_start = contours[tablet_contour_index][i].x;
      }
      if (contours[tablet_contour_index][i].y < y_start) {
        y_start = contours[tablet_contour_index][i].y;
      }
      if (contours[tablet_contour_index][i].x > x_end) {
        x_end = contours[tablet_contour_index][i].x;
      }
      if (contours[tablet_contour_index][i].y > y_end) {
        y_end = contours[tablet_contour_index][i].y;
      }
    }
    width = x_end - x_start;
    height = y_end - y_start;
    RCLCPP_DEBUG(this->get_logger(),
                 "x_start: %d, y_start: %d, x_end: %d, y_end: %d", x_start,
                 y_start, x_end, y_end);

    // find the actual two top corners of the tablet (since its warped, not a
    // perfect rectangle so yeah)
    int smallest_point_sum = 1000;
    int top_left_x = 0, top_left_y = 0, top_right_x = 0, top_right_y = 0,
        bottom_left_x = 0, bottom_left_y = 0;
    for (size_t i = 0; i < contours[tablet_contour_index].size(); i++) {
      if (contours[tablet_contour_index][i].x +
              contours[tablet_contour_index][i].y <
          smallest_point_sum) {
        smallest_point_sum = contours[tablet_contour_index][i].x +
                             contours[tablet_contour_index][i].y;
        top_left_x = contours[tablet_contour_index][i].x;
        top_left_y = contours[tablet_contour_index][i].y;
      }
    }
    top_right_x = x_end - (top_left_x - x_start);
    top_right_y = top_left_y;

    bottom_left_x = x_start;
    bottom_left_y = y_end;

    RCLCPP_DEBUG(
        this->get_logger(),
        "top_left_x: %d, top_left_y: %d, top_right_x: %d, top_right_y: %d",
        top_left_x, top_left_y, top_right_x, top_right_y);

    // Perspective Transformation
    cv::Mat warp_matrix;

    std::vector<cv::Point2f> srcQuad(4), dstQuad(4);

    srcQuad[0] = cv::Point2f(top_left_x, top_left_y);
    srcQuad[1] = cv::Point2f(top_right_x, top_right_y);
    srcQuad[2] = cv::Point2f(x_end, y_end);
    srcQuad[3] = cv::Point2f(bottom_left_x, bottom_left_y);

    dstQuad[0] = cv::Point2f(x_start + 0, y_start);
    dstQuad[1] = cv::Point2f(x_end - 0, y_start);
    dstQuad[2] = cv::Point2f(x_end, y_end);
    dstQuad[3] = cv::Point2f(bottom_left_x, bottom_left_y);

    warp_matrix = cv::getPerspectiveTransform(srcQuad, dstQuad);
    cv::warpPerspective(shadow_removed, shadow_removed, warp_matrix,
                        shadow_removed.size());
    cv::warpPerspective(image, image, warp_matrix, image.size());

    //    cv::drawContours(image, contours, tablet_contour_index,
    //                     cv::Scalar(0, 255, 0), 3);

    // Crop the image using ROI
    cv::Rect roi(x_start + 10, y_start + 10, width - 20, height - 20);
    cv::Mat shadow_removed_cropped = shadow_removed(roi);
    cv::Mat img_cropped = image(roi);

    cv::resize(shadow_removed_cropped, shadow_removed_cropped,
               cv::Size(width, height * 1.7), 0, 0, cv::INTER_LINEAR);
    cv::resize(img_cropped, img_cropped, cv::Size(width, height * 1.7), 0, 0,
               cv::INTER_LINEAR);

    cv::Mat img_cropped_clustered_lines = img_cropped.clone();
    cv::Mat img_cropped_shape_detec = img_cropped.clone();

    // adaptive filter on roi
    // cv::threshold(shadow_removed_cropped, shadow_removed_cropped, 11, 255,
    // 0);
    cv::adaptiveThreshold(shadow_removed_cropped, shadow_removed_cropped, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11,
                          -3);

    // FOR Grid LINES Adaptive Canny TESTTT ###############################

    RCLCPP_DEBUG(this->get_logger(), "GOT TO LINE DETEC");

    cv::Mat processed_imaged;

    // Convert to HSV
    cv::Mat hsv_image;
    cv::cvtColor(img_cropped, hsv_image, cv::COLOR_BGR2HSV);

    // Split channels
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv_image, hsv_channels);

    // Try Value channel first (most similar to grayscale)
    processed_imaged = hsv_channels[2].clone();

    // Publish for debugging
    cv_ptr_->image = hsv_channels[2]; // V channel
    cv_ptr_->encoding = "mono8";
    test_gray_publisher_->publish(*cv_ptr_->toImageMsg());

    cv::Mat lines_processed_image = processed_imaged.clone();

    // Continue with Gaussian blur and Canny...
    cv::GaussianBlur(processed_imaged, processed_imaged, cv::Size(5, 5), 0);

    // Get median pixel for adaptive Canny Thresholds
    std::vector<uchar> pixels(processed_imaged.begin<uchar>(),
                              processed_imaged.end<uchar>());
    std::sort(pixels.begin(), pixels.end());
    double median = pixels[pixels.size() / 2];
    int lower_threshold = std::max(0, static_cast<int>(0.66 * median));
    int upper_threshold = std::min(255, static_cast<int>(1.33 * median));

    // Apply Canny
    cv::Mat edges;
    cv::Canny(processed_imaged, edges, lower_threshold, upper_threshold);

    // Apply Sobel for the lines detection
    // some light inv treatment:
    cv::GaussianBlur(lines_processed_image, lines_processed_image,
                     cv::Size(5, 5), 0);
    auto clahe = cv::createCLAHE(1.5, cv::Size(8, 8));
    clahe->apply(lines_processed_image, lines_processed_image);

    cv::Mat v_sobel;
    cv::Mat h_sobel;
    cv::Sobel(lines_processed_image, v_sobel, CV_16S, 1, 0,
              3); // vertical sobel
    cv::Sobel(lines_processed_image, h_sobel, CV_16S, 0, 1,
              3); // horizontal sobel
    cv::convertScaleAbs(v_sobel, v_sobel);
    cv::convertScaleAbs(h_sobel, h_sobel);

    cv::threshold(v_sobel, v_sobel, 30, 255, cv::THRESH_BINARY);
    cv::threshold(h_sobel, h_sobel, 30, 255, cv::THRESH_BINARY);

    cv::Mat kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
    cv::Mat v_kernel_ =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));

    cv::dilate(v_sobel, v_sobel, v_kernel_);
    cv::dilate(h_sobel, h_sobel, kernel_);

    cv_ptr_->image = v_sobel;
    cv_ptr_->encoding = "mono8";
    vertical_filter_publisher_->publish(*cv_ptr_->toImageMsg());

    cv_ptr_->image = h_sobel;
    cv_ptr_->encoding = "mono8";
    horizontal_filter_publisher_->publish(*cv_ptr_->toImageMsg());

    // GRID LINES DETECTION ######################################

    if (!grid_lines_done_) {
      // Detect vertical lines first
      std::vector<cv::Vec4i> v_lines;
      cv::HoughLinesP(v_sobel, v_lines, 1, CV_PI / 180,
                      VERTICAL_VOTES_THRESHOLD, VERTICAL_MINLINELENGTH,
                      VERTICAL_MAXLINEGAP);

      // filter vertical
      std::vector<cv::Vec4i> v_lines_full;
      for (auto &l : v_lines) {
        float angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
        if (abs(angle - 90) < 20 || abs(angle + 90) < 20) {
          v_lines_full.push_back(l);
          cv::line(img_cropped, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                   cv::Scalar(0, 255, 0), 1, 8);
        }
      }

      // Detect horizontal lines after
      std::vector<cv::Vec4i> h_lines;
      cv::HoughLinesP(h_sobel, h_lines, 1, (CV_PI / 180),
                      HORIZONTAL_VOTES_THRESHOLD, HORIZONTAL_MINLINELENGTH,
                      HORIZONTAL_MAXLINEGAP);
      // filter horizontal
      std::vector<cv::Vec4i> h_lines_full;
      for (auto &l : h_lines) {
        float angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
        if (abs(angle) < 10 || abs(angle) > 170) {
          h_lines_full.push_back(l);
          cv::line(img_cropped, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                   cv::Scalar(0, 0, 255), 1, 8);
        }
      }

      cv_ptr_->image = img_cropped;
      cv_ptr_->encoding = "bgr8";
      grid_lines_server_publisher_->publish(*cv_ptr_->toImageMsg());

      //#########################################################
      // VERTICAL LINES CLUSTERING
      //#########################################################
      std::vector<int> v_line_x_positions;
      std::vector<int> v_line_y_positions;
      int y_vert_min, y_vert_max;

      // Safety check: need vertical lines
      if (v_lines_full.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "No vertical lines detected. Returning...");
        return std::pair<cv::Mat, std::vector<int>>(img_cropped,
                                                    game_board_grid_state);
      }

      for (auto &l : v_lines_full) {
        v_line_x_positions.push_back(static_cast<int>((l[0] + l[2]) / 2.0f));
        RCLCPP_DEBUG(this->get_logger(),
                     "pushed back: %d -> v_line_x_positions",
                     static_cast<int>((l[0] + l[2]) / 2.0f));
      }

      std::sort(v_line_x_positions.begin(), v_line_x_positions.end());

      // Cluster detection with configurable threshold
      const int CLUSTER_THRESHOLD = 20;
      std::vector<std::vector<int>> clusters;
      std::vector<int> current_cluster = {v_line_x_positions[0]};

      for (size_t i = 1; i < v_line_x_positions.size(); i++) {
        if (abs(v_line_x_positions[i] - v_line_x_positions[i - 1]) <=
            CLUSTER_THRESHOLD) {
          // Same cluster
          current_cluster.push_back(v_line_x_positions[i]);
        } else {
          // New cluster detected
          clusters.push_back(current_cluster);
          current_cluster = {v_line_x_positions[i]};
        }
      }
      // Don't forget the last cluster
      clusters.push_back(current_cluster);

      RCLCPP_DEBUG(this->get_logger(), "Detected %zu clusters",
                   clusters.size());

      // Validate: we need exactly 2 clusters (left and right board edges)
      if (clusters.size() != 2) {
        RCLCPP_ERROR(this->get_logger(),
                     "Expected 2 vertical line clusters, found %zu. Noisy "
                     "detection, returning...",
                     clusters.size());
        return std::pair<cv::Mat, std::vector<int>>(img_cropped,
                                                    game_board_grid_state);
      }

      // Get min and max from each cluster
      int min_v_left =
          *std::min_element(clusters[0].begin(), clusters[0].end());
      int max_v_left =
          *std::max_element(clusters[0].begin(), clusters[0].end());
      int min_v_right =
          *std::min_element(clusters[1].begin(), clusters[1].end());
      int max_v_right =
          *std::max_element(clusters[1].begin(), clusters[1].end());

      // Compute median (center) position for each cluster
      int median_v_left = (min_v_left + max_v_left) / 2;
      int median_v_right = (min_v_right + max_v_right) / 2;

      RCLCPP_DEBUG(this->get_logger(), "Left cluster: %zu lines, median=%d",
                   clusters[0].size(), median_v_left);
      RCLCPP_DEBUG(this->get_logger(), "Right cluster: %zu lines, median=%d",
                   clusters[1].size(), median_v_right);

      // Get max and min y
      for (auto &l : v_lines_full) {
        v_line_y_positions.push_back(l[1]);
        v_line_y_positions.push_back(l[3]);
      }
      y_vert_min = *std::min_element(v_line_y_positions.begin(),
                                     v_line_y_positions.end());
      y_vert_max = *std::max_element(v_line_y_positions.begin(),
                                     v_line_y_positions.end());

      RCLCPP_DEBUG(this->get_logger(), "y_vert_min: %d", y_vert_min);
      RCLCPP_DEBUG(this->get_logger(), "y_vert_max: %d", y_vert_max);

      // Draw vertical lines
      cv::line(
          img_cropped_clustered_lines, cv::Point(median_v_left, y_vert_min),
          cv::Point(median_v_left, y_vert_max), cv::Scalar(0, 0, 255), 1, 8);
      cv::line(
          img_cropped_clustered_lines, cv::Point(median_v_right, y_vert_min),
          cv::Point(median_v_right, y_vert_max), cv::Scalar(0, 0, 255), 1, 8);

      //#########################################################
      // HORIZONTAL LINES CLUSTERING
      //#########################################################
      std::vector<int> h_line_y_positions;
      std::vector<int> h_line_x_positions;
      int x_horiz_min, x_horiz_max;

      // Safety check: need horizontal lines
      if (h_lines_full.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "No horizontal lines detected. Returning...");
        return std::pair<cv::Mat, std::vector<int>>(img_cropped,
                                                    game_board_grid_state);
      }

      for (auto &l : h_lines_full) {
        h_line_y_positions.push_back(static_cast<int>((l[1] + l[3]) / 2.0f));
        RCLCPP_DEBUG(this->get_logger(),
                     "pushed back: %d -> h_line_y_positions",
                     static_cast<int>((l[1] + l[3]) / 2.0f));
      }

      std::sort(h_line_y_positions.begin(), h_line_y_positions.end());

      // Cluster detection with configurable threshold
      const int H_CLUSTER_THRESHOLD = 20;
      std::vector<std::vector<int>> h_clusters;
      std::vector<int> current_h_cluster = {h_line_y_positions[0]};

      for (size_t i = 1; i < h_line_y_positions.size(); i++) {
        if (abs(h_line_y_positions[i] - h_line_y_positions[i - 1]) <=
            H_CLUSTER_THRESHOLD) {
          // Same cluster
          current_h_cluster.push_back(h_line_y_positions[i]);
        } else {
          // New cluster detected
          h_clusters.push_back(current_h_cluster);
          current_h_cluster = {h_line_y_positions[i]};
        }
      }
      // Don't forget the last cluster
      h_clusters.push_back(current_h_cluster);

      RCLCPP_DEBUG(this->get_logger(), "Detected %zu horizontal clusters",
                   h_clusters.size());

      // Validate: we need exactly 2 clusters (top and bottom board edges)
      if (h_clusters.size() != 2) {
        RCLCPP_ERROR(this->get_logger(),
                     "Expected 2 horizontal line clusters, found %zu. Noisy "
                     "detection, returning...",
                     h_clusters.size());
        return std::pair<cv::Mat, std::vector<int>>(img_cropped,
                                                    game_board_grid_state);
      }

      // Get min and max from each cluster
      int min_h_top =
          *std::min_element(h_clusters[0].begin(), h_clusters[0].end());
      int max_h_top =
          *std::max_element(h_clusters[0].begin(), h_clusters[0].end());
      int min_h_bottom =
          *std::min_element(h_clusters[1].begin(), h_clusters[1].end());
      int max_h_bottom =
          *std::max_element(h_clusters[1].begin(), h_clusters[1].end());

      // Compute median (center) position for each cluster
      int median_h_top = (min_h_top + max_h_top) / 2;
      int median_h_bottom = (min_h_bottom + max_h_bottom) / 2;

      RCLCPP_DEBUG(this->get_logger(), "Top cluster: %zu lines, median=%d",
                   h_clusters[0].size(), median_h_top);
      RCLCPP_DEBUG(this->get_logger(), "Bottom cluster: %zu lines, median=%d",
                   h_clusters[1].size(), median_h_bottom);

      // Get max and min x
      for (auto &l : h_lines_full) {
        h_line_x_positions.push_back(l[0]);
        h_line_x_positions.push_back(l[2]);
      }
      x_horiz_min = *std::min_element(h_line_x_positions.begin(),
                                      h_line_x_positions.end());
      x_horiz_max = *std::max_element(h_line_x_positions.begin(),
                                      h_line_x_positions.end());

      RCLCPP_DEBUG(this->get_logger(), "x_horiz_min: %d", x_horiz_min);
      RCLCPP_DEBUG(this->get_logger(), "x_horiz_max: %d", x_horiz_max);

      // Draw horizontal lines
      cv::line(
          img_cropped_clustered_lines, cv::Point(x_horiz_min, median_h_top),
          cv::Point(x_horiz_max, median_h_top), cv::Scalar(0, 0, 255), 1, 8);
      cv::line(
          img_cropped_clustered_lines, cv::Point(x_horiz_min, median_h_bottom),
          cv::Point(x_horiz_max, median_h_bottom), cv::Scalar(0, 0, 255), 1, 8);

      // Find and Draw Intersections
      // top->right|bot_left->right
      grid_center_vertices_.push_back(
          find_lines_intersection(cv::Point(x_horiz_min, median_h_top),
                                  cv::Point(x_horiz_max, median_h_top),
                                  cv::Point(median_v_left, y_vert_min),
                                  cv::Point(median_v_left, y_vert_max)));
      grid_center_vertices_.push_back(
          find_lines_intersection(cv::Point(x_horiz_min, median_h_top),
                                  cv::Point(x_horiz_max, median_h_top),
                                  cv::Point(median_v_right, y_vert_min),
                                  cv::Point(median_v_right, y_vert_max)));
      grid_center_vertices_.push_back(
          find_lines_intersection(cv::Point(x_horiz_min, median_h_bottom),
                                  cv::Point(x_horiz_max, median_h_bottom),
                                  cv::Point(median_v_left, y_vert_min),
                                  cv::Point(median_v_left, y_vert_max)));
      grid_center_vertices_.push_back(
          find_lines_intersection(cv::Point(x_horiz_min, median_h_bottom),
                                  cv::Point(x_horiz_max, median_h_bottom),
                                  cv::Point(median_v_right, y_vert_min),
                                  cv::Point(median_v_right, y_vert_max)));
      cv::circle(img_cropped_clustered_lines, grid_center_vertices_[0], 3,
                 cv::Scalar(0, 0, 255));
      cv::circle(img_cropped_clustered_lines, grid_center_vertices_[1], 3,
                 cv::Scalar(0, 0, 255));
      cv::circle(img_cropped_clustered_lines, grid_center_vertices_[2], 3,
                 cv::Scalar(0, 0, 255));
      cv::circle(img_cropped_clustered_lines, grid_center_vertices_[3], 3,
                 cv::Scalar(0, 0, 255));

      cv_ptr_->image = img_cropped_clustered_lines;
      cv_ptr_->encoding = "bgr8";
      detected_grid_publisher_->publish(*cv_ptr_->toImageMsg());

      // CENTER SQUARE INFOS ################################################

      // calculate grid center
      cv::Point grid_center;
      grid_center.x =
          (grid_center_vertices_[0].x + grid_center_vertices_[3].x) / 2;
      grid_center.y =
          (grid_center_vertices_[0].y + grid_center_vertices_[3].y) / 2;
      center_square_width_ =
          grid_center_vertices_[3].x - grid_center_vertices_[0].x;
      center_square_height_ =
          grid_center_vertices_[3].y - grid_center_vertices_[0].y;

      // MARK GRID DETECTION AS DONE
      grid_lines_done_ = true;
    }

    // SHAPE DETECTIONS  ################################################

    for (int i = 0; i < 9; i++) {
      RCLCPP_DEBUG(this->get_logger(), "At step %d in the shape detec loop", i);
      // square 1 contour process
      // generate square ROI
      cv::Rect square_roi = generate_square_roi(i, 5, 5, 1, 1, 6);

      if (!clampRectToMat(edges, square_roi)) {
        RCLCPP_WARN(
            this->get_logger(),
            "Invalid ROI for square %d: x=%d y=%d w=%d h=%d (img %dx%d)", i,
            square_roi.x, square_roi.y, square_roi.width, square_roi.height,
            edges.cols, edges.rows);
        game_board_grid_state[i] = -1; // or 0
        continue;
      }

      cv::Mat square = edges(square_roi);
      cv::Mat color_square = img_cropped_shape_detec(square_roi);

      std::string textt = std::to_string(i);
      //    cv::circle(color_square,
      //               cv::Point(color_square.cols / 2, color_square.rows / 2),
      //               3, cv::Scalar(0, 255, 255));
      cv::putText(
          color_square, textt,
          cv::Point(color_square.cols / 2 + 10, color_square.rows / 2 + 10),
          cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

      cv::rectangle(color_square, cv::Point(1, 1),
                    cv::Point(color_square.cols - 2, color_square.rows - 2),
                    cv::Scalar(255, 0, 0), 1, cv::LINE_8);

      int shape_id = -1;

      // draw biggest contour if one was detected
      std::vector<cv::Point> biggest_contour = find_biggest_contour(square);
      if (static_cast<int>(biggest_contour.size()) > 0) {
        std::vector<std::vector<cv::Point>> tmp{biggest_contour};
        cv::drawContours(color_square, tmp, 0,
                         cv::Scalar(0, 127, 127 + pow(-1, i) * 127), 1);
        // detect shape
        shape_id = detect_shape(biggest_contour, square, 20);

        // Buffer analysis based on votes for the current square
        squares_buffers_[i].push_front(shape_id);
        print_buffer(squares_buffers_[i]);

        if (squares_buffers_[i].size() > 15) {
          squares_buffers_[i].pop_back();
        }
        // Vote oftennn, with adaptive thresh
        int min_buffer_size = 10;
        if (static_cast<int>(squares_buffers_[i].size()) >= min_buffer_size) {
          // Dynamic vote threshold
          int required_votes =
              std::max(3, static_cast<int>(squares_buffers_[i].size() * 0.6));
          int vote = get_buffer_vote(squares_buffers_[i], required_votes);
          shape_id = vote;
        } else {
          shape_id = -1;
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "No Shape (no contours)");
        shape_id = 0;
      }
      draw_detected_shape(color_square, shape_id);
      game_board_grid_state[i] = shape_id;
    }

    return std::pair<cv::Mat, std::vector<int>>(img_cropped_shape_detec,
                                                game_board_grid_state);
  }

  void save_image(cv::Mat image, std::string file_name) {
    const std::string dir = "/home/user/ros2_ws/src/helper_scripts/pics";
    const std::string save_path =
        (std::filesystem::path(dir) / file_name).string();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    if (!cv::imwrite(save_path, image)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write image to %s",
                   save_path.c_str());
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Image saved to %s", save_path.c_str());
  }

  cv::Point find_lines_intersection(cv::Point a1, cv::Point a2, cv::Point b1,
                                    cv::Point b2) {
    float x1 = a1.x, y1 = a1.y, x2 = a2.x, y2 = a2.y;
    float x3 = b1.x, y3 = b1.y, x4 = b2.x, y4 = b2.y;

    float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::abs(denom) < 1e-6f)
      return {-1, -1};

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    float u = -(((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom);

    if (t < 0.f || t > 1.f || u < 0.f || u > 1.f)
      return {-1, -1};

    return {static_cast<int>(x1 + t * (x2 - x1)),
            static_cast<int>(y1 + t * (y2 - y1))};
  }

  cv::Rect generate_square_roi(int square_index, int offset_from_center_vertice,
                               int corners_additional_area, int center_offset,
                               int sides_offset, int sides_length_offset) {
    int x, y, w, h;

    switch (square_index) {

    case 0: // bottom-left corner
      x = std::max(0, grid_center_vertices_[2].x - offset_from_center_vertice -
                          center_square_width_ - corners_additional_area);
      y = grid_center_vertices_[2].y + offset_from_center_vertice;
      w = center_square_width_ + corners_additional_area;
      h = center_square_height_ + corners_additional_area;
      break;

    case 1: // bottom-middle
      x = grid_center_vertices_[2].x + sides_offset;
      y = grid_center_vertices_[2].y + sides_offset;
      w = grid_center_vertices_[3].x - grid_center_vertices_[2].x -
          2 * sides_offset;
      h = center_square_height_ + sides_length_offset;
      break;

    case 2: // bottom-right corner
      x = grid_center_vertices_[3].x + offset_from_center_vertice;
      y = grid_center_vertices_[3].y + offset_from_center_vertice;
      w = center_square_width_ + corners_additional_area;
      h = center_square_height_ + corners_additional_area;
      break;

    case 3: // middle-left
      x = std::max(0, grid_center_vertices_[0].x - sides_offset -
                          center_square_width_ - sides_length_offset);
      y = grid_center_vertices_[0].y + sides_offset;
      w = center_square_width_ + sides_length_offset;
      h = grid_center_vertices_[2].y - grid_center_vertices_[0].y -
          2 * sides_offset;
      break;

    case 4: // center
      x = grid_center_vertices_[0].x + center_offset;
      y = grid_center_vertices_[0].y + center_offset;
      w = grid_center_vertices_[3].x - grid_center_vertices_[0].x -
          2 * center_offset;
      h = grid_center_vertices_[3].y - grid_center_vertices_[0].y -
          2 * center_offset;
      break;

    case 5: // middle-right
      x = grid_center_vertices_[1].x + sides_offset;
      y = grid_center_vertices_[1].y + sides_offset;
      w = center_square_width_ + sides_length_offset;
      h = grid_center_vertices_[3].y - grid_center_vertices_[1].y -
          2 * sides_offset;
      break;

    case 6: // top-left corner
      x = std::max(0, grid_center_vertices_[0].x - offset_from_center_vertice -
                          center_square_width_ - corners_additional_area);
      y = grid_center_vertices_[0].y - offset_from_center_vertice -
          center_square_height_ - corners_additional_area;
      w = center_square_width_ + corners_additional_area;
      h = center_square_height_ + corners_additional_area;
      break;

    case 7: // top-middle
      x = grid_center_vertices_[0].x + sides_offset;
      y = grid_center_vertices_[0].y - sides_offset - center_square_height_ -
          sides_length_offset;
      w = grid_center_vertices_[1].x - grid_center_vertices_[0].x -
          2 * sides_offset;
      h = center_square_height_ + sides_length_offset;
      break;

    case 8: // top-right corner
      x = grid_center_vertices_[1].x + offset_from_center_vertice;
      y = grid_center_vertices_[1].y - offset_from_center_vertice -
          center_square_height_ - corners_additional_area;
      w = center_square_width_ + corners_additional_area;
      h = center_square_height_ + corners_additional_area;
      break;

    default:
      x = 10;
      y = 10;
      w = 10;
      h = 10;
      break;
    }

    // Safety check: ensure positive dimensions
    if (w <= 0 || h <= 0 || x < 0 || y < 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid ROI for square %d: x=%d y=%d w=%d h=%d",
                  square_index, x, y, w, h);
      return cv::Rect(10, 10, 10, 10);
    }

    return cv::Rect(x, y, w, h);
  }

  std::vector<cv::Point> find_biggest_contour(cv::Mat square_of_interest) {
    std::vector<std::vector<cv::Point>> contours;

    // 1. Close gaps → merge fragments into ONE contour
    cv::Mat kernel_cropped =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
    cv::morphologyEx(square_of_interest, square_of_interest, cv::MORPH_CLOSE,
                     kernel_cropped);

    // 2. Now find contours
    cv::findContours(square_of_interest, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // If no contours found, no shape !
    if (contours.size() == 0) {
      return std::vector<cv::Point>();
    }

    // If come contours, draw BIGGEST contour on the chosen square
    double biggest_area = 0;
    double biggest_area_index = 0;

    for (size_t i = 0; i < contours.size(); i++) {
      if (cv::contourArea(contours[i]) > biggest_area) {
        biggest_area = cv::contourArea(contours[i]);
        biggest_area_index = i;
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "biggest contour area: %f", biggest_area);
    return contours[biggest_area_index];
  }

  int detect_shape(std::vector<cv::Point> contour, const cv::Mat &image,
                   int center_threshold = 20) {
    double area = cv::contourArea(contour);
    if (area < 400) {
      RCLCPP_DEBUG(this->get_logger(), "No Shape");
      return 0;
    }

    RCLCPP_DEBUG(this->get_logger(), "area:%f", area);

    cv::Rect bbox = cv::boundingRect(contour);
    int cx = bbox.x + bbox.width / 2;
    int cy = bbox.y + bbox.height / 2;

    cv::Rect roi(cx - 5, cy - 5, 10, 10);
    roi &= cv::Rect(0, 0, image.cols, image.rows);

    int white_pixels = cv::countNonZero(image(roi));

    RCLCPP_DEBUG(this->get_logger(), "white_pixels: %d, threshold: %d",
                 white_pixels, center_threshold);

    if (white_pixels > center_threshold) {
      RCLCPP_DEBUG(this->get_logger(), "Cross");
      return 1;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Circle");
      return 2;
    }
  }

  void print_buffer(const std::deque<int> &buffer) {
    std::ostringstream oss;
    oss << "buffer: |";

    for (const auto &el : buffer) {
      oss << el << "|";
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  int get_buffer_vote(const std::deque<int> &buffer, int acceptance_threshold) {
    std::map<int, int> vote_counts;

    // Count all shape occurrences
    for (const auto &el : buffer) {
      if (el != -1) { // Ignore uncertain detections
        vote_counts[el]++;
      }
    }

    // Find the shape with the most votes
    int best_shape = -1;
    int max_votes = 0;
    for (const auto &[shape, count] : vote_counts) {
      if (count >= acceptance_threshold && count > max_votes) {
        max_votes = count;
        best_shape = shape;
      }
    }

    return best_shape;
  }

  void draw_detected_shape(cv::Mat &img, int shape_id) {
    const char *text = (shape_id == 2)   ? "O"
                       : (shape_id == 1) ? "X"
                       : (shape_id == 0) ? "-"
                                         : "?";

    int font = cv::FONT_HERSHEY_PLAIN;
    double scale = 0.8;
    int thickness = 1;
    cv::Point org((img.cols) / 2, (img.rows) / 2);
    cv::Scalar color = (shape_id == 2 || shape_id == 1)
                           ? cv::Scalar(255, 255, 255)
                       : (shape_id == 0) ? cv::Scalar(0, 255, 0)
                                         : cv::Scalar(0, 0, 255);

    cv::putText(img, text, org, font, scale, color, thickness, cv::LINE_AA);
  }

  static bool clampRectToMat(const cv::Mat &m, cv::Rect &r) {
    cv::Rect bounds(0, 0, m.cols, m.rows);
    r &= bounds; // intersection
    return r.width > 0 && r.height > 0;
  }

  void game_start_subscriber_callback(
      const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    robot_shape_ = msg->data[0];
  }

  void update_2D_board() {
    current_game_board_2D_ = {
        {current_game_board_.second[6], current_game_board_.second[7],
         current_game_board_.second[8]},
        {current_game_board_.second[3], current_game_board_.second[4],
         current_game_board_.second[5]},
        {current_game_board_.second[0], current_game_board_.second[1],
         current_game_board_.second[2]}};
  }

  int check_winner() {
    update_2D_board(); // Convert 1D to 2D first

    // Check rows (horizontal)
    for (int i = 0; i < 3; i++) {
      if (current_game_board_2D_[i][0] == current_game_board_2D_[i][1] &&
          current_game_board_2D_[i][1] == current_game_board_2D_[i][2] &&
          current_game_board_2D_[i][0] != 0) {
        return current_game_board_2D_[i][0];
      }
    }

    // Check columns (vertical)
    for (int j = 0; j < 3; j++) {
      if (current_game_board_2D_[0][j] == current_game_board_2D_[1][j] &&
          current_game_board_2D_[1][j] == current_game_board_2D_[2][j] &&
          current_game_board_2D_[0][j] != 0) {
        return current_game_board_2D_[0][j];
      }
    }

    // Check diagonal (top-left to bottom-right)
    if (current_game_board_2D_[0][0] == current_game_board_2D_[1][1] &&
        current_game_board_2D_[1][1] == current_game_board_2D_[2][2] &&
        current_game_board_2D_[0][0] != 0) {
      return current_game_board_2D_[0][0];
    }

    // Check diagonal (top-right to bottom-left)
    if (current_game_board_2D_[0][2] == current_game_board_2D_[1][1] &&
        current_game_board_2D_[1][1] == current_game_board_2D_[2][0] &&
        current_game_board_2D_[0][2] != 0) {
      return current_game_board_2D_[0][2];
    }

    return 0; // No winner
  }

  bool check_full_board() {
    update_2D_board();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (current_game_board_2D_[i][j] == 0) {
          return false;
        }
      }
    }
    return true;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr order_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      order_response_publisher_;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      end_game_grid_publisher;
  // perception image servers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_server_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      cvtColor_server_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      no_shadow_server_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      grid_lines_server_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      roi_image_server_publisher_;
  // perception para subscriber
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      perception_parameter_subscriber_;

  // Image variables
  cv_bridge::CvImagePtr cv_ptr_;
  std::pair<cv::Mat, std::vector<int>> current_game_board_;
  std::vector<cv::Point> grid_centers_;
  std::pair<std::array<float, 2>, cv::Point> center_square_infos_;
  std_msgs::msg::Int32MultiArray current_game_board_squares_;
  bool order_done_;
  std_msgs::msg::Int32MultiArray perception_params_;
  std::vector<cv::Point> grid_center_vertices_;
  int center_square_width_;
  int center_square_height_;
  std::vector<std::deque<int>>
      squares_buffers_; // buffers for more reliable square shape detection
  // % of perception pipeline failure
  int pipeline_occ_counter_;
  int pipeline_failures_;
  double pipeline_success_rate_;
  // additional pubs
  // In constructor, add:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr test_gray_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      horizontal_filter_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      vertical_filter_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      detected_grid_publisher_;

  bool grid_lines_done_;

  // GAME START INFOS
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      game_start_subscriber_;
  int robot_shape_;
  int nb_right_since_last_piper_order_;

  // yeye
  std::vector<std::vector<int>> current_game_board_2D_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraSubscriber>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}