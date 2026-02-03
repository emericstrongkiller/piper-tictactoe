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

const double TABLET_CONTOUR_AREA_INTERVAL[2] = {470000, 480000};
const int VERTICAL_LINES_X_CLUSTERS_INTERVAL = 10;   // 1
const int HORIZONTAL_LINES_Y_CLUSTERS_INTERVAL = 10; // 2

class CameraSubscriber : public rclcpp::Node {
public:
  CameraSubscriber() : Node("camera_subscriber") {
    // variables init
    perception_params_.data = {0, 3, 0};
    squares_buffers_.resize(9);
    pipeline_occ_counter_ = 0;
    pipeline_success_rate_ = 0.0;
    pipeline_failures_ = 0;
    order_done_ = true;
    nb_right_since_last_piper_order_ = 0;

    // game vars
    robot_shape_ = 0;

    // camera topic parameter
    this->declare_parameter<std::string>("camera_topic", "/camera1/image_raw");
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
    video_server_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/perception_server", 10);
    cvtColor_server_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/cvtColor_perception_server", 10);
    no_shadow_server_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/no_shadow_perception_server", 10);
    binary_image_server_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/binary_image_perception_server", 10);
    roi_image_server_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/roi_image_perception_server", 10);

    // In constructor, add:
    test_gray_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/test_gray_standard", 10);
    test_saturation_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/test_saturation", 10);
    test_color_filter_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/test_color_filter",
                                                        10);
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

        // if robot plays O this game, publish only if nb_cross>nb_circle
        if (robot_shape_ == 2 && nb_cross > nb_circle &&
            nb_right_since_last_piper_order_ >= 10) {
          RCLCPP_INFO(this->get_logger(), "publishing game_grid status...");
          current_game_board_squares_.data = current_game_board_.second;
          order_response_publisher_->publish(current_game_board_squares_);
          order_done_ = true;
        }
        // if robot plays X this game publish only if nb_cross==nb_circle
        else if (robot_shape_ == 1 && nb_cross == nb_circle &&
                 nb_right_since_last_piper_order_ >= 10) {
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

    // adaptive filter on roi
    // cv::threshold(shadow_removed_cropped, shadow_removed_cropped, 11, 255,
    // 0);
    cv::adaptiveThreshold(shadow_removed_cropped, shadow_removed_cropped, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11,
                          -3);

    // FOR Grid LINES Adaptive Canny TESTTT ###############################

    RCLCPP_DEBUG(this->get_logger(), "GOT TO LINE DETEC");

    cv::Mat processed_imaged;
    // Standard grayscale
    cv::Mat gray_standard;
    cv::cvtColor(img_cropped, gray_standard, cv::COLOR_BGR2GRAY);

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

    cv_ptr_->image = hsv_channels[1]; // S channel
    cv_ptr_->encoding = "mono8";
    test_saturation_publisher_->publish(*cv_ptr_->toImageMsg());

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

    // GRID LINES DETECTION ######################################

    // Detect vertical lines first
    std::vector<cv::Vec4i> v_lines;
    cv::HoughLinesP(edges, v_lines, 1, CV_PI / 180, 90, 60, 60);
    // filter vertical
    std::vector<cv::Vec4i> v_lines_full;
    for (auto &l : v_lines) {
      float angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
      if (abs(angle - 90) < 20 || abs(angle + 90) < 20) {
        v_lines_full.push_back(l);
        // cv::line(img_cropped, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
        //          cv::Scalar(0, 255, 0), 1, 8);
      }
    }

    // Detect horizontal lines after
    std::vector<cv::Vec4i> h_lines;
    cv::HoughLinesP(edges, h_lines, 1, (CV_PI / 180), 60, 30, 20);
    // filter horizontal
    std::vector<cv::Vec4i> h_lines_full;
    for (auto &l : h_lines) {
      float angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
      if (abs(angle) < 10 || abs(angle) > 170) {
        h_lines_full.push_back(l);
        // cv::line(img_cropped, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
        //          cv::Scalar(0, 0, 255), 1, 8);
      }
    }

    // Calculate and Draw Vertical lines
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
      RCLCPP_DEBUG(this->get_logger(), "pushed back: %d -> v_line_x_positions",
                   static_cast<int>((l[0] + l[2]) / 2.0f));
    }

    std::sort(v_line_x_positions.begin(), v_line_x_positions.end());
    // Cluster by proximity (merge lines within 4px)
    std::vector<int> clustered;
    clustered.push_back(v_line_x_positions[0]);
    RCLCPP_DEBUG(this->get_logger(), "CLUSTERED[0]: %d", clustered[0]);
    for (auto &pos : v_line_x_positions) {
      RCLCPP_DEBUG(this->get_logger(), "DIDN'T CLUSTER: %d", pos);
      if (abs(pos - clustered.back()) > VERTICAL_LINES_X_CLUSTERS_INTERVAL) {
        clustered.push_back(pos);
        RCLCPP_DEBUG(this->get_logger(), "CLUSTERED: %d", pos);
      }
    }
    // get max and min y
    for (auto &l : v_lines_full) {
      v_line_y_positions.push_back(l[1]);
      v_line_y_positions.push_back(l[3]);
    }
    y_vert_min =
        *std::min_element(v_line_y_positions.begin(), v_line_y_positions.end());
    y_vert_max =
        *std::max_element(v_line_y_positions.begin(), v_line_y_positions.end());
    RCLCPP_DEBUG(this->get_logger(), "y_vert_min: %d", y_vert_min);
    RCLCPP_DEBUG(this->get_logger(), "y_vert_max: %d", y_vert_max);
    // draw vertical lines
    if (clustered.size() == 4) {
      // cv::line(img_cropped, cv::Point(clustered[0], y_vert_min),
      //         cv::Point(clustered[0], y_vert_max), cv::Scalar(0, 0, 255), 1,
      //         8);
      // cv::line(img_cropped, cv::Point(clustered[1], y_vert_min),
      //          cv::Point(clustered[1], y_vert_max), cv::Scalar(0, 0, 255), 1,
      //           8);
      //  cv::line(img_cropped, cv::Point(clustered[2], y_vert_min),
      //           cv::Point(clustered[2], y_vert_max), cv::Scalar(0, 0, 255),
      //           1,
      //          8);
      // cv::line(img_cropped, cv::Point(clustered[3], y_vert_min),
      //          cv::Point(clustered[3], y_vert_max), cv::Scalar(0, 0, 255), 1,
      //         8);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Wrong vertical clustered lines Count: %zu. Returning..",
                   clustered.size());
      return std::pair<cv::Mat, std::vector<int>>(img_cropped,
                                                  game_board_grid_state);
    }

    // Calculate and Draw Horizontal lines
    std::vector<int> h_line_x_positions;
    std::vector<int> h_line_y_positions;
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
      RCLCPP_DEBUG(this->get_logger(), "pushed back: %d -> h_line_y_positions",
                   static_cast<int>((l[1] + l[3]) / 2.0f));
    }
    std::sort(h_line_y_positions.begin(), h_line_y_positions.end());
    // Cluster by proximity (merge lines within 4px)
    std::vector<int> h_clustered;
    h_clustered.push_back(h_line_y_positions[0]);
    RCLCPP_DEBUG(this->get_logger(), "h_clustered[0]: %d", h_clustered[0]);
    for (auto &pos : h_line_y_positions) {
      RCLCPP_DEBUG(this->get_logger(), "DIDN'T CLUSTER: %d", pos);
      if (abs(pos - h_clustered.back()) >
          HORIZONTAL_LINES_Y_CLUSTERS_INTERVAL) {
        h_clustered.push_back(pos);
        RCLCPP_DEBUG(this->get_logger(), "h_clustered: %d", pos);
      }
    }
    // get max and min x
    for (auto &l : h_lines_full) {
      h_line_x_positions.push_back(l[0]);
      h_line_x_positions.push_back(l[2]);
    }
    x_horiz_min =
        *std::min_element(h_line_x_positions.begin(), h_line_x_positions.end());
    x_horiz_max =
        *std::max_element(h_line_x_positions.begin(), h_line_x_positions.end());
    RCLCPP_DEBUG(this->get_logger(), "x_horiz_min: %d", x_horiz_min);
    RCLCPP_DEBUG(this->get_logger(), "x_horiz_max: %d", x_horiz_max);
    RCLCPP_DEBUG(this->get_logger(), "----------------------");
    // draw horizontal lines
    if (h_clustered.size() == 4) {
      //  cv::line(img_cropped, cv::Point(x_horiz_min, h_clustered[0]),
      //           cv::Point(x_horiz_max, h_clustered[0]), cv::Scalar(0, 0,
      //           255), 1,
      //         8);
      // cv::line(img_cropped, cv::Point(x_horiz_min, h_clustered[1]),
      //          cv::Point(x_horiz_max, h_clustered[1]), cv::Scalar(0, 0, 255),
      //          1, 8);
      // cv::line(img_cropped, cv::Point(x_horiz_min, h_clustered[2]),
      //          cv::Point(x_horiz_max, h_clustered[2]), cv::Scalar(0, 0, 255),
      //          1, 8);
      // cv::line(img_cropped, cv::Point(x_horiz_min, h_clustered[3]),
      //         cv::Point(x_horiz_max, h_clustered[3]), cv::Scalar(0, 0, 255),
      //         1,
      //        8);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Wrong horizontal clustered lines Count: %zu. Returning..",
                   h_clustered.size());
      return std::pair<cv::Mat, std::vector<int>>(img_cropped,
                                                  game_board_grid_state);
    }

    // Calculate middle lines
    std::vector<cv::Point> vert_middle_lines;
    std::vector<cv::Point> horiz_middle_lines;
    horiz_middle_lines.push_back(
        cv::Point(x_horiz_min, (h_clustered[0] + h_clustered[1]) / 2));
    horiz_middle_lines.push_back(
        cv::Point(x_horiz_max, (h_clustered[0] + h_clustered[1]) / 2));
    horiz_middle_lines.push_back(
        cv::Point(x_horiz_min, (h_clustered[2] + h_clustered[3]) / 2));
    horiz_middle_lines.push_back(
        cv::Point(x_horiz_max, (h_clustered[2] + h_clustered[3]) / 2));
    vert_middle_lines.push_back(
        cv::Point((clustered[0] + clustered[1]) / 2, y_vert_min));
    vert_middle_lines.push_back(
        cv::Point((clustered[0] + clustered[1]) / 2, y_vert_max));
    vert_middle_lines.push_back(
        cv::Point((clustered[2] + clustered[3]) / 2, y_vert_min));
    vert_middle_lines.push_back(
        cv::Point((clustered[2] + clustered[3]) / 2, y_vert_max));
    // draw middle lines
    cv::line(img_cropped, horiz_middle_lines[0], horiz_middle_lines[1],
             cv::Scalar(0, 0, 255), 1, 8);
    cv::line(img_cropped, horiz_middle_lines[2], horiz_middle_lines[3],
             cv::Scalar(0, 0, 255), 1, 8);
    cv::line(img_cropped, vert_middle_lines[0], vert_middle_lines[1],
             cv::Scalar(0, 0, 255), 1, 8);
    cv::line(img_cropped, vert_middle_lines[2], vert_middle_lines[3],
             cv::Scalar(0, 0, 255), 1, 8);
    // Find and Draw Intersections
    // top->right|bot_left->right
    grid_center_vertices_.push_back(
        find_lines_intersection(horiz_middle_lines[0], horiz_middle_lines[1],
                                vert_middle_lines[0], vert_middle_lines[1]));
    grid_center_vertices_.push_back(
        find_lines_intersection(horiz_middle_lines[0], horiz_middle_lines[1],
                                vert_middle_lines[2], vert_middle_lines[3]));
    grid_center_vertices_.push_back(
        find_lines_intersection(horiz_middle_lines[2], horiz_middle_lines[3],
                                vert_middle_lines[0], vert_middle_lines[1]));
    grid_center_vertices_.push_back(
        find_lines_intersection(horiz_middle_lines[2], horiz_middle_lines[3],
                                vert_middle_lines[2], vert_middle_lines[3]));
    cv::circle(img_cropped, grid_center_vertices_[0], 3, cv::Scalar(0, 0, 255));
    cv::circle(img_cropped, grid_center_vertices_[1], 3, cv::Scalar(0, 0, 255));
    cv::circle(img_cropped, grid_center_vertices_[2], 3, cv::Scalar(0, 0, 255));
    cv::circle(img_cropped, grid_center_vertices_[3], 3, cv::Scalar(0, 0, 255));

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

    // SHAPE DETECTIONS  ################################################

    for (int i = 0; i < 9; i++) {
      RCLCPP_DEBUG(this->get_logger(), "At step %d in the shape detec loop", i);
      // square 1 contour process
      // generate square ROI
      cv::Rect square_roi = generate_square_roi(i, 5, 5, 10, 1, 6);

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
      cv::Mat color_square = img_cropped(square_roi);

      int shape_id = -1;

      // draw biggest contour if one was detected
      std::vector<cv::Point> biggest_contour = find_biggest_contour(square);
      if (static_cast<int>(biggest_contour.size()) > 0) {
        std::vector<std::vector<cv::Point>> tmp{biggest_contour};
        cv::drawContours(color_square, tmp, 0,
                         cv::Scalar(0, 127, 127 + pow(-1, i) * 127), 1);
        // detect shape
        shape_id = detect_shape(biggest_contour);

        // Buffer analysis based on votes for the current square
        squares_buffers_[i].push_front(shape_id);
        print_buffer(squares_buffers_[i]);

        if (squares_buffers_[i].size() > 10) {
          squares_buffers_[i].pop_back();
        }
        // Vote oftennn, with adaptive thresh
        int min_buffer_size = 5;
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
        RCLCPP_DEBUG(this->get_logger(), "No Shape (no contours)");
        shape_id = 0;
      }
      draw_detected_shape(color_square, shape_id);
      game_board_grid_state[i] = shape_id;
    }

    return std::pair<cv::Mat, std::vector<int>>(img_cropped,
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
    cv::Rect square_roi;
    switch (square_index) {
    case 0:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[2].x - offset_from_center_vertice,
                    grid_center_vertices_[2].y + offset_from_center_vertice),
          cv::Point(grid_center_vertices_[2].x - offset_from_center_vertice -
                        center_square_width_ - corners_additional_area,
                    grid_center_vertices_[2].y + offset_from_center_vertice +
                        center_square_height_ + corners_additional_area));
      break;

    case 1:
      square_roi =
          cv::Rect(cv::Point(grid_center_vertices_[2].x + sides_offset,
                             grid_center_vertices_[2].y + sides_offset),
                   cv::Point(grid_center_vertices_[3].x - sides_offset,
                             grid_center_vertices_[3].y + sides_offset +
                                 center_square_height_ + sides_length_offset));
      break;

    case 2:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[3].x + offset_from_center_vertice,
                    grid_center_vertices_[3].y + offset_from_center_vertice),
          cv::Point(grid_center_vertices_[3].x + offset_from_center_vertice +
                        center_square_width_ + corners_additional_area,
                    grid_center_vertices_[3].y + offset_from_center_vertice +
                        center_square_height_ + corners_additional_area));
      break;

    case 3:
      square_roi =
          cv::Rect(cv::Point(grid_center_vertices_[0].x - sides_offset,
                             grid_center_vertices_[0].y + sides_offset),
                   cv::Point(grid_center_vertices_[2].x - sides_offset -
                                 center_square_width_ - sides_length_offset,
                             grid_center_vertices_[2].y - sides_offset));
      break;

    case 4:
      square_roi =
          cv::Rect(cv::Point(grid_center_vertices_[0].x + center_offset,
                             grid_center_vertices_[0].y + center_offset),
                   cv::Point(grid_center_vertices_[3].x - center_offset,
                             grid_center_vertices_[3].y - center_offset));
      break;

    case 5:
      square_roi =
          cv::Rect(cv::Point(grid_center_vertices_[1].x + sides_offset,
                             grid_center_vertices_[1].y + sides_offset),
                   cv::Point(grid_center_vertices_[3].x + sides_offset +
                                 center_square_width_ + sides_length_offset,
                             grid_center_vertices_[3].y - sides_offset));
      break;

    case 6:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[0].x - offset_from_center_vertice,
                    grid_center_vertices_[0].y - offset_from_center_vertice),
          cv::Point(grid_center_vertices_[0].x - offset_from_center_vertice -
                        center_square_width_ - corners_additional_area,
                    grid_center_vertices_[0].y - offset_from_center_vertice -
                        center_square_height_ - corners_additional_area));
      break;

    case 7:
      square_roi =
          cv::Rect(cv::Point(grid_center_vertices_[0].x + sides_offset,
                             grid_center_vertices_[0].y - sides_offset),
                   cv::Point(grid_center_vertices_[1].x - sides_offset,
                             grid_center_vertices_[1].y - sides_offset -
                                 center_square_height_ - sides_length_offset));
      break;

    case 8:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[1].x + offset_from_center_vertice,
                    grid_center_vertices_[1].y - offset_from_center_vertice),
          cv::Point(grid_center_vertices_[1].x + offset_from_center_vertice +
                        center_square_width_ + corners_additional_area,
                    grid_center_vertices_[1].y - offset_from_center_vertice -
                        center_square_height_ - corners_additional_area));
      break;
    }
    if (square_roi.height < 0 || square_roi.width < 0 || square_roi.x < 0 ||
        square_roi.y < 0) {
      square_roi = cv::Rect(10, 10, 10, 10);
    }
    return square_roi;
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

  int detect_shape(std::vector<cv::Point> contour) {
    double area = cv::contourArea(contour);
    if (area < 1000) {
      RCLCPP_DEBUG(this->get_logger(), "No Shape");
      return 0;
    }

    // Calculate circularity (4π * area / perimeter²)
    double perimeter = cv::arcLength(contour, true);
    double circularity = (4 * CV_PI * area) / (perimeter * perimeter);

    RCLCPP_DEBUG(this->get_logger(), "circularity: %.2f", circularity);

    if (circularity > 0.85) {
      RCLCPP_DEBUG(this->get_logger(), "Circle");
      return 2;
    } else if (circularity < 0.65) {
      RCLCPP_DEBUG(this->get_logger(), "Cross");
      return 1;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Uncertain");
      return -1;
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

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr order_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      order_response_publisher_;
  // perception image servers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_server_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      cvtColor_server_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      no_shadow_server_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      binary_image_server_publisher_;
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
      test_saturation_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      test_color_filter_publisher_;
  // GAME START INFOS
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      game_start_subscriber_;
  int robot_shape_;
  int nb_right_since_last_piper_order_;
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