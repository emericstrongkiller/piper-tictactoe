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
const int VERTICAL_LINES_X_CLUSTERS_INTERVAL = 1;
const int HORIZONTAL_LINES_Y_CLUSTERS_INTERVAL = 2;

class CameraSubscriber : public rclcpp::Node {
public:
  CameraSubscriber() : Node("camera_subscriber") {
    // camera topic parameter
    this->declare_parameter<std::string>("camera_topic",
                                         "/camera/D435/color/image_raw");
    std::string camera_topic = this->get_parameter("camera_topic").as_string();

    // Reentrant callback group
    auto callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
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

    // variables init
    perception_params_.data = {0, 3, 0};
  }

private:
  void order_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "Receiving take Pic order. About to perceive game Grid..");
    order_done_ = !(msg->data);
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
      RCLCPP_INFO(this->get_logger(), "Receiving image");

      try {
        cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
      } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      cv_ptr_->encoding = "bgr8";

      current_game_board_ = percieve_game_board(cv_ptr_->image);
      // save_image(current_game_board_.first, "captured_image.jpg");
      // current_game_board_squares_.data = current_game_board_.second;
      // order_response_publisher_->publish(current_game_board_squares_);
      cv_ptr_->image = current_game_board_.first;
      cv_ptr_->encoding = "bgr8";
      video_server_publisher_->publish(*cv_ptr_->toImageMsg());
      // order_done_ = true;
    }
  }

  std::pair<cv::Mat, std::vector<int>> percieve_game_board(cv::Mat image) {
    cv::Mat processed_image;
    std::vector<int> game_board_grid_state(9, 0); // start grid with no shape

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
    double tablet_contour_index;
    for (size_t i = 0; i < contours.size(); i++) {
      if (cv::contourArea(contours[i]) > TABLET_CONTOUR_AREA_INTERVAL[0] &&
          cv::contourArea(contours[i]) < TABLET_CONTOUR_AREA_INTERVAL[1]) {
        tablet_contour_index = i;
      }
      RCLCPP_DEBUG(this->get_logger(), "contour[%zu].area: %f", i,
                   cv::contourArea(contours[i]));
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

    cv::Mat processed_imaged;
    cv::cvtColor(img_cropped, processed_imaged, cv::COLOR_BGR2GRAY);

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
      //          cv::Point(clustered[0], y_vert_max), cv::Scalar(0, 0, 255), 1,
      //          8);
      // cv::line(img_cropped, cv::Point(clustered[1], y_vert_min),
      //          cv::Point(clustered[1], y_vert_max), cv::Scalar(0, 0, 255), 1,
      //          8);
      // cv::line(img_cropped, cv::Point(clustered[2], y_vert_min),
      //          cv::Point(clustered[2], y_vert_max), cv::Scalar(0, 0, 255), 1,
      //          8);
      // cv::line(img_cropped, cv::Point(clustered[3], y_vert_min),
      //          cv::Point(clustered[3], y_vert_max), cv::Scalar(0, 0, 255), 1,
      //          8);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Not enough clustered lines: %zu",
                   clustered.size());
    }

    // Calculate and Draw Horizontal lines
    std::vector<int> h_line_x_positions;
    std::vector<int> h_line_y_positions;
    int x_horiz_min, x_horiz_max;
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
      // cv::line(img_cropped, cv::Point(x_horiz_min, h_clustered[0]),
      //          cv::Point(x_horiz_max, h_clustered[0]), cv::Scalar(0, 0, 255),
      //          1, 8);
      // cv::line(img_cropped, cv::Point(x_horiz_min, h_clustered[1]),
      //          cv::Point(x_horiz_max, h_clustered[1]), cv::Scalar(0, 0, 255),
      //          1, 8);
      // cv::line(img_cropped, cv::Point(x_horiz_min, h_clustered[2]),
      //          cv::Point(x_horiz_max, h_clustered[2]), cv::Scalar(0, 0, 255),
      //          1, 8);
      // cv::line(img_cropped, cv::Point(x_horiz_min, h_clustered[3]),
      //          cv::Point(x_horiz_max, h_clustered[3]), cv::Scalar(0, 0, 255),
      //          1, 8);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Not enough h_clustered lines: %zu",
                   h_clustered.size());
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
    // cv::line(img_cropped, horiz_middle_lines[0], horiz_middle_lines[1],
    //         cv::Scalar(0, 0, 255), 1, 8);
    // cv::line(img_cropped, horiz_middle_lines[2], horiz_middle_lines[3],
    //         cv::Scalar(0, 0, 255), 1, 8);
    // cv::line(img_cropped, vert_middle_lines[0], vert_middle_lines[1],
    //         cv::Scalar(0, 0, 255), 1, 8);
    // cv::line(img_cropped, vert_middle_lines[2], vert_middle_lines[3],
    //         cv::Scalar(0, 0, 255), 1, 8);
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
    // cv::circle(img_cropped, grid_center_vertices_[0], 3, cv::Scalar(0, 0,
    // 255)); cv::circle(img_cropped, grid_center_vertices_[1], 3, cv::Scalar(0,
    // 0, 255)); cv::circle(img_cropped, grid_center_vertices_[2], 3,
    // cv::Scalar(0, 0, 255)); cv::circle(img_cropped, grid_center_vertices_[3],
    // 3, cv::Scalar(0, 0, 255));

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

    // SHAPE DETECTION  ################################################

    // square 1 contour process
    // generate square ROI
    cv::Rect square_roi = generate_square_roi(perception_params_.data[0], 5);

    cv::Mat square = edges(square_roi);
    cv::Mat color_square = img_cropped(square_roi);

    // 1. Close gaps → merge fragments into ONE contour
    cv::Mat kernel_cropped = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(perception_params_.data[1], perception_params_.data[1]));
    cv::morphologyEx(square, square, cv::MORPH_CLOSE, kernel_cropped);

    // 2. Now find cropped_contours
    std::vector<std::vector<cv::Point>> cropped_contours;
    cv::findContours(square, cropped_contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    // and Draw BIGGEST contour on the chosen square
    double biggest_area = 0;
    double biggest_area_index = 0;
    for (size_t i = 0; i < cropped_contours.size(); i++) {
      if (contourArea(cropped_contours[i]) > biggest_area) {
        biggest_area = contourArea(cropped_contours[i]);
        biggest_area_index = i;
      }
    }
    cv::drawContours(color_square, cropped_contours, biggest_area_index,
                     cv::Scalar(0, 255, 0), 1);
    RCLCPP_DEBUG(this->get_logger(), "contour area: %f", biggest_area);

    cv_ptr_->image = color_square;
    cv_ptr_->encoding = "bgr8";
    roi_image_server_publisher_->publish(*cv_ptr_->toImageMsg());

    save_image(img_cropped, "teeeeest.png");

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

  cv::Point find_lines_intersection(cv::Point x1_y1, cv::Point x2_y2,
                                    cv::Point x3_y3, cv::Point x4_y4) {
    cv::Point intersection;
    float x1 = x1_y1.x, y1 = x1_y1.y, x2 = x2_y2.x, y2 = x2_y2.y, x3 = x3_y3.x,
          y3 = x3_y3.y, x4 = x4_y4.x, y4 = x4_y4.y;

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) /
              ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    float u = -(((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) /
                ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)));

    if (0 <= t && t <= 1 && 0 <= u && u <= 1) {
      intersection.x = x1 + t * (x2 - x1);
      intersection.y = y1 + t * (y2 - y1);
      return intersection;
    }

    RCLCPP_ERROR(this->get_logger(), "No intersection found");
    return intersection;
  }

  cv::Rect generate_square_roi(int square_index,
                               int offset_from_center_vertice) {
    cv::Rect square_roi;
    switch (square_index) {
    case 0:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[2].x - offset_from_center_vertice,
                    grid_center_vertices_[2].y + offset_from_center_vertice),
          cv::Point(grid_center_vertices_[2].x - offset_from_center_vertice -
                        center_square_width_,
                    grid_center_vertices_[2].y + offset_from_center_vertice +
                        center_square_height_));
      break;

    case 1:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[2].x + offset_from_center_vertice,
                    grid_center_vertices_[2].y + offset_from_center_vertice),
          cv::Point(grid_center_vertices_[3].x - offset_from_center_vertice,
                    grid_center_vertices_[3].y + offset_from_center_vertice +
                        center_square_height_));
      break;

    case 2:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[3].x + offset_from_center_vertice,
                    grid_center_vertices_[3].y + offset_from_center_vertice),
          cv::Point(grid_center_vertices_[3].x + offset_from_center_vertice +
                        center_square_width_,
                    grid_center_vertices_[3].y + offset_from_center_vertice +
                        center_square_height_));
      break;

    case 3:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[0].x - offset_from_center_vertice,
                    grid_center_vertices_[0].y + offset_from_center_vertice),
          cv::Point(grid_center_vertices_[2].x - offset_from_center_vertice -
                        center_square_width_,
                    grid_center_vertices_[2].y - offset_from_center_vertice));
      break;

    case 4:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[0].x + offset_from_center_vertice,
                    grid_center_vertices_[0].y + offset_from_center_vertice),
          cv::Point(grid_center_vertices_[3].x - offset_from_center_vertice,
                    grid_center_vertices_[3].y - offset_from_center_vertice));
      break;

    case 5:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[1].x + offset_from_center_vertice,
                    grid_center_vertices_[1].y + offset_from_center_vertice),
          cv::Point(grid_center_vertices_[3].x + offset_from_center_vertice +
                        center_square_width_,
                    grid_center_vertices_[3].y - offset_from_center_vertice));
      break;

    case 6:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[0].x - offset_from_center_vertice,
                    grid_center_vertices_[0].y - offset_from_center_vertice),
          cv::Point(grid_center_vertices_[0].x - offset_from_center_vertice -
                        center_square_width_,
                    grid_center_vertices_[0].y - offset_from_center_vertice -
                        center_square_height_));
      break;

    case 7:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[0].x + offset_from_center_vertice,
                    grid_center_vertices_[0].y - offset_from_center_vertice),
          cv::Point(grid_center_vertices_[1].x - offset_from_center_vertice,
                    grid_center_vertices_[1].y - offset_from_center_vertice -
                        center_square_height_));
      break;

    case 8:
      square_roi = cv::Rect(
          cv::Point(grid_center_vertices_[1].x + offset_from_center_vertice,
                    grid_center_vertices_[1].y - offset_from_center_vertice),
          cv::Point(grid_center_vertices_[1].x + offset_from_center_vertice +
                        center_square_width_,
                    grid_center_vertices_[1].y - offset_from_center_vertice -
                        center_square_height_));
      break;
    }
    if (square_roi.height < 0 || square_roi.width < 0 || square_roi.x < 0 ||
        square_roi.y < 0) {
      square_roi = cv::Rect(10, 10, 10, 10);
    }
    return square_roi;
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