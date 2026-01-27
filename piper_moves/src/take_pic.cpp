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
    perception_params_.data = {50, 50};
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
    RCLCPP_DEBUG(this->get_logger(), "perception_params_ are now: %d | %d",
                 static_cast<int>(perception_params_.data[0]),
                 static_cast<int>(perception_params_.data[1]));
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
    // perception_params_.data[0] for threshold dddude
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
    cv_ptr_->image = binary_filtered;
    cv_ptr_->encoding = "mono8";
    binary_image_server_publisher_->publish(*cv_ptr_->toImageMsg());

    // Crop the image using ROI
    cv::Rect roi(x_start + 10, y_start + 10, width - 20, height - 20);
    cv::Mat shadow_removed_cropped = shadow_removed(roi);
    cv::Mat img_cropped = image(roi);

    // adaptive filter on roi
    // cv::threshold(shadow_removed_cropped, shadow_removed_cropped, 11, 255,
    // 0);
    cv::adaptiveThreshold(shadow_removed_cropped, shadow_removed_cropped, 255,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11,
                          -3);

    // Detect vertical lines first
    std::vector<cv::Vec4i> v_lines;
    cv::HoughLinesP(shadow_removed_cropped, v_lines, 1, CV_PI / 180, 50, 50,
                    10);
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
    cv::HoughLinesP(shadow_removed_cropped, h_lines, 1, (CV_PI / 180), 85, 1,
                    10);
    // filter horizontal
    std::vector<cv::Vec4i> h_lines_full;
    for (auto &l : h_lines) {
      float angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
      if (abs(angle) < 2 || abs(angle) > 178) {
        h_lines_full.push_back(l);
        //        cv::line(img_cropped, cv::Point(l[0], l[1]), cv::Point(l[2],
        //        l[3]),
        //                 cv::Scalar(0, 0, 255), 1, 8);
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
      if (abs(pos - clustered.back()) > 4) {
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
      //      cv::line(img_cropped, cv::Point(clustered[0], y_vert_min),
      //               cv::Point(clustered[0], y_vert_max), cv::Scalar(0, 0,
      //               255), 1, 8);
      //      cv::line(img_cropped, cv::Point(clustered[1], y_vert_min),
      //               cv::Point(clustered[1], y_vert_max), cv::Scalar(0, 0,
      //               255), 1, 8);
      //      cv::line(img_cropped, cv::Point(clustered[2], y_vert_min),
      //               cv::Point(clustered[2], y_vert_max), cv::Scalar(0, 0,
      //               255), 1, 8);
      //      cv::line(img_cropped, cv::Point(clustered[3], y_vert_min),
      //               cv::Point(clustered[3], y_vert_max), cv::Scalar(0, 0,
      //               255), 1, 8);
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
      if (abs(pos - h_clustered.back()) > 3) {
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
    std::vector<cv::Point> grid_center_vertices;
    grid_center_vertices.push_back(
        find_lines_intersection(horiz_middle_lines[0], horiz_middle_lines[1],
                                vert_middle_lines[0], vert_middle_lines[1]));
    grid_center_vertices.push_back(
        find_lines_intersection(horiz_middle_lines[0], horiz_middle_lines[1],
                                vert_middle_lines[2], vert_middle_lines[3]));
    grid_center_vertices.push_back(
        find_lines_intersection(horiz_middle_lines[2], horiz_middle_lines[3],
                                vert_middle_lines[0], vert_middle_lines[1]));
    grid_center_vertices.push_back(
        find_lines_intersection(horiz_middle_lines[2], horiz_middle_lines[3],
                                vert_middle_lines[2], vert_middle_lines[3]));
    cv::circle(img_cropped, grid_center_vertices[0], 3, cv::Scalar(0, 0, 255));
    cv::circle(img_cropped, grid_center_vertices[1], 3, cv::Scalar(0, 0, 255));
    cv::circle(img_cropped, grid_center_vertices[2], 3, cv::Scalar(0, 0, 255));
    cv::circle(img_cropped, grid_center_vertices[3], 3, cv::Scalar(0, 0, 255));

    // calculate grid center
    cv::Point grid_center;
    grid_center.x = (grid_center_vertices[0].x + grid_center_vertices[3].x) / 2;
    grid_center.y = (grid_center_vertices[0].y + grid_center_vertices[3].y) / 2;
    int center_square_width =
        grid_center_vertices[3].x - grid_center_vertices[0].x;
    int center_square_height =
        grid_center_vertices[3].y - grid_center_vertices[0].y;
    // calculate all 9 grid square centers
    std::vector<cv::Point> square_centers;
    square_centers.push_back(cv::Point(grid_center.x - center_square_width,
                                       grid_center.y + center_square_height));
    square_centers.push_back(
        cv::Point(grid_center.x, grid_center.y + center_square_height));
    square_centers.push_back(cv::Point(grid_center.x + center_square_width,
                                       grid_center.y + center_square_height));
    square_centers.push_back(
        cv::Point(grid_center.x - center_square_width, grid_center.y));
    square_centers.push_back(cv::Point(grid_center.x, grid_center.y));
    square_centers.push_back(
        cv::Point(grid_center.x + center_square_width, grid_center.y));
    square_centers.push_back(cv::Point(grid_center.x - center_square_width,
                                       grid_center.y - center_square_height));
    square_centers.push_back(
        cv::Point(grid_center.x, grid_center.y - center_square_height));
    square_centers.push_back(cv::Point(grid_center.x + center_square_width,
                                       grid_center.y - center_square_height));
    // draw grid centers
    for (auto &el : square_centers) {
      cv::circle(img_cropped, el, 3, cv::Scalar(0, 255, 0));
    }
    // draw rectangle areas around centers
    for (size_t i = 0; i < square_centers.size(); i++) {
      cv::rectangle(img_cropped,
                    cv::Point(square_centers[static_cast<int>(i)].x -
                                  (center_square_width / 2),
                              square_centers[static_cast<int>(i)].y -
                                  (center_square_height / 2)),
                    cv::Point(square_centers[static_cast<int>(i)].x +
                                  (center_square_width / 2),
                              square_centers[static_cast<int>(i)].y +
                                  (center_square_height / 2)),
                    cv::Scalar(255, 0, 0), 1);
    }

    cv::Mat cropped_gray;
    cv::cvtColor(img_cropped, cropped_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> detected_shapes = detect_shapes(cropped_gray, 10);
    print_detected_shapes(detected_shapes, img_cropped);

    RCLCPP_INFO(this->get_logger(), "detected_shapes_size: %zu",
                detected_shapes.size());
    for (size_t i = 0; i < detected_shapes.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "detected_shapes[%zu]: YAH", i);
    }

    cv_ptr_->image = img_cropped;
    cv_ptr_->encoding = "bgr8";
    roi_image_server_publisher_->publish(*cv_ptr_->toImageMsg());

    // gridLines test

    // cv_ptr_->image = processed_image;
    // cv_ptr_->encoding = "mono8";
    // cvtColor_server_publisher_->publish(*cv_ptr_->toImageMsg());

    // cv::medianBlur(processed_image, processed_image, 11);
    // cv_ptr_->image = image;
    // no_shadow_server_publisher_->publish(*cv_ptr_->toImageMsg());
    // cv::Canny(processed_image, processed_image, 100, 300, 3);

    /*

    std::vector<std::vector<float>> grid_lines =
        detect_grid_lines(processed_image);
    std::vector<float> grid_h = grid_lines[0];
    std::vector<float> grid_v = grid_lines[1];

    // check to avoid segmentation fault down the llline
    if (grid_h.empty() || grid_v.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Grid detection failed");
      return {cv::Mat(), std::vector<int>(9, 0)};
    }

    // detect and draw grid square centers
    center_square_infos_ = calculate_center_square_infos(grid_v, grid_h);
    grid_centers_ = calculate_all_grid_centers(center_square_infos_);



    // shape detection
    cv::Mat mser_image;
    cv::cvtColor(image, mser_image, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> detected_shapes = detect_shapes(mser_image, 60);

    // Shape identification using template matching
    cv::Mat match_image;
    cv::cvtColor(image, match_image, cv::COLOR_BGR2GRAY);
    auto pkg_share =
        ament_index_cpp::get_package_share_directory("piper_moves");
    std::string cross_templ_path =
        pkg_share + "/templates/cross.png"; // cross-tpl
    cv::Mat cross_templ = cv::imread(cross_templ_path);
    cv::Mat cross_processed_templ;
    cv::cvtColor(cross_templ, cross_processed_templ, cv::COLOR_BGR2GRAY);
    std::string circle_templ_path =
        pkg_share + "/templates/circle.png"; // circle-tpl
    cv::Mat circle_templ = cv::imread(circle_templ_path);
    cv::Mat circle_processed_templ;
    cv::cvtColor(circle_templ, circle_processed_templ, cv::COLOR_BGR2GRAY);

    // identify each shape (+ filter out shapes outside of grid squares)
    for (size_t i = 0; i < detected_shapes.size(); i++) {
      for (size_t j = 0; j < grid_centers_.size(); j++) {
        // identify and store shape if close enough to one of the grid centers
        if (detected_shapes[i].pt.x < grid_centers_[j].x + 60 &&
            detected_shapes[i].pt.x > (grid_centers_[j].x - 60) &&
            detected_shapes[i].pt.y < grid_centers_[j].y + 60 &&
            detected_shapes[i].pt.y > (grid_centers_[j].y - 60)) {
          int shape_id = determine_shape(2574 4051 4053 4110 4126 4111 4116
              match_image, cross_processed_templ, circle_processed_templ, image,
              cv::Point(detected_shapes[i].pt.x, detected_shapes[i].pt.y), i);
          game_board_grid_state[j] = shape_id;
          RCLCPP_INFO(this->get_logger(),
                      "Shape N*: %zu is close enough to grid_center: %zu", i,
                      j);
          RCLCPP_INFO(
              this->get_logger(),
              "=> Shape_id: %d was stored in game_board_grid_state[%zu]",
              shape_id, j);
          break;
        }
      }
    }

    // image printings part
     print_grid_lines(grid_h, grid_v, image);
     print_grid_contours(image);
     print_grid_centers(grid_centers_, image);
     print_detected_shapes(detected_shapes, image);

    */
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

    RCLCPP_INFO(this->get_logger(), "Image saved to %s", save_path.c_str());
  }

  std::vector<std::vector<float>> detect_grid_lines(cv::Mat image) {
    // Apply ROI mask (center 70% to reduce edge noise)
    int margin_x = image.cols * 0.15;
    int margin_y = image.rows * 0.15;
    cv::Rect roi(margin_x, margin_y, image.cols - 2 * margin_x,
                 image.rows - 2 * margin_y);
    cv::Mat roi_image = image(roi);

    // Detect lines in ROI
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(roi_image, lines, 1, (CV_PI / 180), 50, 50, 10);

    // Offset coordinates back to full image
    for (auto &l : lines) {
      l[0] += margin_x;
      l[1] += margin_y;
      l[2] += margin_x;
      l[3] += margin_y;
    }

    // Separate horizontal and vertical lines (store full line, not just avg)
    std::vector<cv::Vec4i> h_lines_full, v_lines_full;
    for (auto &l : lines) {
      float angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
      if (abs(angle) < 20 || abs(angle) > 160) {
        h_lines_full.push_back(l);
      } else if (abs(angle - 90) < 20 || abs(angle + 90) < 20) {
        v_lines_full.push_back(l);
      }
    }

    // Find all line intersections
    std::vector<cv::Point2f> intersections;
    for (const auto &h : h_lines_full) {
      for (const auto &v : v_lines_full) {
        float x1 = h[0], y1 = h[1], x2 = h[2], y2 = h[3];
        float x3 = v[0], y3 = v[1], x4 = v[2], y4 = v[3];

        float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (abs(denom) < 1e-6)
          continue;

        float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;

        float px = x1 + t * (x2 - x1);
        float py = y1 + t * (y2 - y1);
        // Only keep intersections within image bounds
        if (px >= 0 && px < image.cols && py >= 0 && py < image.rows) {
          intersections.push_back(cv::Point2f(px, py));
        }
      }
    }

    // Cluster intersections (merge nearby points)
    std::vector<cv::Point2f> clustered_intersections;
    std::vector<bool> used(intersections.size(), false);
    const float cluster_dist = 25.0f;

    for (size_t i = 0; i < intersections.size(); i++) {
      if (used[i])
        continue;
      float sum_x = intersections[i].x, sum_y = intersections[i].y;
      int count = 1;
      used[i] = true;

      for (size_t j = i + 1; j < intersections.size(); j++) {
        if (used[j])
          continue;
        if (cv::norm(intersections[i] - intersections[j]) < cluster_dist) {
          sum_x += intersections[j].x;
          sum_y += intersections[j].y;
          count++;
          used[j] = true;
        }
      }
      clustered_intersections.push_back(
          cv::Point2f(sum_x / count, sum_y / count));
    }

    if (clustered_intersections.size() < 4) {
      RCLCPP_ERROR(this->get_logger(),
                   "Not enough grid intersections detected");
      return {{}, {}};
    }

    // Extract unique horizontal and vertical positions from intersections
    std::vector<float> h_positions, v_positions;
    for (const auto &pt : clustered_intersections) {
      h_positions.push_back(pt.y);
      v_positions.push_back(pt.x);
    }

    // Cluster positions to get unique lines
    auto cluster_1d = [](std::vector<float> &vals, float tol) {
      std::sort(vals.begin(), vals.end());
      std::vector<float> result;
      for (float v : vals) {
        if (result.empty() || abs(v - result.back()) > tol) {
          result.push_back(v);
        }
      }
      return result;
    };

    h_positions = cluster_1d(h_positions, 30.0f);
    v_positions = cluster_1d(v_positions, 30.0f);

    // Score and select best pair (reasonable spacing + centered)
    auto find_best_pair = [](std::vector<float> &positions, float min_spacing,
                             float max_spacing, float img_center) {
      std::vector<float> best_pair;
      float best_score = -1;

      for (size_t i = 0; i < positions.size(); i++) {
        for (size_t j = i + 1; j < positions.size(); j++) {
          float spacing = abs(positions[j] - positions[i]);
          if (spacing >= min_spacing && spacing <= max_spacing) {
            float center = (positions[i] + positions[j]) / 2.0f;
            float img_center_dist = abs(center - img_center);
            float score = spacing / (1 + img_center_dist * 0.01f);
            if (score > best_score) {
              best_score = score;
              best_pair = {positions[i], positions[j]};
            }
          }
        }
      }
      return best_pair;
    };

    std::vector<float> grid_h =
        find_best_pair(h_positions, 80.0f, 300.0f, image.rows / 2.0f);
    std::vector<float> grid_v =
        find_best_pair(v_positions, 80.0f, 300.0f, image.cols / 2.0f);

    return {grid_h, grid_v};
  }

  void print_grid_lines(std::vector<float> grid_h, std::vector<float> grid_v,
                        cv::Mat image) {
    // print clustered lines
    for (auto &el : grid_h) {
      cv::line(image, cv::Point(600, el), cv::Point(1250, el),
               cv::Scalar(0, 0, 255), 3, 8);
      RCLCPP_INFO(this->get_logger(), "Cluster h_line: %f", el);
    }
    for (auto &el : grid_v) {
      cv::line(image, cv::Point(el, 300), cv::Point(el, 800),
               cv::Scalar(0, 0, 255), 3, 8);
      RCLCPP_INFO(this->get_logger(), "Cluster v_line: %f", el);
    }
  }

  std::pair<std::array<float, 2>, cv::Point>
  calculate_center_square_infos(std::vector<float> grid_v,
                                std::vector<float> grid_h) {
    // Calculate squares positions
    // First: center square edges
    cv::Point center_upper_left = cv::Point(grid_v[0], grid_h[0]);
    cv::Point center_upper_right = cv::Point(grid_v[1], grid_h[0]);
    cv::Point center_lower_left = cv::Point(grid_v[0], grid_h[1]);
    // center square's center
    float square_height = center_upper_left.y - center_lower_left.y;
    float square_length = center_upper_right.x - center_upper_left.x;
    cv::Point center = cv::Point(center_upper_right.x - square_length / 2,
                                 center_upper_right.y - square_height / 2);
    std::array<float, 2> dimensions = {square_height, square_length};
    std::pair<std::array<float, 2>, cv::Point> center_grid_infos = {dimensions,
                                                                    center};
    return center_grid_infos;
  }

  void print_grid_contours(cv::Mat image) {
    cv::rectangle(
        image,
        cv::Point(
            center_square_infos_.second.x - center_square_infos_.first[1] * 2,
            center_square_infos_.second.y - center_square_infos_.first[0] * 2),
        cv::Point(
            center_square_infos_.second.x + center_square_infos_.first[1] * 2,
            center_square_infos_.second.y + center_square_infos_.first[0] * 2),
        cv::Scalar(0, 0, 255), 1);
  }

  std::vector<cv::Point> calculate_all_grid_centers(
      std::pair<std::array<float, 2>, cv::Point> center_square_infos) {
    std::vector<cv::Point> grid_centers;

    float center_x = center_square_infos.second.x;
    float center_y = center_square_infos.second.y;
    float square_height = center_square_infos.first[0];
    float square_length = center_square_infos.first[1];

    grid_centers.push_back(
        cv::Point(center_x - square_length, center_y - square_height));
    grid_centers.push_back(cv::Point(center_x, center_y - square_height));
    grid_centers.push_back(
        cv::Point(center_x + square_length, center_y - square_height));
    grid_centers.push_back(cv::Point(center_x - square_length, center_y));
    grid_centers.push_back(cv::Point(center_x, center_y));
    grid_centers.push_back(cv::Point(center_x + square_length, center_y));
    grid_centers.push_back(
        cv::Point(center_x - square_length, center_y + square_height));
    grid_centers.push_back(cv::Point(center_x, center_y + square_height));
    grid_centers.push_back(
        cv::Point(center_x + square_length, center_y + square_height));

    return grid_centers;
  }

  void print_grid_centers(std::vector<cv::Point> grid_centers, cv::Mat image) {
    for (size_t i = 0; i < grid_centers.size(); i++) {
      cv::circle(image, grid_centers[i], 10, cv::Scalar(0, 0, 255));
    }
  }

  std::vector<cv::KeyPoint> detect_shapes(cv::Mat image, float min_size) {
    // MSER detector
    cv::Ptr<cv::MSER> detector = cv::MSER::create();

    std::vector<cv::KeyPoint> fs;
    detector->detect(image, fs);

    std::sort(fs.begin(), fs.end(),
              [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                return a.size > b.size; // descending
              });

    // filter by neighbours
    std::vector<cv::KeyPoint> sfs;
    sfs.reserve(fs.size());
    for (const auto &x : fs) {
      if (!suppressByLargerNearby(x, fs)) {
        sfs.push_back(x);
      }
    }

    // filter by min_size
    std::vector<cv::KeyPoint> sized_sfs;
    sized_sfs.reserve(sfs.size());
    for (const auto &x : sfs) {
      if (x.size > min_size) {
        sized_sfs.push_back(x);
      }
    }

    return sized_sfs;
  }

  static bool suppressByLargerNearby(const cv::KeyPoint &x,
                                     const std::vector<cv::KeyPoint> &fs) {
    for (const auto &f : fs) {
      const float distx = f.pt.x - x.pt.x;
      const float disty = f.pt.y - x.pt.y;
      const float dist = std::sqrt(distx * distx + disty * disty);

      if ((f.size > x.size) && (dist < f.size / 2.0f)) {
        return true;
      }
    }
    return false;
  }

  void print_detected_shapes(std::vector<cv::KeyPoint> detected_shapes,
                             cv::Mat image) {
    const cv::Scalar d_red(65, 55, 150);   // BGR
    const cv::Scalar l_red(200, 200, 250); // BGR

    for (const auto &circle : detected_shapes) {
      const cv::Point center(cvRound(circle.pt.x), cvRound(circle.pt.y));
      const int radius = cvRound(circle.size / 2.0f);

      cv::circle(image, center, radius / 5, d_red, 2, cv::LINE_AA);
      cv::circle(image, center, radius / 5, l_red, 1, cv::LINE_AA);
    }
  }

  int determine_shape(cv::Mat processed_image, cv::Mat cross_processed_templ,
                      cv::Mat circle_processed_templ, cv::Mat image,
                      cv::Point point_of_interest, int image_nb) {
    // return unknown shape if no shape is recognized with template matching
    int shape_type = -1;

    // create a smaller matrix in the area the caller wants to
    // check for cross/circle
    cv::Point c = point_of_interest;
    float square_height = std::abs(center_square_infos_.first[0]);
    float square_length = std::abs(center_square_infos_.first[1]);
    float res_x0 = c.x - square_length / 1.7;
    float res_y0 = c.y - square_height / 1.7;
    cv::Rect roi(res_x0, res_y0, square_length * 1.1, square_height * 1.1);
    cv::Mat area_of_interest = processed_image(roi);

    cv::Mat res;

    // test each shape on the area of interest:
    // cross first
    cv::matchTemplate(area_of_interest, cross_processed_templ, res,
                      cv::TM_CCOEFF_NORMED);

    std::string file_name = "shape_" + std::to_string(image_nb) + ".jpg";
    save_image(area_of_interest, file_name);

    if (res.size.dims() > 0) {
      // Threshold
      const double threshold = 0.35;

      // Find all locations with score >= threshold
      for (int y = 0; y < res.rows; ++y) {
        for (int x = 0; x < res.cols; ++x) {
          float score = res.at<float>(y, x);
          if (score >= threshold) {
            cv::putText(image, "cross",
                        cv::Point(res_x0 + x + square_length / 2,
                                  res_y0 + y + square_height / 2),
                        cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 0),
                        1);
            shape_type = 1;
          }
        }
      }
    }
    // then circle
    cv::matchTemplate(area_of_interest, circle_processed_templ, res,
                      cv::TM_CCOEFF_NORMED);

    if (res.size.dims() > 0) {
      // Threshold
      const double threshold = 0.34;

      // Find all locations with score >= threshold
      for (int y = 0; y < res.rows; ++y) {
        for (int x = 0; x < res.cols; ++x) {
          float score = res.at<float>(y, x);
          if (score >= threshold) {
            cv::circle(image,
                       cv::Point(res_x0 + x + square_length / 2,
                                 res_y0 + y + square_height / 2),
                       40, cv::Scalar(0, 0, 255), 2);
            shape_type = 2;
          }
        }
      }
    }

    return shape_type;
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