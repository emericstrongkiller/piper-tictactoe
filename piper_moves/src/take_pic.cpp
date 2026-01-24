#include "rclcpp/logging.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/detail/int32_multi_array__struct.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
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

class CameraSubscriber : public rclcpp::Node {
public:
  CameraSubscriber() : Node("camera_subscriber") {
    // camera topic parameter
    this->declare_parameter<std::string>("camera_topic", "/camera1/image_raw");
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 10,
        std::bind(&CameraSubscriber::listener_callback, this,
                  std::placeholders::_1));

    order_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/take_pic_order", 10,
        std::bind(&CameraSubscriber::order_subscriber_callback, this,
                  std::placeholders::_1));

    order_response_publisher_ =
        this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/take_pic_order_response", 10);
  }

private:
  void order_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "Receiving take Pic order. About to perceive game Grid..");
    order_done_ = !(msg->data);
  }

  void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Analyse + output grid state
    if (!order_done_) {
      RCLCPP_INFO(this->get_logger(), "Receiving image");

      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      current_game_board_ = percieve_game_board(cv_ptr->image);
      save_image(current_game_board_.first, "captured_image.jpg");
      current_game_board_squares_.data = current_game_board_.second;
      order_response_publisher_->publish(current_game_board_squares_);
      order_done_ = true;
    }
  }

  std::pair<cv::Mat, std::vector<int>> percieve_game_board(cv::Mat image) {
    cv::Mat processed_image;
    std::vector<int> game_board_grid_state(9, 0); // start grid with no shape

    // Processing image
    cv::cvtColor(image, processed_image, cv::COLOR_BGR2GRAY);
    cv::medianBlur(processed_image, processed_image, 11);
    cv::Canny(processed_image, processed_image, 100, 300, 3);

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
          int shape_id = determine_shape(
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
    return std::pair<cv::Mat, std::vector<int>>(processed_image,
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

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr order_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
      order_response_publisher_;

  // Image variables
  std::pair<cv::Mat, std::vector<int>> current_game_board_;
  std::vector<cv::Point> grid_centers_;
  std::pair<std::array<float, 2>, cv::Point> center_square_infos_;
  std_msgs::msg::Int32MultiArray current_game_board_squares_;
  bool order_done_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}