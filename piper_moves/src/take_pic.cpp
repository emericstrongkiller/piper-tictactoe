#include "rclcpp/logging.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

class CameraSubscriber : public rclcpp::Node {
public:
  CameraSubscriber() : Node("camera_subscriber") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10,
        std::bind(&CameraSubscriber::listener_callback, this,
                  std::placeholders::_1));
  }

private:
  void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Receiving image");

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Now lets do some image processing !
    processed_image_ = set_grid(cv_ptr->image);

    save_image(processed_image_);

    rclcpp::shutdown();
  }

  cv::Mat set_grid(cv::Mat image) {
    // convert to more efficient color space (HSV)
    cv::Mat processed_image;

    cv::cvtColor(image, processed_image, cv::COLOR_BGR2GRAY);
    // circle detection on gray image
    std::vector<cv::KeyPoint> detected_circles =
        detect_circles(processed_image);
    // print_circles(detected_circles, image);

    cv::medianBlur(processed_image, processed_image, 11);
    cv::Canny(processed_image, processed_image, 100, 300, 3);

    // detect and draw grid lines
    std::vector<std::vector<float>> grid_lines =
        detect_grid_lines(processed_image);
    std::vector<float> grid_h = grid_lines[0];
    std::vector<float> grid_v = grid_lines[1];
    // print_grid_lines(grid_h, grid_v, image);

    // detect and draw grid square centers
    center_square_infos_ = calculate_center_square_infos(grid_v, grid_h);
    grid_centers_ = calculate_all_grid_centers(center_square_infos_);
    // print_grid_centers(grid_centers_, image);

    // Cross template matching test
    cv::Mat match_image;
    cv::cvtColor(image, match_image, cv::COLOR_BGR2GRAY);
    auto pkg_share =
        ament_index_cpp::get_package_share_directory("piper_moves");
    std::string templ_path = pkg_share + "/templates/cross.png";
    cv::Mat templ = cv::imread(templ_path);
    cv::Mat processed_templ;
    cv::cvtColor(templ, processed_templ, cv::COLOR_BGR2GRAY);
    find_crosses(match_image, processed_templ, image);

    return image;
  }

  void save_image(cv::Mat image) {
    const std::string dir = "/home/user/ros2_ws/src/helper_scripts/pics";
    const std::string save_path =
        (std::filesystem::path(dir) / "captured_image.jpg").string();
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
    std::vector<std::vector<float>> detected_grid_lines;
    std::vector<cv::Vec4i> lines;

    cv::HoughLinesP(image, lines, 1, (CV_PI / 180), 50, 50, 10);
    // filter lines by length threshold
    for (size_t i = 0; i < lines.size(); i++) {
      if (sqrt(pow(lines[i][0] - lines[i][2], 2) +
               pow(lines[i][1] - lines[i][3], 2)) > 100) {
        lines.erase(lines.begin() + i);
      }
    }

    // separate horizontal and vertical lines
    std::vector<float> h_lines, v_lines;
    for (auto &l : lines) {
      float angle = atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;

      if (abs(angle) < 20 || abs(angle) > 160) {                 // horizontal
        h_lines.push_back((l[1] + l[3]) / 2.0f);                 // avg y
      } else if (abs(angle - 90) < 20 || abs(angle + 90) < 20) { // vertical
        v_lines.push_back((l[0] + l[2]) / 2.0f);                 // avg x
      }
    }
    // sorting
    std::sort(h_lines.begin(), h_lines.end());
    std::sort(v_lines.begin(), v_lines.end());

    // clustering with vote counting
    // horizontal lines
    std::vector<std::pair<float, int>> h_clusters; // (avg_position, vote_count)
    int cluster_start = 0;
    for (int i = 1; i <= static_cast<int>(h_lines.size()); i++) {
      if (i == static_cast<int>(h_lines.size()) ||
          h_lines[i] > h_lines[i - 1] + 10) {
        float cluster_sum = 0;
        int count = i - cluster_start;
        for (int j = cluster_start; j < i; j++) {
          cluster_sum += h_lines[j];
        }
        h_clusters.push_back({cluster_sum / count, count});
        cluster_start = i;
      }
    }
    // Sort by vote count (descending)
    std::sort(h_clusters.begin(), h_clusters.end(),
              [](auto &a, auto &b) { return a.second > b.second; });

    // Find best pair with valid spacing (60-120 for horizontal)
    std::vector<float> clusterized_h_lines;
    for (size_t i = 0; i < h_clusters.size() && clusterized_h_lines.size() < 2;
         i++) {
      for (size_t j = i + 1;
           j < h_clusters.size() && clusterized_h_lines.size() < 2; j++) {
        float spacing = std::abs(h_clusters[i].first - h_clusters[j].first);
        if (spacing >= 50 && spacing <= 200) {
          clusterized_h_lines = {h_clusters[i].first, h_clusters[j].first};
          break;
        }
      }
    }
    std::sort(clusterized_h_lines.begin(), clusterized_h_lines.end());

    // vertical lines (same logic)
    std::vector<std::pair<float, int>> v_clusters;
    cluster_start = 0;
    for (int i = 1; i <= static_cast<int>(v_lines.size()); i++) {
      if (i == static_cast<int>(v_lines.size()) ||
          v_lines[i] > v_lines[i - 1] + 20) {
        float cluster_sum = 0;
        int count = i - cluster_start;
        for (int j = cluster_start; j < i; j++) {
          cluster_sum += v_lines[j];
        }
        v_clusters.push_back({cluster_sum / count, count});
        cluster_start = i;
      }
    }
    std::sort(v_clusters.begin(), v_clusters.end(),
              [](auto &a, auto &b) { return a.second > b.second; });

    // Find best pair with valid spacing (70-130 for vertical)
    std::vector<float> clusterized_v_lines;
    for (size_t i = 0; i < v_clusters.size() && clusterized_v_lines.size() < 2;
         i++) {
      for (size_t j = i + 1;
           j < v_clusters.size() && clusterized_v_lines.size() < 2; j++) {
        float spacing = std::abs(v_clusters[i].first - v_clusters[j].first);
        if (spacing >= 50 && spacing <= 300) {
          clusterized_v_lines = {v_clusters[i].first, v_clusters[j].first};
          break;
        }
      }
    }
    std::sort(clusterized_v_lines.begin(), clusterized_v_lines.end());
    // Second filter for grid lines only
    std::vector<float> grid_h, grid_v;

    for (auto x : clusterized_v_lines) {
      if (x > 600 && x < 1500) { // not near top/bottom edge
        grid_v.push_back(x);
      }
    }
    for (auto y : clusterized_h_lines) {
      if (y > 300 && y < 750) { // not near top/bottom edge
        grid_h.push_back(y);
      }
    }

    if (grid_v.size() < 2 || grid_h.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "Not enough grid lines detected");
    }

    detected_grid_lines.push_back(grid_h);
    detected_grid_lines.push_back(grid_v);
    return detected_grid_lines;
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

  std::vector<cv::KeyPoint> detect_circles(cv::Mat image) {
    // MSER detector
    cv::Ptr<cv::MSER> detector = cv::MSER::create();

    std::vector<cv::KeyPoint> fs;
    detector->detect(image, fs);

    std::sort(fs.begin(), fs.end(),
              [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                return a.size > b.size; // descending
              });

    std::vector<cv::KeyPoint> sfs;
    sfs.reserve(fs.size());
    for (const auto &x : fs) {
      if (!suppressByLargerNearby(x, fs)) {
        sfs.push_back(x);
      }
    }

    return sfs;
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

  void print_circles(std::vector<cv::KeyPoint> detected_circles,
                     cv::Mat image) {
    const cv::Scalar d_red(65, 55, 150);   // BGR
    const cv::Scalar l_red(200, 200, 250); // BGR

    for (const auto &circle : detected_circles) {
      const cv::Point center(cvRound(circle.pt.x), cvRound(circle.pt.y));
      const int radius = cvRound(circle.size / 2.0f);

      cv::circle(image, center, radius / 5, d_red, 2, cv::LINE_AA);
      cv::circle(image, center, radius / 5, l_red, 1, cv::LINE_AA);
    }
  }

  void find_crosses(cv::Mat processed_image, cv::Mat processed_templ,
                    cv::Mat image) {
    // Template width and height
    const int w = processed_templ.cols;
    const int h = processed_templ.rows;

    cv::Mat res;
    cv::matchTemplate(processed_image, processed_templ, res,
                      cv::TM_CCOEFF_NORMED);

    // Create mask to 
    cv::Mat masked = cv::Mat::zeros(res.size(), res.type());

    cv::Point c = grid_centers_[8];
    float square_height = std::abs(center_square_infos_.first[0]);
    float square_length = std::abs(center_square_infos_.first[1]);

    int res_x0 = c.x - square_length / 2;
    int res_y0 = c.y - square_height / 2;
    int res_x1 = c.x + square_length / 2;
    int res_y1 = c.y + square_height / 2;

    if (res_x1 > res_x0 && res_y1 > res_y0) {
      cv::Rect roi(res_x0, res_y0, res_x1 - res_x0, res_y1 - res_y0);
      res(roi).copyTo(masked(roi));
      res = masked;
    }

    cv::rectangle(image, cv::Point(res_x0, res_y0), cv::Point(res_x1, res_y1),
                  cv::Scalar(0, 0, 255), 2);

    // Threshold
    const double threshold = 0.3;

    // Find all locations with score >= threshold
    for (int y = 0; y < masked.rows; ++y) {
      for (int x = 0; x < masked.cols; ++x) {
        float score = masked.at<float>(y, x);
        if (score >= threshold) {
          cv::rectangle(image, cv::Point(x, y), cv::Point(x + w, y + h),
                        cv::Scalar(0, 255, 255), 2);
        }
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  // Image variables
  cv::Mat processed_image_;
  std::vector<cv::Point> grid_centers_;
  std::pair<std::array<float, 2>, cv::Point> center_square_infos_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}