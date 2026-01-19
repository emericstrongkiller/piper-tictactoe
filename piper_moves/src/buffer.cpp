// 1. grid setter outputs also square of the whole grid
// 2. then another function will scan only thie part in the future to   get
// the status of each square of the grid

// get circles + draw them
std::vector<cv::Point> circles = find_circles(image);
for (size_t i = 0; i < circles.size(); i++) {
  cv::circle(image, circles[i], 20, cv::Scalar(0, 0, 255));
}

std::vector<cv::Point> find_circles(const cv::Mat &image) {
  cv::Mat processed_image_;
  cv::cvtColor(image, processed_image_, cv::COLOR_BGR2GRAY);
  cv::medianBlur(processed_image_, processed_image_, 5);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(
      processed_image_, circles, cv::HOUGH_GRADIENT,
      1.5, // dp (try 1.2–2.0)
      60,  // minDist between centers (increase if you still get many)
      120, // param1 (Canny high threshold)
      35,  // param2 (accumulator threshold) -> increase to get fewer circles
      80,  // minRadius (tune!)
      100  // maxRadius (tune!)
  );

  // Extract center points from detected circles
  std::vector<cv::Point> centers;
  for (const auto &circle : circles) {
    centers.push_back(cv::Point(circle[0], circle[1]));
  }

  /*
// draw initialy filtered lines
for (size_t i = 0; i < lines.size(); i++) {
  cv::line(image, cv::Point(lines[i][0], lines[i][1]),
           cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3,
           8);
}
RCLCPP_INFO(this->get_logger(), "Found %zu lines", lines.size());
*/