cv::Mat masked = cv::Mat::zeros(res.size(), res.type());

// float square_height = center_square_infos_.first[0];
// float square_length = center_square_infos_.first[1];
float square_height = 500;
float square_length = 500;

// ROI in IMAGE coords around grid_centers_[0]
cv::Point c = grid_centers_[0];

// TEMPPP
cv::rectangle(image,
              cv::Point(c.x - square_height / 2, c.y - square_height / 2),
              cv::Point(c.x + square_height / 2, c.y + square_height / 2),
              cv::Scalar(0, 255, 255), 2);

int x0 = std::max(0, (int)std::round(c.x - square_length / 2));
int y0 = std::max(0, (int)std::round(c.y - square_height / 2));
int x1 =
    std::min(processed_image.cols, (int)std::round(c.x + square_length / 2));
int y1 =
    std::min(processed_image.rows, (int)std::round(c.y + square_height / 2));

// convert IMAGE ROI -> RES ROI (res is smaller by template size)
x1 = std::min(x1, res.cols);
y1 = std::min(y1, res.rows);

x0 = std::clamp(x0, 0, res.cols - 1);
y0 = std::clamp(y0, 0, res.rows - 1);
x1 = std::clamp(x1, 0, res.cols - 1);
y1 = std::clamp(y1, 0, res.rows - 1);

x0 -= w / 2;
y0 -= h / 2;
x1 -= w / 2;
y1 -= h / 2;

x0 = std::clamp(x0, 0, res.cols - 1);
y0 = std::clamp(y0, 0, res.rows - 1);
x1 = std::clamp(x1, 0, res.cols);
y1 = std::clamp(y1, 0, res.rows);

if (x1 <= x0 || y1 <= y0)
  return; // IMPORTANT

cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
res(roi).copyTo(masked(roi));
res = masked;

//

//

//
//
//
//

cv::Point c = grid_centers_[0];
float square_height = center_square_infos_.first[0];
float square_length = center_square_infos_.first[1];

// Calculate ROI in IMAGE coordinates
int img_x0 = std::max(0, (int)(c.x - square_length / 2));
int img_y0 = std::max(0, (int)(c.y - square_height / 2));
int img_x1 = std::min(processed_image.cols, (int)(c.x + square_length / 2));
int img_y1 = std::min(processed_image.rows, (int)(c.y + square_height / 2));

// Convert to RES coordinates (result is (img.size - template.size + 1))
int res_x0 = std::max(0, img_x0);
int res_y0 = std::max(0, img_y0);
int res_x1 = std::min(res.cols, img_x1 - w + 1);
int res_y1 = std::min(res.rows, img_y1 - h + 1);