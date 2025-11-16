#include "img_tools.hpp"

namespace tools
{
void draw_point(cv::Mat & img, const cv::Point & point, const cv::Scalar & color, int radius)
{
  cv::circle(img, point, radius, color, -1);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
  std::vector<std::vector<cv::Point>> contours = {points};
  cv::drawContours(img, contours, -1, color, thickness);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
  std::vector<cv::Point> int_points(points.begin(), points.end());
  draw_points(img, int_points, color, thickness);
}

void draw_text(
  cv::Mat & img, const std::string & text, const cv::Point & point, const cv::Scalar & color,
  double font_scale, int thickness)
{
  cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness);
}

void letterbox(const cv::Mat & image, cv::Mat & out, double & scale)
{
  int w = image.cols;
  int h = image.rows;
  int _w = out.cols;
  int _h = out.rows;
  scale = std::min(static_cast<double>(_w) / w, static_cast<double>(_h) / h);
  int new_w = static_cast<int>(w * scale);
  int new_h = static_cast<int>(h * scale);
  cv::Mat resized_img;
  cv::resize(image, resized_img, cv::Size(new_w, new_h));
  int top = (_h - new_h) / 2;
  int bottom = _h - new_h - top;
  int left = (_w - new_w) / 2;
  int right = _w - new_w - left;
  cv::copyMakeBorder(resized_img, out, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
}

}  // namespace tools