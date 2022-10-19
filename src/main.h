#pragma once

void StereoEstimation_Naive(
  const int& window_size,
  const int& dmin,
  int height,
  int width,
  cv::Mat& r_image, cv::Mat& l_image, cv::Mat& naive_disparities);

void StereoEstimation_DP(
  const int& window_size,
  const int& dmin,
  int height,
  int width,
  int lambda,
  cv::Mat& r_image, cv::Mat& l_image, cv::Mat& dp_disparities);

void StereoEstimation_OpenCV(
  const int& window_size,
  const int& dmin,
  cv::Mat& r_image, cv::Mat& l_image, cv::Mat& opencv_disparities);

cv::Mat mad(const cv::Mat& disp_est, const cv::Mat& disp_gt);

cv::Mat ssd(const cv::Mat& disp_est, const cv::Mat& disp_gt);

cv::Mat MSSIM(const cv::Mat& i1, const cv::Mat& i2);

pcl::visualization::PCLVisualizer::Ptr pointCloudVisualization (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);

void Disparity2PointCloud(
  const std::string& output_file,
  int height, int width, cv::Mat& disparities,
  const int& window_size,
  const int& dmin, const double& baseline, const double& focal_length, cv::Mat& l_image_color);
