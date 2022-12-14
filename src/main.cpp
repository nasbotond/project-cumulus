#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string> 
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "main.h"

using namespace std::chrono_literals;

int main(int argc, char** argv) 
{
  // camera setup parameters
  const double focal_length = 3740;
  const double baseline = 160;

  // check if correct number of parameters are given
  if (argc < 8) 
  {
    std::cerr << "Usage: " << argv[0] << " LEFT_IMAGE RIGHT_IMAGE GT_IMAGE OUTPUT_FILE WINDOW_SIZE LAMBDA DMIN" << std::endl;
    return 1;
  }

  // needed for coloring the point cloud
  cv::Mat image_color = cv::imread(argv[2], cv::IMREAD_COLOR);

  // read image files and set variables
  cv::Mat l_image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  cv::Mat r_image = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
  cv::Mat gt = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
  const std::string output_file = argv[4];
  const int window_size = atoi(argv[5]);
  const int lambda = atoi(argv[6]);
  const int dmin = atoi(argv[7]);

  if (!r_image.data) 
  {
    std::cerr << "No r_image data" << std::endl;
    return EXIT_FAILURE;
  }

  if (!l_image.data) 
  {
    std::cerr << "No l_image data" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "------------------ Parameters -------------------" << std::endl;
  std::cout << "focal_length = " << focal_length << std::endl;
  std::cout << "baseline = " << baseline << std::endl;
  std::cout << "window_size = " << window_size << std::endl;
  std::cout << "disparity added due to image cropping = " << dmin << std::endl;
  std::cout << "DP lambda = " << lambda << std::endl;
  std::cout << "output filename = " << argv[4] << std::endl;
  std::cout << "-------------------------------------------------" << std::endl;

  // image dimensions
  int height = r_image.size().height;
  int width = r_image.size().width;

  // disparity images
  cv::Mat naive_disparities = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat dp_disparities = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat opencv_disparities = cv::Mat::zeros(height, width, CV_8UC1);

  // disparity calculations with execution time measured
  // auto startNaive = std::chrono::steady_clock::now();
  // StereoEstimation_Naive(
  //   window_size, dmin, height, width,
  //   r_image, l_image,
  //   naive_disparities);

  // auto endNaive = std::chrono::steady_clock::now();
  // std::cout << "Naive approach elapsed time in milliseconds: " << std::chrono::duration_cast<std::chrono::milliseconds>(endNaive - startNaive).count() << " ms" << std::endl;

  // auto startDP = std::chrono::steady_clock::now();
  // StereoEstimation_DP(
  //   window_size, dmin, height, width, lambda,
  //   r_image, l_image,
  //   dp_disparities);

  // auto endDP = std::chrono::steady_clock::now();
  // std::cout << "DP approach elapsed time in milliseconds: " << std::chrono::duration_cast<std::chrono::milliseconds>(endDP - startDP).count() << " ms" << std::endl;

  auto startOpenCV = std::chrono::steady_clock::now();
  StereoEstimation_OpenCV(
    window_size, dmin,
    r_image, l_image,
    opencv_disparities);

  auto endOpenCV = std::chrono::steady_clock::now();
  std::cout << "OpenCV approach elapsed time in milliseconds: " << std::chrono::duration_cast<std::chrono::milliseconds>(endOpenCV - startOpenCV).count() << " ms" << std::endl;

  // save disparity images
  // std::stringstream outNaive;
  // outNaive << output_file << "_naive";
  // cv::imwrite(outNaive.str()+"_disp.png", naive_disparities);

  // std::stringstream outDP;
  // outDP << output_file << "_dp";
  // cv::imwrite(outDP.str()+"_disp.png", dp_disparities);

  std::stringstream outOpenCV;
  outOpenCV << output_file << "_opencv";
  cv::imwrite(outOpenCV.str()+"_disp.png", opencv_disparities);

  // Normalize all disparities to 0-255
  // double minVal; double maxVal;
  // cv::minMaxLoc(naive_disparities, &minVal, &maxVal);

  // std::cout << minVal << std::endl;
  // std::cout << maxVal << std::endl;

  // cv::minMaxLoc(dp_disparities, &minVal, &maxVal);

  // std::cout << minVal << std::endl;
  // std::cout << maxVal << std::endl;

  // cv::minMaxLoc(opencv_disparities, &minVal, &maxVal);

  // std::cout << minVal << std::endl;
  // std::cout << maxVal << std::endl;

  // cv::normalize(gt, gt, 255./(maxVal-minVal), 0, cv::NORM_MINMAX);

  // compare and get similarity measures, save difference images
  // MAD(naive_disparities, gt, outNaive.str());
  // MSE(naive_disparities, gt, outNaive.str());
  // MSSIM(naive_disparities, gt, outNaive.str());
  // NCC(naive_disparities, gt);

  // MAD(dp_disparities, gt, outDP.str());
  // MSE(dp_disparities, gt, outDP.str());
  // MSSIM(dp_disparities, gt, outDP.str());
  // NCC(dp_disparities, gt);

  MAD(opencv_disparities, gt, outOpenCV.str());
  MSE(opencv_disparities, gt, outOpenCV.str());
  MSSIM(opencv_disparities, gt, outOpenCV.str());
  NCC(opencv_disparities, gt);

  // 3D point cloud reconstruction using Naive
  // Disparity2PointCloud(
  //   outNaive.str(),
  //   height, width, naive_disparities,
  //   window_size, dmin, baseline, focal_length, image_color);

  // // 3D point cloud reconstruction using DP
  // Disparity2PointCloud(
  //   outDP.str(),
  //   height, width, dp_disparities,
  //   window_size, dmin, baseline, focal_length, image_color);

  // 3D point cloud reconstruction using OpenCV
  Disparity2PointCloud(
    outOpenCV.str(),
    height, width, opencv_disparities,
    window_size, dmin, baseline, focal_length, image_color);

  // 3D point cloud reconstruction using GT (for visual comparison)
  Disparity2PointCloud(
    output_file+"_gt",
    height, width, gt,
    window_size, dmin, baseline, focal_length, image_color);

  return 0;
}

void StereoEstimation_Naive(
  const int& window_size,
  const int& dmin,
  int height,
  int width,
  cv::Mat& r_image, cv::Mat& l_image, cv::Mat& naive_disparities)
{
  int half_window_size = window_size / 2;

  for (int i = half_window_size; i < height - half_window_size; ++i) 
  {

    std::cout
      << "Calculating disparities for the naive approach... "
      << std::ceil(((i - half_window_size + 1) / static_cast<double>(height - window_size + 1)) * 100) << "%\r"
      << std::flush;

    for (int j = half_window_size; j < width - half_window_size; ++j) 
    {
      int min_ssd = INT_MAX;
      int disparity = 0;

      for (int d = -j + half_window_size; d < width - j - half_window_size; ++d) 
      {
        int ssd = 0;

        for(int u = -half_window_size; u <= half_window_size; ++u)
        {
          for(int v = -half_window_size; v <= half_window_size; ++v)
          {
            // ssd += pow((r_image.at<uchar>(i+u, j+v) - l_image.at<uchar>(i+u, j+v+d)), 2);
            ssd += pow((l_image.at<uchar>(i+u, j+v) - r_image.at<uchar>(i+u, j+v+d)), 2);
          }
        }

        if (ssd < min_ssd)
        {
          min_ssd = ssd;
          disparity = d;
        }
      }

      naive_disparities.at<uchar>(i - half_window_size, j - half_window_size) = std::abs(disparity);
    }
  }

  std::cout << "Calculating disparities for the naive approach... Done.\r" << std::flush;
  std::cout << std::endl;
}

void StereoEstimation_DP(
  const int& window_size,
  const int& dmin,
  int height,
  int width,
  int lambda,
  cv::Mat& r_image, cv::Mat& l_image, cv::Mat& dp_disparities)
{
  int half_window_size = window_size / 2;

  // for each row (scan line)
  for (int y_0 = half_window_size; y_0 < height - half_window_size; ++y_0) 
  {

    std::cout
      << "Calculating disparities for the DP approach... "
      << std::ceil(((y_0 - half_window_size + 1) / static_cast<double>(height - window_size + 1)) * 100) << "%\r"
      << std::flush;

    // dissimilarity(i,j) for each (i,j)
    cv::Mat dissim = cv::Mat::zeros(width, width, CV_32FC1); // go through every pixel in the scan line 

    for (int i = half_window_size; i < width - half_window_size; ++i) // left image
    {
      // for (int j = half_window_size; j < width - half_window_size; ++j) // right image
      for (int j = i; j < width - half_window_size; ++j)
      {
        float sum = 0;

        for(int u = -half_window_size; u <= half_window_size; ++u)
        {
          for(int v = -half_window_size; v <= half_window_size; ++v)
          {
            float i1 = static_cast<float>(l_image.at<uchar>(y_0 + u, i + v)); // left image
            float i2 = static_cast<float>(r_image.at<uchar>(y_0 + u, j + v)); // right image
            sum += std::abs(i1-i2); // absolute difference
            // sum += (i1-i2)*(i1-i2); // SSD
          }
        }
        dissim.at<float>(i, j) = sum;
        dissim.at<float>(j, i) = sum;
      }
    }

    // allocate C (cost), M (points to preceding nodes)
    cv::Mat C = cv::Mat::zeros(width, width, CV_32FC1);
    cv::Mat M = cv::Mat::zeros(width, width, CV_8UC1); // match 0, left-occlusion 1, right-occlusion 2

    // fill first row and column of C and M matrices
    for(int x = 1; x < width; ++x)
    {
      C.at<float>(0, x) = x*static_cast<float>(lambda);
      M.at<uchar>(0, x) = 1;

      C.at<float>(x, 0) = x*static_cast<float>(lambda);
      M.at<uchar>(x, 0) = 2;
    }

    // fill C and M matrices
    for(int r = 1; r < width - 1; ++r)
    {
      for(int c = 1; c < width - 1; ++c)
      {
        double cm = C.at<float>(r-1, c-1) + dissim.at<float>(r, c);
        double cl = C.at<float>(r-1, c) + static_cast<float>(lambda);
        double cr = C.at<float>(r, c-1) + static_cast<float>(lambda);

        double min = std::min({cm, cl, cr}, std::less<double>());
        C.at<float>(r, c) = min;
        M.at<uchar>(r, c) = min == cm ? 0 : min == cl ? 1 : 2;
      }
    }

    // backtrack to get optimal disparities
    int ri = width - 1;
    int li = width - 1;
    int col = width;
    while (ri != 0 && li != 0)
    {
        switch (M.at<uchar>(li, ri))
        {
          case 0:
              dp_disparities.at<uchar>(y_0 - half_window_size, col) = abs(li - ri);
              ri--;
              li--;
              col--;
              break;
          case 1:
              li--;
              break;
          case 2:
              ri--;
              break;
        }
    }
  }

  cv::normalize(dp_disparities, dp_disparities, 255, 0, cv::NORM_MINMAX);

  std::cout << "Calculating disparities for the DP approach... Done.\r" << std::flush;
  std::cout << std::endl;
}

void StereoEstimation_OpenCV(
  const int& window_size,
  const int& dmin,
  cv::Mat& r_image, cv::Mat& l_image, cv::Mat& opencv_disparities)
{
  cv::Mat disp; // Disparity
  cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create();

  // set SGBM parameters
  int numberOfDisparities = 63;
  stereo->setMinDisparity(0);
  stereo->setNumDisparities(numberOfDisparities);
	stereo->setBlockSize(window_size);
  stereo->setP1(8*window_size*window_size);
  stereo->setP2(32*window_size*window_size);
  stereo->setMode(cv::StereoSGBM::MODE_HH);

  std::cout
      << "Calculating disparities for the OpenCV approach... " << "%\r"
      << std::flush;

  double minVal; double maxVal;
  stereo->compute(r_image, l_image, disp); // images flipped for some reason?
  cv::minMaxLoc(disp, &minVal, &maxVal);
  disp.convertTo(opencv_disparities, CV_8UC1, 255./(maxVal - minVal)); // scale to 0-255

  std::cout << "Calculating disparities for the OpenCV approach... Done.\r" << std::flush;
  std::cout << std::endl;
}

// Mean Absolute Difference
void MAD(const cv::Mat& disp_est, const cv::Mat& disp_gt, const std::string& output_file)
{
  cv::Mat mad;
  cv::absdiff(disp_est, disp_gt, mad);
  // cv::normalize(mad, mad, 255, 0, cv::NORM_MINMAX);

  std::cout << "MAD mean: " << cv::mean(mad) << std::endl;
  cv::imwrite(output_file + "_mad.png", mad);
}

// Mean Squared Error
void MSE(const cv::Mat& disp_est, const cv::Mat& disp_gt, const std::string& output_file)
{
  int height = disp_est.rows;
  int width = disp_est.cols;
  cv::Mat ssd = cv::Mat::zeros(height, width, CV_8UC1);

  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      ssd.at<uchar>(i, j) = pow((disp_gt.at<uchar>(i, j) - disp_est.at<uchar>(i, j)), 2);
    }
  }
  // cv::normalize(ssd, ssd, 255, 0, cv::NORM_MINMAX);

  std::cout << "MSE mean: " << cv::mean(ssd) << std::endl;
  cv::imwrite(output_file + "_sad.png", ssd);
}

// Normalized Cross Correlation
void NCC(const cv::Mat& disp_est, const cv::Mat& disp_gt)
{
  int height = disp_est.rows;
  int width = disp_est.cols;
  cv::Mat ncc = cv::Mat::zeros(height, width, CV_32FC1);

  cv::Mat float_gt;
  cv::Mat float_est;
  disp_gt.convertTo(float_gt, CV_32FC1);
  disp_est.convertTo(float_est, CV_32FC1);

  cv::matchTemplate(float_gt, float_est, ncc, cv::TM_CCORR_NORMED);

  std::cout << "NCC mean: " << cv::mean(ncc) << std::endl;
}

// OpenCV Implementation of Structural Similarity Measure
void MSSIM(const cv::Mat& disp_est, const cv::Mat& disp_gt, const std::string& output_file)
{
  const double C1 = 6.5025, C2 = 58.5225;

  int d = CV_32F;
  cv::Mat I1, I2;
  disp_est.convertTo(I1, d); // cannot calculate on one byte large values
  disp_gt.convertTo(I2, d);
  cv::Mat I2_2 = I2.mul(I2); // I2^2
  cv::Mat I1_2 = I1.mul(I1); // I1^2
  cv::Mat I1_I2 = I1.mul(I2); // I1 * I2

  cv::Mat mu1, mu2;
  cv::GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
  cv::GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);
  cv::Mat mu1_2 = mu1.mul(mu1);
  cv::Mat mu2_2 = mu2.mul(mu2);
  cv::Mat mu1_mu2 = mu1.mul(mu2);
  cv::Mat sigma1_2, sigma2_2, sigma12;
  cv::GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
  sigma1_2 -= mu1_2;
  cv::GaussianBlur(I2_2, sigma2_2, cv::Size(11, 11), 1.5);
  sigma2_2 -= mu2_2;
  cv::GaussianBlur(I1_I2, sigma12, cv::Size(11, 11), 1.5);
  sigma12 -= mu1_mu2;

  cv::Mat t1, t2, t3;
  t1 = 2 * mu1_mu2 + C1;
  t2 = 2 * sigma12 + C2;
  t3 = t1.mul(t2); // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))
  t1 = mu1_2 + mu2_2 + C1;
  t2 = sigma1_2 + sigma2_2 + C2;
  t1 = t1.mul(t2); // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

  cv::Mat ssim_map;
  cv::divide(t3, t1, ssim_map); // ssim_map =  t3./t1;
  // cv::Scalar mssim = cv::mean(ssim_map); // mssim = average of ssim map
  std::cout << "SSIM mean: " << cv::mean(ssim_map) << std::endl;

  double minVal; double maxVal;
  cv::minMaxLoc(ssim_map, &minVal, &maxVal);

  cv::normalize(ssim_map, ssim_map, 255./(maxVal-minVal), 0, cv::NORM_MINMAX);
  cv::imwrite(output_file + "_ssim.png", ssim_map);
}

// PCL point cloud visualizer
pcl::visualization::PCLVisualizer::Ptr pointCloudVisualization (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "Cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 50, 10, "normals"); // every 50 normal to display, normal magnitudes scaled by 10
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

// estimate normal vectors for oriented point cloud
void getNormalVectors(cv::Mat& points, cv::Mat& normals, const int& window_size)
{
  int hw = window_size/2; // window size of 5, hw is half window size

  // for each point
  for(int i = hw; i < points.rows - hw; ++i)
  {
    for(int j = hw; j < points.cols - hw; ++j)
    {
      // get window of points
      std::vector<cv::Point3f> pts;
      for(int a = -hw; a <= hw; ++a)
      {
        for(int b = -hw; b <= hw; ++b)
        {
          cv::Point3f point;
          point.x = static_cast<float>(points.at<cv::Vec3f>(i+a, j+b)[0]);
          point.y = static_cast<float>(points.at<cv::Vec3f>(i+a, j+b)[1]);
          point.z = static_cast<float>(points.at<cv::Vec3f>(i+a, j+b)[2]);

          pts.push_back(point);
        }
      }
      
      int num = pts.size();

      // find center of gravity
      cv::Point3d t(0.0, 0.0, 0.0);

      for (int idx = 0; idx < num; idx++)
      {
        t.x += pts.at(idx).x;
        t.y += pts.at(idx).y;
        t.z += pts.at(idx).z;
      }

      t.x = t.x / num;
      t.y = t.y / num;
      t.z = t.z / num;

      // X*l = 0 (homogenous equation)
      cv::Mat X(num, 3, CV_32F);

      // get matrix X (plane matrix)
      for (int idx = 0; idx < num; idx++)
      {
        cv::Point3d pt = pts.at(idx);
        X.at<float>(idx, 0) = pt.x - t.x;
        X.at<float>(idx, 1) = pt.y - t.y;
        X.at<float>(idx, 2) = pt.z - t.z;
      }

      // normal vector l -> eigenvector of X^T X corresponding to the least eigenvalues
      cv::Mat mtx = X.t() * X;
      cv::Mat evals, evecs;

      cv::eigen(mtx, evals, evecs);

      float nx = evecs.at<float>(2, 0);
      float ny = evecs.at<float>(2, 1);
      float nz = evecs.at<float>(2, 2);

      // check if we need to flip any plane normals towards viewpoint
      float vp_x = 0 - static_cast<float>(points.at<cv::Vec3f>(i, j)[0]);
      float vp_y = 0 - static_cast<float>(points.at<cv::Vec3f>(i, j)[1]);
      float vp_z = 0 - static_cast<float>(points.at<cv::Vec3f>(i, j)[2]);

      // dot product between the (viewpoint - point) and the plane normal
      float cos_theta = (vp_x * nx + vp_y * ny + vp_z * nz);

      normals.at<cv::Vec3f>(i, j) = cos_theta < 0 ? cv::Vec3f(-nx, -ny, -nz) : cv::Vec3f(nx, ny, nz);
    }
  }
}

// create and write .ply file for point clouds
void writePLY(const std::string& output_file, cv::Mat points, cv::Mat normals, cv::Mat colors)
{
  int rows = points.rows;
  int cols = points.cols;

  int triangleSize = 3; // size of triangles for triangulated surface

  std::stringstream out3d;
  out3d << output_file << ".ply";
  std::ofstream outfile(out3d.str());
  
  // header
  outfile <<"ply\n";
  outfile <<"format ascii 1.0\n";
  outfile <<"element vertex "<< rows*cols <<std::endl;
  outfile <<"property float x\n";
  outfile <<"property float y\n";
  outfile <<"property float z\n";
  outfile <<"property float nx\n";
  outfile <<"property float ny\n";
  outfile <<"property float nz\n";
  outfile <<"property uchar red\n";
  outfile <<"property uchar green\n";
  outfile <<"property uchar blue\n";
  outfile <<"element face " << 2*((rows-1)/triangleSize)*((cols-1)/triangleSize) << std::endl;
  outfile <<"property list uchar int vertex_index\n";
  outfile <<"end_header\n";
  
  // write point vertices, normals, and colors
  for (int r = 0; r < rows; r++)
  {
    for(int c = 0; c < cols; c++)
    {
      cv::Vec3f point = points.at<cv::Vec3f>(r, c);
      cv::Vec3f normal = normals.at<cv::Vec3f>(r, c);
      cv::Vec3b color = colors.at<cv::Vec3b>(r, c);

      outfile << point.val[0] << " " << point.val[1] << " " << point.val[2] << " " << normal.val[0] << " " << normal.val[1] << " " << normal.val[2] << " " << (int)color.val[0] << " " << (int)color.val[1] << " " << (int)color.val[2] <<std::endl;        
    }
  }

  // determine and write faces
  for (int r = triangleSize; r <= triangleSize*((rows-1)/triangleSize); r=r+triangleSize)
  {
    for(int c = 0; c < triangleSize*((cols-1)/triangleSize); c=c+triangleSize)
    {
      outfile << "3 " << r*cols+c << " " << (r*cols+c)-triangleSize*cols << " " << (r*cols+c)-triangleSize*cols+triangleSize << std::endl;
      outfile << "3 " << r*cols+c << " " << (r*cols+c)-triangleSize*cols+triangleSize << " " << (r*cols+c)+triangleSize << std::endl; 
    }
  }

  outfile.close();
}

// calculate 3D point from disparity map
void Disparity2PointCloud(
  const std::string& output_file,
  int height, int width, cv::Mat& disparities,
  const int& window_size,
  const int& dmin, const double& baseline, const double& focal_length, cv::Mat& image_color)
{
  cv::Mat pointsMat = cv::Mat(height - window_size, width - window_size, CV_32FC3, cv::Scalar(0.,0.,0.));
  cv::Mat colorsMat = cv::Mat(height - window_size, width - window_size, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat normalsMat = cv::Mat(height - window_size, width - window_size, CV_32FC3, cv::Scalar(0.,0.,0.));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  for (int i = 0; i < height - window_size; ++i)
  {
    std::cout << "Reconstructing 3D point cloud from disparities... " << std::ceil(((i) / static_cast<double>(height - window_size + 1)) * 100) << "%\r" << std::flush;
    for (int j = 0; j < width - window_size; ++j)
    {
      if(disparities.at<uchar>(i, j) + dmin == 0) continue;

      const double u1 = j-(width/2);
      const double d = static_cast<double>(disparities.at<uchar>(i, j)) + dmin;
      const double u2 = u1-d; // u1+d
      const double v = i-(height/2);

      const double Z = (baseline*focal_length)/d;
      const double X = -1*(baseline*(u1+u2))/(2*d);
      const double Y = baseline*(v)/d;

      pointsMat.at<cv::Vec3f>(i, j) = cv::Vec3f(-X, Y, Z);

      // set color of point
      cv::Vec3b color = image_color.at<cv::Vec3b>(i, j);
      colorsMat.at<cv::Vec3b>(i, j) = cv::Vec3b(color.val[2], color.val[1], color.val[0]);

      // populate the cloud for PCL visualization and normal calculation
      pcl::PointXYZRGB basic_point;
      basic_point.x = X;
      basic_point.y = Y;
      basic_point.z = Z;
      basic_point.r = color.val[2];
      basic_point.g = color.val[1];
      basic_point.b = color.val[0];
      cloud->points.push_back(basic_point);
    }
  }

  std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
  std::cout << std::endl;

  // estimate normal vectors
  getNormalVectors(pointsMat, normalsMat, 5);

  // write 3D point cloud file
  writePLY(output_file, pointsMat, normalsMat, colorsMat);

  // estimate normal vectors using PCL library (for visual comparison)
  cloud->width = cloud->size ();
  cloud->height = 1;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (5);
  ne.compute (*cloud_normals1);

  // visualize PCL point cloud and normals
  pcl::visualization::PCLVisualizer::Ptr viewer = pointCloudVisualization(cloud, cloud_normals1);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
}