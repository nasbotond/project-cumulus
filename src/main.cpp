#include <opencv2/opencv.hpp>
#include <iostream>
#include <string> 
#include <fstream>
#include <sstream>
#include "main.h"

int main(int argc, char** argv) {

  ////////////////
  // Parameters //
  ////////////////

  // camera setup parameters
  const double focal_length = 3740; // pixels 1247;
  const double baseline = 160; // mm 213;

  // stereo estimation parameters
  // const int dmin = 150; // 67; //230;
  // const int window_size = 3;

  ///////////////////////////
  // Commandline arguments //
  ///////////////////////////

  if (argc < 8) {
    std::cerr << "Usage: " << argv[0] << " LEFT_IMAGE RIGHT_IMAGE GT_IMAGE OUTPUT_FILE WINDOW_SIZE LAMBDA DMIN" << std::endl;
    return 1;
  }

  cv::Mat l_image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  cv::Mat r_image = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
  cv::Mat gt = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
  const std::string output_file = argv[4];
  const int window_size = atoi(argv[5]);
  const int lambda = atoi(argv[6]);
  const int dmin = atoi(argv[7]);

  if (!r_image.data) {
    std::cerr << "No r_image data" << std::endl;
    return EXIT_FAILURE;
  }

  if (!l_image.data) {
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

  int height = r_image.size().height;
  int width = r_image.size().width;

  ////////////////////
  // Reconstruction //
  ////////////////////

  // Naive disparity image
  //cv::Mat naive_disparities = cv::Mat::zeros(height - window_size, width - window_size, CV_8UC1);
  cv::Mat naive_disparities = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat dp_disparities = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat opencv_disparities = cv::Mat::zeros(height, width, CV_8UC1);

  StereoEstimation_Naive(
    window_size, dmin, height, width,
    r_image, l_image,
    naive_disparities);

  StereoEstimation_DP(
    window_size, dmin, height, width, lambda,
    r_image, l_image,
    dp_disparities);

  StereoEstimation_OpenCV(
    window_size, dmin,
    r_image, l_image,
    opencv_disparities);

  ////////////
  // Output //
  ////////////

  // reconstruction Naive
  Disparity2PointCloud(
    output_file + "_naive",
    height, width, naive_disparities,
    window_size, dmin, baseline, focal_length);

  // reconstruction DP
  Disparity2PointCloud(
    output_file + "_dp",
    height, width, dp_disparities,
    window_size, dmin, baseline, focal_length);

  // reconstruction OpenCV
  Disparity2PointCloud(
    output_file + "_opencv",
    height, width, opencv_disparities,
    window_size, dmin, baseline, focal_length);

  // save / display images
  std::stringstream out1;
  out1 << output_file << "_naive.png";
  cv::imwrite(out1.str(), naive_disparities);

  std::stringstream out2;
  out2 << output_file << "_dp.png";  
  cv::imwrite(out2.str(), dp_disparities);

  std::stringstream out3;
  out3 << output_file << "_opencv.png";
  cv::imwrite(out3.str(), opencv_disparities);

  // Compare and get similarity measures
  cv::Mat mad_naive = mad(naive_disparities, gt);
  std::cout << cv::mean(mad_naive) << std::endl;
  cv::imwrite("mad_naive.png", mad_naive);
  cv::Mat sad_naive = ssd(naive_disparities, gt);
  std::cout << cv::mean(sad_naive) << std::endl;
  cv::imwrite("sad_naive.png", sad_naive);
  cv::Mat ssim_naive = MSSIM(naive_disparities, gt);
  std::cout << cv::mean(ssim_naive) << std::endl;
  cv::imwrite("ssim_naive.png", ssim_naive);

  cv::Mat mad_dp = mad(dp_disparities, gt);
  std::cout << cv::mean(mad_dp) << std::endl;
  cv::imwrite("mad_dp.png", mad_dp);
  cv::Mat sad_dp = ssd(dp_disparities, gt);
  std::cout << cv::mean(sad_dp) << std::endl;
  cv::imwrite("sad_dp.png", sad_dp);
  cv::Mat ssim_dp = MSSIM(dp_disparities, gt);
  std::cout << cv::mean(ssim_dp) << std::endl;
  cv::imwrite("ssim_dp.png", ssim_dp);

  cv::Mat mad_opencv = mad(opencv_disparities, gt);
  std::cout << cv::mean(mad_opencv) << std::endl;
  cv::imwrite("mad_opencv.png", mad_opencv);
  cv::Mat sad_opencv = ssd(opencv_disparities, gt);
  std::cout << cv::mean(sad_opencv) << std::endl;
  cv::imwrite("sad_opencv.png", sad_opencv);
  cv::Mat ssim_opencv = MSSIM(opencv_disparities, gt);
  std::cout << cv::mean(ssim_opencv) << std::endl;
  cv::imwrite("ssim_opencv.png", ssim_opencv);

  // cv::namedWindow("DP", cv::WINDOW_AUTOSIZE);
  // cv::imshow("DP", dp_disparities);

  // cv::namedWindow("Naive", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Naive", naive_disparities);

  // cv::namedWindow("OpenCV", cv::WINDOW_AUTOSIZE);
  // // cv::imshow("OpenCV", opencv_disparities);
  // cv::imshow("OpenCV", mad(opencv_disparities, cv::imread("../data/art_disp_1.png", cv::IMREAD_GRAYSCALE)));
  // std::cout << cv::mean(mad(opencv_disparities, cv::imread("../data/art_disp_1.png", cv::IMREAD_GRAYSCALE))) << std::endl;
  
  // cv::waitKey(0);

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
          // options: SSD, SAD, ...
          float sum = 0;

          for(int u = -half_window_size; u <= half_window_size; ++u)
          {
            for(int v = -half_window_size; v <= half_window_size; ++v)
            {
              float i1 = static_cast<float>(l_image.at<uchar>(y_0 + u, i + v)); // left image
              float i2 = static_cast<float>(r_image.at<uchar>(y_0 + u, j + v)); // right image
              sum += std::abs(i1-i2); // SAD
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

      for(int x = 1; x < width; ++x)
      {
        C.at<float>(0, x) = x*static_cast<float>(lambda);
        M.at<uchar>(0, x) = 1;

        C.at<float>(x, 0) = x*static_cast<float>(lambda);
        M.at<uchar>(x, 0) = 2;
      }

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

    // cv::normalize(dp_disparities, dp_disparities, 255, 0, cv::NORM_MINMAX);

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

  int numberOfDisparities = 128;
  stereo->setMinDisparity(0);
  stereo->setNumDisparities(numberOfDisparities);
	stereo->setBlockSize(window_size);
  stereo->setP1(8*window_size*window_size);
  stereo->setP2(32*window_size*window_size);
  stereo->setMode(cv::StereoSGBM::MODE_HH);

  double minVal; double maxVal;
  stereo->compute(r_image, l_image, disp); // images flipped for some reason?
  cv::minMaxLoc(disp, &minVal, &maxVal);
  disp.convertTo(opencv_disparities, CV_8UC1, 255./(maxVal - minVal));
}

cv::Mat mad(const cv::Mat& disp_est, const cv::Mat& disp_gt)
{
  // MAD:
  cv::Mat tmp;
  cv::absdiff(disp_est, disp_gt, tmp);
  return tmp;

  // NCC: template matching
  // SSIM: c++ opencv contrib
}

cv::Mat ssd(const cv::Mat& disp_est, const cv::Mat& disp_gt)
{
  int height = disp_est.rows;
  int width = disp_est.cols;
  cv::Mat ssd = cv::Mat::zeros(height, width, CV_8UC1);

  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      ssd.at<uchar>(i, j) = std::abs(pow((disp_gt.at<uchar>(i, j) - disp_est.at<uchar>(i, j)), 2));
    }
  }
  return ssd;
}

cv::Mat MSSIM(const cv::Mat& i1, const cv::Mat& i2)
{
    const double C1 = 6.5025, C2 = 58.5225;
    /***************************** INITS **********************************/
    int d = CV_32F;
    cv::Mat I1, I2;
    i1.convertTo(I1, d);            // cannot calculate on one byte large values
    i2.convertTo(I2, d);
    cv::Mat I2_2   = I2.mul(I2);        // I2^2
    cv::Mat I1_2   = I1.mul(I1);        // I1^2
    cv::Mat I1_I2  = I1.mul(I2);        // I1 * I2
    /*************************** END INITS **********************************/

    cv::Mat mu1, mu2;                   // PRELIMINARY COMPUTING
    cv::GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
    cv::GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);
    cv::Mat mu1_2   =   mu1.mul(mu1);
    cv::Mat mu2_2   =   mu2.mul(mu2);
    cv::Mat mu1_mu2 =   mu1.mul(mu2);
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
    t3 = t1.mul(t2);                 // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))
    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);                 // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

    cv::Mat ssim_map;
    cv::divide(t3, t1, ssim_map);        // ssim_map =  t3./t1;
    // cv::Scalar mssim = cv::mean(ssim_map);   // mssim = average of ssim map
    cv::normalize(ssim_map, ssim_map, 255, 0, cv::NORM_MINMAX);
    return ssim_map;
}

void Disparity2PointCloud(
  const std::string& output_file,
  int height, int width, cv::Mat& disparities,
  const int& window_size,
  const int& dmin, const double& baseline, const double& focal_length)
{
  std::stringstream out3d;
  out3d << output_file << ".xyz";
  std::ofstream outfile(out3d.str());
  for (int i = 0; i < height - window_size; ++i) 
  {
    std::cout << "Reconstructing 3D point cloud from disparities... " << std::ceil(((i) / static_cast<double>(height - window_size + 1)) * 100) << "%\r" << std::flush;
    for (int j = 0; j < width - window_size; ++j) 
    {
      if (disparities.at<uchar>(i, j) == 0) continue;

      const double u1 = j-(width/2);
      const double d = static_cast<double>(disparities.at<uchar>(i, j)) + dmin;
      // const double d = static_cast<double>(disparities.at<uchar>(i, j));
      const double u2 = u1-d; // u1+d
      const double v = i-(height/2);

      const double Z = (baseline*focal_length)/d;
      const double X = -1*(baseline*(u1+u2))/(2*d);
      const double Y = baseline*(v)/d;
	  
      outfile << X << " " << Y << " " << Z << std::endl;
    }
  }

  std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
  std::cout << std::endl;
}

// compute similarity metrics between the ground truth disparity provided in the middlebury dataset and the one that you computed
// metrics to use: SSD, SSIM, NCC (normalized cross correlation)
// sum of squared diffrences
// also look out how the ground truth disparity maps are scaled