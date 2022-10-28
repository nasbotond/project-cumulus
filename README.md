# project-cumulus

## Description
This code compares three different disparity map calculation approaches, i.e., naive approach, dynamic programming approach, and the built-in OpenCV StereoSGBM approach, to a ground truth disparity map. Using the disparity maps, the point clouds are reconstructed including an oriented point cloud and a rudimentary triangulated surface mesh.

## Requirements
- Working C++ compiler (C++ 17 or greater)
- CMake (version >= 2.8)
- OpenCV (version >= 4.0.0)
- PCL

## Usage
- `./stereo <path to left image> <path to right image> <path to ground truth disparity image> <output file prefix> <window size> <lambda> <dmin>`

## Example Results

| ![Naive Disp](example_results/laundry_7_naive_disp.png) | 
|:--:| 
| *Disparity map calculated using Naive approach* |

| ![DP Disp](example_results/laundry_7_dp_disp.png) | 
|:--:| 
| *Disparity map calculated using dynamic programming approach* |

| ![OpenCV Disp](example_results/laundry_7_opencv_disp.png) | 
|:--:| 
| *Disparity map calculated using OpenCV SGBM approach* |

| ![DP PC](example_results/pc_dp.png) | 
|:--:| 
| *Point cloud from disparity map calculated using dynamic programming approach* |
