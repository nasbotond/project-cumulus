#pragma once

void writePLY(const std::string& output_file, std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, std::vector<cv::Point3i> colors);