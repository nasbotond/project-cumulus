#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <numeric>
#include <cmath>
#include <stdint.h>

#include "ply_writer.h"

void writePLY(const std::string& output_file, std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, std::vector<cv::Point3i> colors)
{
    int num = points.size();
    std::ofstream myfile;
    myfile.open(output_file);
    
    myfile <<"ply\n";
    myfile <<"format ascii 1.0\n";
    myfile <<"element vertex "<< num <<std::endl;
    myfile <<"property float x\n";
    myfile <<"property float y\n";
    myfile <<"property float z\n";
    myfile <<"property float nx\n";
    myfile <<"property float ny\n";
    myfile <<"property float nz\n";
    myfile <<"property uchar red\n";
    myfile <<"property uchar green\n";
    myfile <<"property uchar blue\n";
    myfile <<"end_header\n";    
    
    for (int idx=0; idx<num; idx++)
    {
        cv::Point3f point=points.at(idx);
        cv::Point3f normal=normals.at(idx);
        cv::Point3i color=colors.at(idx);
        
        myfile <<point.x << " " << point.y << " " << point.z << " " << normal.x << " " << normal.y << " " << normal.z << " " << color.x << " " << color.y << " " <<color.z <<std::endl;        
    }
        
    myfile.close();
}