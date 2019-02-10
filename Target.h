# pragma once

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

#include "VisionData.hpp"


typedef std::pair<cv::Point2d, cv::Point2d> TargetCorner;

namespace Lightning
{

class TargetSection
{
public:
    std::vector<cv::Point2f> corners;
    cv::RotatedRect rect;
    double score; 
    cv::Point2f center;
    double area;
};

class Target
{
public:
    std::vector<TargetSection> sections;
    cv::Point2f center;
    VisionData data;
    cv::Mat rvec;
    cv::Mat tvec;

    void GetInverseTransforms(cv::Mat&, cv::Mat&) const;
};

}