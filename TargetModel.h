#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

namespace Lightning
{

class TargetModel
{

public:
    std::vector<cv::Point3d> GetKeyPoints() { return _keyPoints; }

    std::vector<cv::Point3d> GetTargetAxes() { return _targetAxes; }

private:

    // 3D position of target keypoints in millimeters relative to target origin
    std::vector<cv::Point3d> _keyPoints;

    // 3D points for target axes - mainly used for debug images
    std::vector<cv::Point3d> _targetAxes;
};

}