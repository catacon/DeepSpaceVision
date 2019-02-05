#pragma once

#include <opencv2/opencv.hpp>

namespace Lightning
{

class CameraModel
{

public:

    CameraModel()
    {
        _cameraMatrix = cv::Mat(3, 3, CV_64FC1);
        _distanceCoefficients = cv::Mat(5, 1, CV_64FC1);
    }

    cv::Mat GetCameraMatrix() const { return _cameraMatrix; }
    cv::Mat GetDistanceCoefficients() const { return _distanceCoefficients; }

protected:
    cv::Mat _cameraMatrix;
    cv::Mat _distanceCoefficients;

};

}