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

        // TODO get from camera calibration
        _cameraMatrix.at<double>(0,0) = 544.4283532337362;
        _cameraMatrix.at<double>(0,1) = 0;
        _cameraMatrix.at<double>(0,2) = 316.1598132539191;
        _cameraMatrix.at<double>(1,0) = 0;
        _cameraMatrix.at<double>(1,1) = 546.3692871964042;
        _cameraMatrix.at<double>(1,2) = 236.8928602692628;
        _cameraMatrix.at<double>(2,0) = 0;
        _cameraMatrix.at<double>(2,1) = 0;
        _cameraMatrix.at<double>(2,2) = 1;

        _distanceCoefficients.at<double>(0,0) = -0.1490001736257689;
        _distanceCoefficients.at<double>(1,0) = 0.5220732935095163;
        _distanceCoefficients.at<double>(2,0) = 0.00142439391154711;
        _distanceCoefficients.at<double>(3,0) = -0.002128107879961602;
        _distanceCoefficients.at<double>(4,0) = -0.8172294974012634;
    }

    cv::Mat GetCameraMatrix() const { return _cameraMatrix; }
    cv::Mat GetDistanceCoefficients() const { return _distanceCoefficients; }

private:
    cv::Mat _cameraMatrix;
    cv::Mat _distanceCoefficients;

};

}