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
        _cameraMatrix.at<double>(0,0) = 4.3437671163346170e+02;
        _cameraMatrix.at<double>(0,1) = 0;
        _cameraMatrix.at<double>(0,2) = 3.8936743695763329e+02;
        _cameraMatrix.at<double>(1,0) = 0;
        _cameraMatrix.at<double>(1,1) = 4.2758461537279669e+02;
        _cameraMatrix.at<double>(1,2) = 2.5252602377767519e+02;
        _cameraMatrix.at<double>(2,0) = 0;
        _cameraMatrix.at<double>(2,1) = 0;
        _cameraMatrix.at<double>(2,2) = 1;

        _distanceCoefficients.at<double>(0,0) = -1.6166081617619424e-01;
        _distanceCoefficients.at<double>(1,0) = 6.6144114895915318e-02;
        _distanceCoefficients.at<double>(2,0) = -9.9544467794404016e-04;
        _distanceCoefficients.at<double>(3,0) = 1.5263799584402009e-02;
        _distanceCoefficients.at<double>(4,0) = 0;
    }

    cv::Mat GetCameraMatrix() const { return _cameraMatrix; }
    cv::Mat GetDistanceCoefficients() const { return _distanceCoefficients; }

private:
    cv::Mat _cameraMatrix;
    cv::Mat _distanceCoefficients;

};

}