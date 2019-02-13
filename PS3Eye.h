#pragma once

#include "CameraModel.h"

namespace Lightning
{
class PS3EyeModel : public CameraModel
{
public:
    PS3EyeModel()
    {
        _cameraMatrix = cv::Mat(3, 3, CV_64FC1);
        _distanceCoefficients = cv::Mat(5, 1, CV_64FC1);

        // TODO get from camera calibration
        _cameraMatrix.at<double>(0,0) = 5.3978998477177777e+02;
        _cameraMatrix.at<double>(0,1) = 0;
        _cameraMatrix.at<double>(0,2) = 3.1387384515857258e+02;
        _cameraMatrix.at<double>(1,0) = 0;
        _cameraMatrix.at<double>(1,1) = 5.3959736049747960e+02;
        _cameraMatrix.at<double>(1,2) = 2.3186414031626754e+02;
        _cameraMatrix.at<double>(2,0) = 0;
        _cameraMatrix.at<double>(2,1) = 0;
        _cameraMatrix.at<double>(2,2) = 1;

        _distanceCoefficients.at<double>(0,0) = -1.2177044514044434e-01;
        _distanceCoefficients.at<double>(1,0) = 1.6107320330688607e-01;
        _distanceCoefficients.at<double>(2,0) = -1.0523229353437240e-03;
        _distanceCoefficients.at<double>(3,0) = -3.2604889426788471e-03;
        _distanceCoefficients.at<double>(4,0) = 0;
    }
};
}