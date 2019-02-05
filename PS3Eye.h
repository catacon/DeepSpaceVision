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
        _cameraMatrix.at<double>(0,0) = 5.4196392293730526e+02;
        _cameraMatrix.at<double>(0,1) = 0;
        _cameraMatrix.at<double>(0,2) = 3.1247292273005922e+02;
        _cameraMatrix.at<double>(1,0) = 0;
        _cameraMatrix.at<double>(1,1) = 5.4124252906352831e+02;
        _cameraMatrix.at<double>(1,2) = 2.3289573811286698e+02;
        _cameraMatrix.at<double>(2,0) = 0;
        _cameraMatrix.at<double>(2,1) = 0;
        _cameraMatrix.at<double>(2,2) = 1;

        _distanceCoefficients.at<double>(0,0) = -1.0671238646724683e-01;
        _distanceCoefficients.at<double>(1,0) = 1.4710437279644903e-01;
        _distanceCoefficients.at<double>(2,0) = 1.4590162010940562e-03;
        _distanceCoefficients.at<double>(3,0) = -1.6469380429579145e-03;
        _distanceCoefficients.at<double>(4,0) = 0;
    }
};
}