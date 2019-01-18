#pragma once

#include <memory>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

#include "TargetModel.h"
#include "CameraModel.h"

namespace Lightning
{

class Setup;
class VisionData;

class TargetFinder
{

public:

    TargetFinder(std::shared_ptr<Setup>, std::shared_ptr<spdlog::logger>, TargetModel, CameraModel);

    bool Process(cv::Mat&, VisionData&);

private:

    std::shared_ptr<spdlog::logger> _logger;
    std::shared_ptr<Setup> _setup;

    TargetModel _targetModel;
    CameraModel _cameraModel;
};

}