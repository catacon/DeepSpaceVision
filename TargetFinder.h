#pragma once

#include <memory>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

namespace Lightning
{

class Setup;
class TargetModel;
class CameraModel;
class VisionData;

class TargetFinder
{

public:

    TargetFinder(std::shared_ptr<Setup>, std::shared_ptr<spdlog::logger>, std::unique_ptr<TargetModel>, std::unique_ptr<CameraModel>);

    bool Process(cv::Mat&, VisionData&);

private:

    std::shared_ptr<spdlog::logger> _logger;
    std::shared_ptr<Setup> _setup;

    std::unique_ptr<TargetModel> _targetModel;
    std::unique_ptr<CameraModel> _cameraModel;
};

}