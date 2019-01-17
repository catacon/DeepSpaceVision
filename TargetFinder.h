#pragma once

#include <memory>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

#include "Setup.hpp"
#include "TargetModel.h"
#include "CameraModel.h"

namespace Lightning
{

class TargetFinder
{

public:

    TargetFinder(std::shared_ptr<Setup> setup, std::shared_ptr<spdlog::logger> logger, std::unique_ptr<TargetModel> targetModel, std::unique_ptr<CameraModel> cameraModel);

    bool Process(cv::Mat& image);

private:

    std::shared_ptr<spdlog::logger> _logger;
    std::shared_ptr<Setup> _setup;

    std::unique_ptr<TargetModel> _targetModel;
    std::unique_ptr<CameraModel> _cameraModel;
};

}