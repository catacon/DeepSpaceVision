#pragma once

#include <memory>

#include "spdlog/spdlog.h"

#include "VisionData.hpp"
#include "TargetFinder.h"

namespace cv
{
class VideoCapture;
}

namespace Lightning
{

class DeepSpaceProcessor
{
public:
    DeepSpaceProcessor(std::vector<spdlog::sink_ptr>, std::string, std::shared_ptr<cv::VideoCapture>);

    bool ProcessNextImage(std::vector<VisionData>&);

    void ShowDebugImages();

private:
    std::shared_ptr<spdlog::logger> _logger;

    std::shared_ptr<cv::VideoCapture> _capture;
    std::unique_ptr<TargetFinder> _targetFinder;
};

}