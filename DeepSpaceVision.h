#pragma once

#include <memory>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

#include "Setup.hpp"
#include "TargetFinder.h"
#include "DataSender.h"

namespace Lightning
{

class DeepSpaceVision
{
public:
    DeepSpaceVision(std::shared_ptr<Setup>, std::shared_ptr<spdlog::logger>);

    bool StartProcessing();

    bool StopProcessing();

    bool IsRunning() { return _isProcessorRunning.load(); }

private:

    void Process();

    std::shared_ptr<Setup> _setup;
    std::shared_ptr<spdlog::logger> _logger;

    std::unique_ptr<cv::VideoCapture> _targetCapture;
    std::unique_ptr<TargetFinder> _targetFinder;
    std::unique_ptr<DataSender> _dataSender;    

    std::atomic<bool> _runProcessing;

    std::atomic<bool> _isProcessorRunning;
};

}