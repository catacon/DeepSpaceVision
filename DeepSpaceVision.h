#pragma once

#include "DeepSpaceProcessor.h"
#include "DataSender.h"

namespace cv
{

class VideoCapture;

}

namespace Lightning
{

class DeepSpaceVision
{
public:

    DeepSpaceVision(std::vector<spdlog::sink_ptr>);

    bool IsProcessRunning() { return _isProcessorRunning; };

    bool StartProcessing();
    void StopProcessing();

private:

    void Process();

    std::unique_ptr<DeepSpaceProcessor> _hatchProcessor;
    std::unique_ptr<DeepSpaceProcessor> _cargoProcessor;

    std::shared_ptr<cv::VideoCapture> _hatchCapture;
    std::shared_ptr<cv::VideoCapture> _cargoCapture;

    std::unique_ptr<DataSender> _dataSender;

    std::shared_ptr<spdlog::logger> _logger;

    std::atomic<bool> _doProcessing;
    std::atomic<bool> _isProcessorRunning;

};

}