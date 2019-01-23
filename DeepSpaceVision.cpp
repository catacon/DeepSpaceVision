#include <thread>

#include "DeepSpaceVision.h"
#include "VisionData.hpp"
#include "DeepSpaceTargetModel.h"
#include "CameraModel.h"

using namespace Lightning;

DeepSpaceVision::DeepSpaceVision(std::shared_ptr<Setup> setup, std::shared_ptr<spdlog::logger> logger)
    : _setup(setup)
    , _logger(logger)
    , _targetCapture(std::make_unique<cv::VideoCapture>(setup->CameraId))
    , _targetFinder(std::make_unique<TargetFinder>(setup, logger, DeepSpaceTargetModel(), CameraModel()))
    , _dataSender(std::make_unique<DataSender>(setup, logger))
{
    _runProcessing.store(false);
    _isProcessorRunning.store(false);
}

bool DeepSpaceVision::StartProcessing()
{
    if (_isProcessorRunning)
    {
        return false;
    }

    _runProcessing = true;

    std::thread t(&DeepSpaceVision::Process, this);

    if (t.joinable())
    {
        t.detach();
        return true;
    }

    return false;
}

bool DeepSpaceVision::StopProcessing()
{
    _runProcessing = false;
}

void DeepSpaceVision::Process()
{
    _isProcessorRunning = true;

    while (_runProcessing)
    {
        cv::Mat image;

        if (_setup->UseTestImage)
        {
            image = cv::imread(_setup->TestImagePath);
        }
        else
        {
            _targetCapture->read(image);
        }

        bool isImageOk = true;

        if (image.empty() || image.rows <= 0 || image.cols <= 0)
        {
            isImageOk = false;
        }

        if (isImageOk)
        {
            VisionData data;

            if (_targetFinder->Process(image, data))
            {
                if (_dataSender->Send(data))
                {

                }
                else
                {
                    _logger->error("Failed to send data");   // TODO add error code
                }
            }
            else
            {
                _logger->error("Failed to process image");   // TODO add error code
            }

            if (_setup->DebugImages)
            {
                _targetFinder->ShowDebugImages();
            }
        }
        else
        {
            _logger->error("Failed to read from {0}", _setup->UseTestImage ? "File" : "Target Capture");
        }
    }

    _isProcessorRunning = false;
}