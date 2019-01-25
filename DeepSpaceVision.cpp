#include <thread>

#include "DeepSpaceVision.h"
#include "VisionData.hpp"
#include "DeepSpaceTargetModel.h"
#include "CameraModel.h"
#include "Setup.h"

using namespace Lightning;

DeepSpaceVision::DeepSpaceVision(std::shared_ptr<spdlog::logger> logger)
    : _logger(logger)
    , _targetCapture(std::make_unique<cv::VideoCapture>(Setup::Camera::CameraId))
    , _targetFinder(std::make_unique<TargetFinder>(logger, DeepSpaceTargetModel(), CameraModel()))
    , _dataSender(std::make_unique<DataSender>(logger))
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

    while (_runProcessing && _targetCapture->isOpened())
    {
        // If desired, reread setup file to update values
        if (Setup::Diagnostics::ReadSetupFile)
        {
            Setup::LoadSetup();
        }

        cv::Mat image;

        if (Setup::Diagnostics::UseTestImage)
        {
            image = cv::imread(Setup::Diagnostics::TestImagePath);
        }
        else
        {
            _targetCapture->read(image);
        }

        if (!image.empty())
        {
            std::vector<VisionData> data;

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

            if (Setup::Diagnostics::DisplayDebugImages)
            {
                _targetFinder->ShowDebugImages();
            }
        }
        else
        {
            _logger->error("Failed to read from {0}", Setup::Diagnostics::UseTestImage ? "File" : "Target Capture");
        }
    }

    _isProcessorRunning = false;
}