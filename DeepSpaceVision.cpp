#include <thread>

#include "DeepSpaceVision.h"
#include "VisionData.hpp"
#include "DeepSpaceTargetModel.h"
#include "CameraModel.h"
#include "Setup.h"

using namespace Lightning;

DeepSpaceVision::DeepSpaceVision(std::shared_ptr<spdlog::logger> logger, std::shared_ptr<cv::VideoCapture> capture)
    : _logger(logger)
    , _targetCapture(capture)
    , _targetFinder(std::make_unique<TargetFinder>(logger, DeepSpaceTargetModel(), PS3EyeModel()))
    , _dataSender(std::make_unique<DataSender>(logger))
{
    _runProcessing.store(false);
    _isProcessorRunning.store(false);

    if (Setup::Diagnostics::RecordVideo)
    {
        InitializeVideoWriters();
    }
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
    bool escapeKeyPressed = false;

    while (_runProcessing && _targetCapture->isOpened() && !escapeKeyPressed)
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

            if (Setup::Diagnostics::RecordVideo && !Setup::Diagnostics::UseTestVideo)
            {
                if (!_rawVideoWriter)
                {
                    InitializeVideoWriters();
                }

                _rawVideoWriter->write(image);
            }
        }

        if (!image.empty())
        {
            std::vector<VisionData> data;

            if (_targetFinder->Process(image, data))
            {
                if (!_dataSender->Send(data))
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
                auto key = _targetFinder->ShowDebugImages();

                // Quit if the escape key was pressed on an image - makes exiting the program a bit easier while debugging
                if (key == 27)
                {
                    escapeKeyPressed = true;
                }
            }

            if (Setup::Diagnostics::RecordVideo && !Setup::Diagnostics::UseTestImage)
            {
                if (!_processedVideoWriter)
                {
                    InitializeVideoWriters();
                }

                _processedVideoWriter->write(image);
            }
        }
        else
        {
            _logger->error("Failed to read from {0}", Setup::Diagnostics::UseTestImage ? "File" : "Target Capture");
        }
    }

    _isProcessorRunning = false;
}

void DeepSpaceVision::InitializeVideoWriters()
{
    std::string rawFileName;
    std::string processedFileName;
    _rawVideoWriter = std::make_unique<cv::VideoWriter>(rawFileName, cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(Setup::Camera::Width, Setup::Camera::Height));
    _processedVideoWriter = std::make_unique<cv::VideoWriter>(rawFileName, cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(Setup::Camera::Width, Setup::Camera::Height));
}