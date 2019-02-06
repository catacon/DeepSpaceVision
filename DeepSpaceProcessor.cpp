#include <thread>

#include "DeepSpaceProcessor.h"
#include "DeepSpaceTargetModel.h"
#include "PS3Eye.h"
#include "Setup.h"

using namespace Lightning;

DeepSpaceProcessor::DeepSpaceProcessor(std::vector<spdlog::sink_ptr> sinks, std::string name, std::shared_ptr<cv::VideoCapture> capture)
    : _capture(capture)
    , _targetFinder(std::make_unique<TargetFinder>(sinks, name, DeepSpaceTargetModel(), PS3EyeModel()))
{
    _logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    _logger->set_level(Lightning::Setup::Diagnostics::LogLevel);
}

bool DeepSpaceProcessor::ProcessNextImage(std::vector<VisionData>& targetData)
{
    if (_capture->isOpened())
    {
        // Read next frame from source
        cv::Mat image;
        _capture->read(image);

        // TODO record video

        if (!image.empty())
        {
            _targetFinder->Process(image, targetData);

            // TODO record processed video

            return true;
        }
        else
        {
            _logger->error("Invalid image - shutting down capture");
            _capture->release();
            return false;
        }
    }
}

void DeepSpaceProcessor::ShowDebugImages()
{
    _targetFinder->ShowDebugImages();
}