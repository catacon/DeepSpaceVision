#include <thread>

#include "DeepSpaceProcessor.h"
#include "DeepSpaceTargetModel.h"
#include "PS3Eye.h"
#include "Setup.h"

using namespace Lightning;

DeepSpaceProcessor::DeepSpaceProcessor(std::vector<spdlog::sink_ptr> sinks, std::string name, std::shared_ptr<cv::VideoCapture> capture, cv::Vec3d offset)
    : _capture(capture)
    , _targetFinder(std::make_unique<TargetFinder>(sinks, name, std::make_unique<DeepSpaceTargetModel>(), std::make_unique<PS3EyeModel>(), offset))
    , _name(name)
{
    _logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
    _logger->set_level(Lightning::Setup::Diagnostics::LogLevel);

    if (Setup::Diagnostics::RecordVideo)
    {
        std::string path = fmt::format("{0}{1}_raw.avi", Setup::Diagnostics::RecordVideoPath, _name);

        _rawVideoWriter = std::make_unique<cv::VideoWriter>();

        _rawVideoWriter->open(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, 
            cv::Size(Setup::Camera::Width, Setup::Camera::Height),
            true);
    }
}

bool DeepSpaceProcessor::ProcessNextImage(std::vector<VisionData>& targetData)
{
    if (_capture->isOpened())
    {
        // Read next frame from source
        cv::Mat image;
        _capture->read(image);

        if (Setup::Diagnostics::RecordVideo && _rawVideoWriter)
        {
            _rawVideoWriter->write(image);
            
        }

        if (!image.empty())
        {
            _targetFinder->Process(image, targetData);

            if (Setup::Diagnostics::RecordProcessedVideo && _processedVideoWriter)
            {
                
            }

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