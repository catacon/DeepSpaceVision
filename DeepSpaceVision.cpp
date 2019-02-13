#include <opencv2/opencv.hpp>

#include "DeepSpaceVision.h"
#include "Setup.h"
#include "VisionData.hpp"
#include "DataSender.h"

using namespace Lightning;

DeepSpaceVision::DeepSpaceVision(std::vector<spdlog::sink_ptr> sinks)
    : _isProcessorRunning(false)
    , _doProcessing(false)
{
    _logger = std::make_shared<spdlog::logger>("DeepSpaceVision", sinks.begin(), sinks.end());
    _logger->set_level(Lightning::Setup::Diagnostics::LogLevel);

    // Setup hatch capture
    if (Setup::Camera::HatchCameraId >= 0)
    {    
        if (Setup::Diagnostics::UseTestVideo)
        {
            _hatchCapture = std::make_shared<cv::VideoCapture>(Setup::Diagnostics::TestVideoPath);
            _logger->info("Hatch capture set to video: {0}", Setup::Diagnostics::TestVideoPath);
        }
        else if (Setup::Diagnostics::UseTestImage)
        {
            _hatchCapture = std::make_shared<cv::VideoCapture>(Setup::Diagnostics::TestImagePath);
            _logger->info("Hatch capture set to image(s): {0}", Setup::Diagnostics::TestImagePath);
        }
        else
        {
            _hatchCapture = std::make_shared<cv::VideoCapture>(Setup::Camera::HatchCameraId);
            _logger->info("Hatch capture set to camera ID: {0}", Setup::Camera::HatchCameraId);
        }
        
    }
    else
    {
        _logger->info("Hatch capture will not be used");
    }
    

    // Setup cargo capture
    if (Setup::Camera::CargoCameraId >= 0)
    {    
        if (Setup::Diagnostics::UseTestVideo)
        {
            _cargoCapture = std::make_shared<cv::VideoCapture>(Setup::Diagnostics::TestVideoPath);
            _logger->info("Cargo capture set to video: {0}", Setup::Diagnostics::TestVideoPath);

        }
        else if (Setup::Diagnostics::UseTestImage)
        {
            _cargoCapture = std::make_shared<cv::VideoCapture>(Setup::Diagnostics::TestImagePath);
            _logger->info("Cargo capture set to image(s): {0}", Setup::Diagnostics::TestImagePath);
        }
        else
        {
            _cargoCapture = std::make_shared<cv::VideoCapture>(Setup::Camera::CargoCameraId);
            _logger->info("Cargo capture set to camera: {0}", Setup::Camera::CargoCameraId);
        }
        
    }
    else
    {
        _logger->info("Cargo capture will not be used");

    }

    // Setup hatch processor
    if (_hatchCapture)
    {
        if (_hatchCapture->isOpened())
        {
            _hatchProcessor = std::make_unique<DeepSpaceProcessor>(sinks, "Hatch", _hatchCapture);  
        }
        else
        {           
            _logger->error("Failed to open hatch capture.");          
        }
    }
    else
    {
        _logger->info("Hatch processor will not be used");
    }

    // Setup cargo processor
    if (_cargoCapture)
    {
        if (_cargoCapture->isOpened())
        {
            _cargoProcessor = std::make_unique<DeepSpaceProcessor>(sinks, "Cargo", _cargoCapture);
        }
        else
        {
            _logger->error("Failed to open cargo capture.");                      
        }
        
    }
    else
    {
        _logger->info("Cargo processor will not be used");
    }    

    _dataSender = std::make_unique<DataSender>();
}

bool DeepSpaceVision::StartProcessing()
{
    if (_isProcessorRunning)
    {
        _logger->info("DeepSpaceVision is already running.");
        return false;
    }

    _doProcessing = true;

    std::thread t(&DeepSpaceVision::Process, this);

    if (t.joinable())
    {
        _isProcessorRunning = true;
        t.detach();
        return true;
    }

    return false;
}

void DeepSpaceVision::StopProcessing()
{
    _doProcessing = false;
}

void DeepSpaceVision::Process()
{
    _logger->debug("Enter Process thread");

    while (_doProcessing)
    {
        if (Setup::Diagnostics::ReadSetupFile)
        {
            Setup::LoadSetup();
        }

        std::vector<VisionData> hatchData;

        if (_hatchProcessor)
        {
            _hatchProcessor->ProcessNextImage(hatchData);
            // TODO check return? - shutdown after so any failed attempts?
        }

        std::vector<VisionData> cargoData;

        if (_cargoProcessor)
        {
            _cargoProcessor->ProcessNextImage(cargoData);
            // TODO check return? - shutdown after so any failed attempts?
        }

        // Pack results
        std::vector<VisionMessage> messages
        {
            VisionMessage { 0, hatchData },
            VisionMessage { Setup::Camera::CargoCameraId, cargoData }
        };

        // Send results
        _dataSender->Send(messages);

        // Display diagnostic images, if desired
        if (Setup::Diagnostics::DisplayDebugImages)
        {
            if (_hatchProcessor)
            {
                _hatchProcessor->ShowDebugImages();
            }

            if (_cargoProcessor)
            {
                _cargoProcessor->ShowDebugImages();
            }

            int key = cv::waitKey(Setup::Diagnostics::WaitKeyDelay);

            if (key == 27)
            {
                _logger->info("Escape key pressed. Shutting down.");
                _doProcessing = false;
            }
        }
    }

    _isProcessorRunning = false;

    _logger->debug("Leaving Process thread");
}