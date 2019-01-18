#include "TargetFinder.h"
#include "CameraModel.h"
#include "TargetModel.h"

using namespace Lightning;

    TargetFinder::TargetFinder(std::shared_ptr<Setup> setup, std::shared_ptr<spdlog::logger> logger, TargetModel targetModel, CameraModel cameraModel)
        : _setup(setup)
        , _logger(logger)
        , _targetModel(targetModel)
        , _cameraModel(cameraModel)
    {

    }

bool TargetFinder::Process(cv::Mat& image, VisionData& data)
{
    // Convert image to HSV

    // Filter based on color

    // Close disconnected contours

    // Detect contours

    // Filter out small contours


    // Approximate contours

    // Draw contours

    // Detect corners

    // Convert keypoints to regular points

    // Combine close points

    // Sort points into corners

    // Find subpixels

    // Calculate target center

    // Sort corners by relationship to center



    // Set image points

    // Find transform

    // Convert rotation

    // ...

    // Convert to target coordinates


    // Debugging images?


    return true;
}