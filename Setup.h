#pragma once

#include <string>

#include "spdlog/spdlog.h"

namespace Lightning
{

namespace Setup
{
    const static std::string LogPath = "DeepSpaceVision.log";
    const static std::string SetupPath = "setup.ini";

    void LoadSetup();
    void SaveSetup();

    namespace Camera
    {
        // Default camera id for hatch side
        extern int HatchCameraId;

        // Default camera id for cargo side
        extern int CargoCameraId;

        // Image width
        extern int Width;

        // Image height
        extern int Height;
    }

    namespace Network
    {
        // Default port for sending data to robot
        extern int DataPort;
    }

    namespace Diagnostics
    {
        // Use saved image instead of camera
        extern bool UseTestImage;

        // Path to test image
        extern std::string TestImagePath;

        // Use saved video instead of camera
        extern bool UseTestVideo;

        // Path to test video
        extern std::string TestVideoPath;

        // Log Level
        extern spdlog::level::level_enum LogLevel;

        // Display development images
        extern bool DisplayDebugImages;

        // Record and save raw video for diagnostics
        extern bool RecordVideo;

        // Record and save processed video
        extern bool RecordProcessedVideo;

        // Folder in which to save videos
        extern std::string RecordVideoPath;

        // Read setup file during execution to adjust program behavior in real time
        extern bool ReadSetupFile;

        // Wait time for displaying images - 0 will pause the display until a key is pressed
        extern int WaitKeyDelay;
    }

    namespace Processing
    {
        // Minimum size of contours
        extern int ContourSizeThreshold;

        // Accuracy setting for approxPolyDP function
        extern double ContourApproximationAccuracy;

        // Minimum threshold for shape factor filter
        extern double ShapeFactorMin;

        // Maximum threshold for shape factor filter
        extern double ShapeFactorMax;

        // Threshold for FAST corner detector
        extern int FastThreshold;

        // Distance threshold for close corner points
        extern int CornerDistanceThreshold;

        // Minimum angle difference between target sections in degrees
        extern double MinAngleDiff;

        // Maximum angle difference between target sections in degrees
        extern double MaxAngleDiff;

        // Maximum separation of target sections - larger number allows larger target separation
        extern double TargetSeparationThreshold;

        // Number of iterations for corner subpixel calculation
        extern int MaxCornerSubPixelIterations;

        // Threshold for corner subpixel calculation
        extern double CornerSubPixelThreshold;

        // Switch for world coordinates or camera coordinates in final answer
        extern bool UseWorldCoordinates;

        // X Offset for hatch camera in millimeters
        extern double HatchOffset;

        // X Offset for cargo camera in millimeters
        extern double CargoOffset;

        // Distance from edge of image before contour is rejected
        extern double ImageEdgeThreshold;

        // Flag to allow processing of half targets
        extern bool ProcessHalfTargets;
    }
    
    namespace HSVFilter
    {
        // Low Hue limit
        extern int LowH;

        // Low Saturation limit
        extern int LowS;

        // Low Value limit
        extern int LowV;

        // High Hue limit
        extern int HighH;

        // High Saturation limit
        extern int HighS;

        // High Value limit
        extern int HighV;

        // Number of morphology iterations
        extern int MorphologyIterations;
    }
}

}