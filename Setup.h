#pragma once

#include <string>

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
        // Default camera id
        extern int CameraId;
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

        // Record and save diagnostic video
        extern bool RecordVideo;

        // Read setup file during execution to adjust program behavior in real time
        extern bool ReadSetupFile;
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

        // Maximum separation of target sections in pixels
        extern double MaxTargetSeparation;

        // Number of iterations for corner subpixel calculation
        extern int MaxCornerSubPixelIterations;

        // Threshold for corner subpixel calculation
        extern double CornerSubPixelThreshold;
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