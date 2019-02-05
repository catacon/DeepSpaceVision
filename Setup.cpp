#include <simpleini/SimpleIni.h>
#include "spdlog/spdlog.h"

#include "Setup.h"

namespace Lightning
{
namespace Setup
{
    namespace Camera
    {
        int HatchCameraId = 0;
        int CargoCameraId = 1;
        int Width = 640;
        int Height = 480;
    }

    namespace Network
    {
        int DataPort = 5801;
    }

    namespace Diagnostics
    {
        bool UseTestImage = false;
        std::string TestImagePath = "";
        bool UseTestVideo = false;
        std::string TestVideoPath = "";
        spdlog::level::level_enum LogLevel = spdlog::level::debug;
        bool DisplayDebugImages = false;
        bool RecordVideo = false;
        bool ReadSetupFile = false;
    }

    namespace Processing
    {
        int ContourSizeThreshold = 10;
        double ContourApproximationAccuracy = 2.5;
        double ShapeFactorMin = 0.4;
        double ShapeFactorMax = 0.8;
        int FastThreshold = 30;
        int CornerDistanceThreshold = 2;
        double MinAngleDiff = 90;
        double MaxAngleDiff = 180;
        double MaxTargetSeparation = 160;
        int MaxCornerSubPixelIterations = 100;
        double CornerSubPixelThreshold = 0.1;
        bool UseWorldCoordinates = false;
    }

    namespace HSVFilter
    {
        int LowH = 40;
        int LowS = 30;
        int LowV = 50;
        int HighH = 100;
        int HighS = 255;
        int HighV = 200;
        int MorphologyIterations = 2;
    }

    void SaveSetup()
    {
        CSimpleIniA ini;

            // Camera
            ini.SetLongValue("Camera", "HatchCameraId", Camera::HatchCameraId);
            ini.SetLongValue("Camera", "CargoCameraId", Camera::CargoCameraId);
            ini.SetLongValue("Camera", "Width", Camera::Width);
            ini.SetLongValue("Camera", "Height", Camera::Height);

            // Network
            ini.SetLongValue("Network", "DataPort", Network::DataPort);

            // Diagnostics
            ini.SetBoolValue("Diagnostics", "UseTestImage", Diagnostics::UseTestImage);  
            ini.SetValue("Diagnostics", "TestImagePath", Diagnostics::TestImagePath.c_str());   
            ini.SetBoolValue("Diagnostics", "UseTestVideo", Diagnostics::UseTestVideo);  
            ini.SetValue("Diagnostics", "TestVideoPath", Diagnostics::TestVideoPath.c_str());        
            ini.SetLongValue("Diagnostics", "LogLevel", (int)Diagnostics::LogLevel);
            ini.SetBoolValue("Diagnostics", "DisplayDebugImages", Diagnostics::DisplayDebugImages);     
            ini.SetBoolValue("Diagnostics", "RecordVideo", Diagnostics::RecordVideo);       
            ini.SetBoolValue("Diagnostics", "ReadSetupFile", Diagnostics::ReadSetupFile);      

            // Processing
            ini.SetLongValue("Processing", "ContourSizeThreshold", Processing::ContourSizeThreshold);
            ini.SetDoubleValue("Processing", "ContourApproximationAccuracy", Processing::ContourApproximationAccuracy);
            ini.SetDoubleValue("Processing", "ShapeFactorMin", Processing::ShapeFactorMin);
            ini.SetDoubleValue("Processing", "ShapeFactorMax", Processing::ShapeFactorMax);
            ini.SetLongValue("Processing", "FastThreshold", Processing::FastThreshold);
            ini.SetLongValue("Processing", "CornerDistanceThreshold", Processing::CornerDistanceThreshold);
            ini.SetDoubleValue("Processing", "MinAngleDiff", Processing::MinAngleDiff);
            ini.SetDoubleValue("Processing", "MaxAngleDiff", Processing::MaxAngleDiff);
            ini.SetDoubleValue("Processing", "MaxTargetSeparation", Processing::MaxTargetSeparation);
            ini.SetLongValue("Processing", "MaxCornerSubPixelIterations", Processing::MaxCornerSubPixelIterations);
            ini.SetDoubleValue("Processing", "CornerSubPixelThreshold", Processing::CornerSubPixelThreshold);
            ini.SetBoolValue("Processing", "UseWorldCoordinates", Processing::UseWorldCoordinates);  

            // HSVFilter
            ini.SetLongValue("HSVFilter", "LowH", HSVFilter::LowH);
            ini.SetLongValue("HSVFilter", "LowS", HSVFilter::LowS);
            ini.SetLongValue("HSVFilter", "LowV", HSVFilter::LowV);
            ini.SetLongValue("HSVFilter", "HighH", HSVFilter::HighH);
            ini.SetLongValue("HSVFilter", "HighS", HSVFilter::HighS);
            ini.SetLongValue("HSVFilter", "HighV", HSVFilter::HighV);
            ini.SetLongValue("HSVFilter", "MorphologyIterations", HSVFilter::MorphologyIterations);

        // TODO create directories?

        ini.SaveFile(SetupPath.c_str(), true);
    }

    void LoadSetup()
    {
        CSimpleIniA ini;

        SI_Error result = ini.LoadFile(SetupPath.c_str());

        if (result == SI_Error::SI_OK)
        {
            // Camera
            Camera::HatchCargoId = ini.GetLongValue("Camera", "HatchCameraId", Camera::HatchCameraId);
            Camera::CargoCameraId = ini.GetLongValue("Camera", "CargoCameraId", Camera::CargoCameraId);
            Camera::CameraId = ini.GetLongValue("Camera", "CameraId", Camera::CameraId);
            Camera::Width = ini.GetLongValue("Camera", "Width", Camera::Width);
            Camera::Height = ini.GetLongValue("Camera", "Height", Camera::Height);

            // Network
            Network::DataPort = ini.GetLongValue("Network", "DataPort", Network::DataPort);

            // Diagnostics
            Diagnostics::UseTestImage = ini.GetBoolValue("Diagnostics", "UseTestImage", Diagnostics::UseTestImage);            
            Diagnostics::TestImagePath = ini.GetValue("Diagnostics", "TestImagePath", Diagnostics::TestImagePath.c_str());
            Diagnostics::UseTestImage = ini.GetBoolValue("Diagnostics", "UseTestVideo", Diagnostics::UseTestVideo);            
            Diagnostics::TestImagePath = ini.GetValue("Diagnostics", "TestVideoPath", Diagnostics::TestVideoPath.c_str());
            Diagnostics::LogLevel = (spdlog::level::level_enum)ini.GetLongValue("Diagnostics", "LogLevel", Diagnostics::LogLevel);
            Diagnostics::DisplayDebugImages = ini.GetBoolValue("Diagnostics", "DisplayDebugImages", Diagnostics::DisplayDebugImages);     
            Diagnostics::RecordVideo = ini.GetBoolValue("Diagnostics", "RecordVideo", Diagnostics::RecordVideo);       
            Diagnostics::ReadSetupFile = ini.GetBoolValue("Diagnostics", "ReadSetupFile", Diagnostics::ReadSetupFile);      

            // Processing
            Processing::ContourSizeThreshold = ini.GetLongValue("Processing", "ContourSizeThreshold", Processing::ContourSizeThreshold);
            Processing::ContourApproximationAccuracy = ini.GetDoubleValue("Processing", "ContourApproximationAccuracy", Processing::ContourApproximationAccuracy);
            Processing::ShapeFactorMin = ini.GetDoubleValue("Processing", "ShapeFactorMin", Processing::ShapeFactorMin);
            Processing::ShapeFactorMax = ini.GetDoubleValue("Processing", "ShapeFactorMax", Processing::ShapeFactorMax);
            Processing::FastThreshold = ini.GetLongValue("Processing", "FastThreshold", Processing::FastThreshold);
            Processing::CornerDistanceThreshold = ini.GetLongValue("Processing", "CornerDistanceThreshold", Processing::CornerDistanceThreshold);
            Processing::MinAngleDiff = ini.GetDoubleValue("Processing", "MinAngleDiff", Processing::MinAngleDiff);
            Processing::MaxAngleDiff = ini.GetDoubleValue("Processing", "MaxAngleDiff", Processing::MaxAngleDiff);
            Processing::MaxTargetSeparation = ini.GetDoubleValue("Processing", "MaxTargetSeparation", Processing::MaxTargetSeparation);
            Processing::MaxCornerSubPixelIterations = ini.GetLongValue("Processing", "MaxCornerSubPixelIterations", Processing::MaxCornerSubPixelIterations);
            Processing::CornerSubPixelThreshold = ini.GetDoubleValue("Processing", "CornerSubPixelThreshold", Processing::CornerSubPixelThreshold);
            Processing::UseWorldCoordinates = ini.GetBoolValue("Processing", "UseWorldCoordinates", Processing::UseWorldCoordinates);

            // HSVFilter
            HSVFilter::LowH = ini.GetLongValue("HSVFilter", "LowH", HSVFilter::LowH);
            HSVFilter::LowS = ini.GetLongValue("HSVFilter", "LowS", HSVFilter::LowS);
            HSVFilter::LowV = ini.GetLongValue("HSVFilter", "LowV", HSVFilter::LowV);
            HSVFilter::HighH = ini.GetLongValue("HSVFilter", "HighH", HSVFilter::HighH);
            HSVFilter::HighS = ini.GetLongValue("HSVFilter", "HighS", HSVFilter::HighS);
            HSVFilter::HighV = ini.GetLongValue("HSVFilter", "HighV", HSVFilter::HighV);
            HSVFilter::MorphologyIterations = ini.GetLongValue("HSVFilter", "MorphologyIterations", HSVFilter::MorphologyIterations);
        }
        else
        {
            // Save a default file if the file does not exist
            SaveSetup();
        }
    }
}
}