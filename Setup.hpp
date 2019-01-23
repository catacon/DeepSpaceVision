#pragma once

#include <simpleini/SimpleIni.h>
#include "spdlog/spdlog.h"

namespace Lightning
{

class Setup 
{
    public:
        Setup()
        {
            _file = _KDefaultSetupFile;
        }

        Setup(std::string file)
            : _file(file)
        {

        }

        void Load()
        {
            _ini.LoadFile(_file.c_str());

            // Camera
            CameraId = _ini.GetLongValue("Camera", "CameraId", CameraId);

            // Network
            DataPort = _ini.GetLongValue("Network", "DataPort", DataPort);

            // Diagnostics
            UseTestImage = _ini.GetBoolValue("Diagnostics", "UseTestImage", UseTestImage);
            TestImagePath = _ini.GetValue("Diagnostics", "TestImagePath", "/");
            LogLevel = _ini.GetLongValue("Diagnostics", "LogLevel", LogLevel);
            DebugImages = _ini.GetBoolValue("Diagnostics", "DebugImages", DebugImages);
            RecordVideo = _ini.GetBoolValue("Diagnostics", "RecordVideo", RecordVideo);
        }

        /* 
            Camera
        */

        // ID of camera to use for targetting
        long CameraId = 0;

        /*
            Network
        */
       long DataPort = 5801;

        /*
            Diagnostics
        */

        // Flag to read test image from disk
        bool UseTestImage = false;

        // Path to test image
        std::string TestImagePath = "";

        // Log Level
        long LogLevel = (long)spdlog::level::debug;

        // Display development images
        bool DebugImages = false;

        // Record diagnostic video
        bool RecordVideo = false;

    private:
        CSimpleIni _ini;
        std::string _file;
        std::string _KDefaultSetupFile = "setup.ini";
};

}