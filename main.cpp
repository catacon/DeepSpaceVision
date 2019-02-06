#include <memory>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/ansicolor_sink.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "DeepSpaceVision.h"
#include "Setup.h"

// Sigint Handler
void sigint_handler(int signal);

// Vision processor
std::unique_ptr<Lightning::DeepSpaceVision> vision;

// Logger - create sinks here so they can be shared with main and processors
std::vector<spdlog::sink_ptr> sinks {
    std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>(),
    std::make_shared<spdlog::sinks::basic_file_sink_mt>("DeepSpaceVision.log")
};

std::shared_ptr<spdlog::logger> logger;

int main(int, char**) {

    // Register Ctrl-C Handler
    signal(SIGINT, sigint_handler);

    // Setup
    Lightning::Setup::LoadSetup();

    // Logger
    logger = std::make_shared<spdlog::logger>("Main", sinks.begin(), sinks.end());
    logger->set_level(Lightning::Setup::Diagnostics::LogLevel);

    // Vision Processors
    vision = std::make_unique<Lightning::DeepSpaceVision>(sinks);
    
    logger->debug("Starting DeepSpaceVision");
    if (!vision->StartProcessing())
    {
        logger->error("Failed to start DeepSpaceVision");
    }

    while (vision->IsProcessRunning())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    logger->debug("DeepSpaceVision has stopped.");

    return 0;
}

void sigint_handler(int signal)
{
    logger->debug("sigint_handler: {0}", signal);
    vision->StopProcessing();
}
