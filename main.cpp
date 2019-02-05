#include <memory>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/ansicolor_sink.h"
#include "spdlog/sinks/basic_file_sink.h"

#include "DeepSpaceVision.h"
#include "Setup.h"

// Sigint Handler
std::atomic<bool> run(true);
void sigint_handler(int signal);

// Logger
std::vector<spdlog::sink_ptr> sinks {
    std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>(),
    std::make_shared<spdlog::sinks::basic_file_sink_mt>("DeepSpaceVision.log")
};

std::shared_ptr<spdlog::logger> logger;

// Vision Processors
std::unique_ptr<Lightning::DeepSpaceVision> _hatchSideProcessor;
std::unique_ptr<Lightning::DeepSpaceVision> _cargoSideProcessor;

int main(int, char**) {

    // Register Ctrl-C Handler
    signal(SIGINT, sigint_handler);

    // Setup
    Lightning::Setup::LoadSetup();

    // Logger
    logger = std::make_shared<spdlog::logger>("DeepSpaceVision", sinks.begin(), sinks.end());
    logger->set_level(Lightning::Setup::Diagnostics::LogLevel);
    logger->debug("Starting DeepSpaceVision");

    // Vision Processors
    _hatchSideProcessor = std::make_unique<Lightning::DeepSpaceVision>(logger);
    _cargoSideProcessor = std::make_unique<Lightning::DeepSpaceVision>(logger);

    if (!_hatchSideProcessor->StartProcessing())
    {
        logger->error("Failed to start hatch vision processing");
    }

    if (!_cargoSideProcessor->StartProcessing())
    {
        logger->error("Failed to start cargo vision processing");
    }

    while (_hatchSideProcessor->IsRunning() || _cargoSideProcessor->IsRunning())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    logger->debug("Stopping DeepSpaceVision");

    return 0;
}

void sigint_handler(int signal)
{
    logger->debug("sigint_handler: {0}", signal);
    _visionProcessor->StopProcessing();
}
