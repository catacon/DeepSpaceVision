#include <memory>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

#include "DeepSpaceVision.h"

// Sigint Handler
std::atomic<bool> run(true);
void sigint_handler(int signal);

// Setup
std::shared_ptr<Lightning::Setup> setup;

// Logger
std::vector<spdlog::sink_ptr> sinks {
    std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>(),
    std::make_shared<spdlog::sinks::simple_file_sink_mt>("DeepSpaceVision.log")
};

std::shared_ptr<spdlog::logger> logger;

// Vision Processor
std::unique_ptr<Lightning::DeepSpaceVision> _visionProcessor;

int main(int, char**) {

    // Register Ctrl-C Handler
    signal(SIGINT, sigint_handler);

    // Setup
    setup = std::make_shared<Lightning::Setup>();
    setup->Load();

    // Logger
    logger = std::make_shared<spdlog::logger>("DeepSpaceVision", sinks.begin(), sinks.end());
    logger->set_level((spdlog::level::level_enum)setup->LogLevel);

    logger->debug("Starting DeepSpaceVision");

    // Vision Processor
    _visionProcessor = std::make_unique<Lightning::DeepSpaceVision>(setup, logger);

    if (!_visionProcessor->StartProcessing())
    {
        logger->error("Failed to start vision processing"); // TODO error code
    }

    while (_visionProcessor->IsRunning())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}

void sigint_handler(int signal)
{
    logger->debug("sigint_handler: {0}", signal);
    _visionProcessor->StopProcessing();
}
