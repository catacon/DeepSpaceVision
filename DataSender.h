#pragma once

#include <memory>

#include "spdlog/spdlog.h"
#include <zmq.hpp>
#include <json.hpp>

#include "Setup.hpp"

namespace Lightning
{

class DataSender
{

public: 
    DataSender(std::shared_ptr<Setup> setup, std::shared_ptr<spdlog::logger> logger)
        : _setup(setup)
        , _logger(logger)
    {

    }

    bool Send();

private:
    std::shared_ptr<Setup> _setup;
    std::shared_ptr<spdlog::logger> _logger;

    zmq::context_t _context;
    
};

}