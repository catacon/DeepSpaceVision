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

    DataSender::DataSender(std::shared_ptr<Setup> setup, std::shared_ptr<spdlog::logger> logger, int port)

    bool Send();

private:
    std::shared_ptr<Setup> _setup;
    std::shared_ptr<spdlog::logger> _logger;

    zmq::context_t _context;
    zmq::socket_t _socket;

    int _port;
    
};

}