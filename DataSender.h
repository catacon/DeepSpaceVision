#pragma once

#include <memory>

#include "spdlog/spdlog.h"
#include <zmq.hpp>

namespace Lightning
{

class VisionData;

class DataSender
{

public: 

    DataSender(std::shared_ptr<spdlog::logger>);

    ~DataSender();

    bool Send(const std::vector<VisionData>&);

private:
    std::shared_ptr<spdlog::logger> _logger;

    zmq::context_t _context;
    zmq::socket_t _socket;
    
};

}