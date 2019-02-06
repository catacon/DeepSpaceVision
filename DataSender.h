#pragma once

#include <memory>
#include <zmq.hpp>

#include "VisionData.hpp"

namespace Lightning
{

class DataSender
{

public: 

    DataSender();
    ~DataSender();

    bool Send(const std::vector<VisionMessage>&);

private:
    zmq::context_t _context;
    zmq::socket_t _socket;
    
};

}