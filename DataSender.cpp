#include <json.hpp>

#include "DataSender.h"
#include "VisionData.hpp"
#include "Setup.hpp"

using namespace Lightning;

DataSender::DataSender(std::shared_ptr<Setup> setup, std::shared_ptr<spdlog::logger> logger)
    : _setup(setup)
    , _logger(logger)
    , _context(1)
    , _socket(_context, ZMQ_PUB)
{
    _socket.bind("tcp://*:" + std::to_string(setup->DataPort));
}

bool DataSender::Send(const VisionData& d)
{
    nlohmann::json j = d;
    std::string s = j.dump();

    zmq::message_t message;
    std::memcpy(message.data(), s.c_str(), s.length());

    _socket.send(message);

    return true;
}