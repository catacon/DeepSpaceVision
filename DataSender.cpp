#include <json.hpp>

#include "DataSender.h"
#include "VisionData.hpp"
#include "Setup.h"

using namespace Lightning;

DataSender::DataSender(std::shared_ptr<spdlog::logger> logger)
    : _logger(logger)
    , _context(1)
    , _socket(_context, ZMQ_PUB)
{
    std::string s = std::string("tcp://*:" + std::to_string(Setup::Network::DataPort));
    //TODO
    //_socket.bind("tcp://*:5556");
}

DataSender::~DataSender()
{
    _socket.close();
    zmq_ctx_destroy(&_context);
}

bool DataSender::Send(const VisionData& d)
{
    nlohmann::json j = d;
    std::string s = j.dump();

    zmq::message_t message(s.length());
    std::memcpy(message.data(), s.c_str(), s.length());


// TODO
    //_socket.send(message);

    return true;
}