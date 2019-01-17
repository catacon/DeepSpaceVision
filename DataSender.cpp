#include "DataSender.h"

using namespace Lightning;

bool DataSender::Send()
{
    nlohmann::json j;

    j["test"] = false;
    return true;
}