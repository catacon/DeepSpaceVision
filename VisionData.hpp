#pragma once

#include <json.hpp>

#include "VisionStatus.hpp"

namespace Lightning
{

class VisionData
{
public:

/*
    VisionData()
        : status(VisionStatus::NotRunning)
        , x(0.0)
        , y(0.0)
        , z(0.0)
        , roll(0.0)
        , pitch(0.0)
        , yaw(0.0)
    {

    }

    VisionData(VisionStatus _status, double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
        : status(_status)
        , x(_x)
        , y(_y)
        , z(_z)
        , roll(_roll)
        , pitch(_pitch)
        , yaw(_yaw)
    {

    }
    */

    VisionStatus status;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

    inline void to_json(nlohmann::json& j, const VisionData& d) {
        j = nlohmann::json{{"status", (int)d.status}, {"x", d.x}, {"y", d.y}, {"z", d.z}, {"roll", d.roll}, {"pitch", d.pitch}, {"yaw", d.yaw}};
    }

    inline void from_json(const nlohmann::json& j, VisionData& d) {
        j.at("status").get_to(d.status);
        j.at("x").get_to(d.x);
        j.at("y").get_to(d.y);
        j.at("z").get_to(d.z);
        j.at("roll").get_to(d.roll);
        j.at("pitch").get_to(d.pitch);
        j.at("yaw").get_to(d.yaw);
    }

}