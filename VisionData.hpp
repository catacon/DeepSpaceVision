#pragma once

#include <json.hpp>

#include "VisionStatus.hpp"

namespace Lightning
{

class VisionData
{
public:

   // TODO define id better

    VisionStatus status;
    int targetId;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double imageX;
    double imageY;
};

inline void to_json(nlohmann::json& j, const VisionData& d) {
    j = nlohmann::json{{"status", (int)d.status}, {"x", d.x}, {"y", d.y}, {"z", d.z}, {"roll", d.roll}, {"pitch", d.pitch}, {"yaw", d.yaw}, {"imageX", d.imageX}, {"imageY", d.imageY}};
}

inline void from_json(const nlohmann::json& j, VisionData& d) {
    j.at("status").get_to(d.status);
    j.at("x").get_to(d.x);
    j.at("y").get_to(d.y);
    j.at("z").get_to(d.z);
    j.at("roll").get_to(d.roll);
    j.at("pitch").get_to(d.pitch);
    j.at("yaw").get_to(d.yaw);
    j.at("imageX").get_to(d.imageX);
    j.at("imageY").get_to(d.imageY);
}

class VisionMessage
{
public:
    int cameraId;
    std::vector<VisionData> packets;

};

inline void to_json(nlohmann::json& j, const VisionMessage& d) {
    j = nlohmann::json{{"cameraId", (int)d.cameraId}, {"packets", d.packets}};
}

inline void from_json(const nlohmann::json& j, VisionMessage& d) {
    j.at("cameraId").get_to(d.cameraId);
    j.at("packets").get_to(d.packets);
}

}