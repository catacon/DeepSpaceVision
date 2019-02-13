#pragma once

#include <opencv2/opencv.hpp>

#include "TargetModel.h"

namespace Lightning
{
class DeepSpaceTargetModel : public TargetModel
{
public:

    DeepSpaceTargetModel();

    virtual std::vector<cv::Point3d> GetSubTargetKeyPoints(int) const;
};
}