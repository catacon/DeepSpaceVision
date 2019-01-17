#pragma once

#include <opencv2/opencv.hpp>

#include "TargetModel.h"

namespace Lightning
{
class DeepSpaceTargetModel : TargetModel
{
public:

    DeepSpaceTargetModel() : TargetModel()
    {
        keyPoints = std::vector<cv::Point3d> {
            {0, 0, 0},
            {2, 0, 0},
            {18, 0, 0},
            {20, 0, 0},
            {2, 12, 0},
            {18, 12, 0},
            {0, 14, 0},
            {20, 14, 0}
        };

        targetAxes = std::vector<cv::Point3d> {
            {10,0,0},
            {0,10,0},
            {0,0,10}
        };
    }
};
}