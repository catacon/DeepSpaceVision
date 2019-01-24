#pragma once

#include <opencv2/opencv.hpp>

#include "TargetModel.h"

namespace Lightning
{
class DeepSpaceTargetModel : public TargetModel
{
public:

    DeepSpaceTargetModel() : TargetModel()
    {
        _keyPoints = std::vector<cv::Point3d> {
            {34.978, 0, 0},
            {336.542, 0, 0},
            {84.16, 12.719, 0},
            {287.36, 12.719, 0},
            {0, 135.25, 0},
            {371.52, 135.25, 0},
            {49.182, 147.97, 0},
            {322.338, 147.97, 0}
        };

        _targetAxes = std::vector<cv::Point3d> {
            // {0,0,0},
            // {100,0,0},
            // {0,100,0},
            // {0,0,100}
            {34.978, 0, 0},
            {336.542, 0, 0},
            {84.16, 12.719, 0},
            {287.36, 12.719, 0},
            {0, 135.25, 0},
            {371.52, 135.25, 0},
            {49.182, 147.97, 0},
            {322.338, 147.97, 0}
        };
    }
};
}