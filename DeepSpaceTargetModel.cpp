#include "DeepSpaceTargetModel.h"

using namespace Lightning;

DeepSpaceTargetModel::DeepSpaceTargetModel() : TargetModel()
{
    _keyPoints = std::vector<cv::Point3d> {
        {185.76, 73.985, 0},
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
        {0,0,0},
        {100,0,0},
        {0,100,0},
        {0,0,100}
    };
}

std::vector<cv::Point3d> DeepSpaceTargetModel::GetSubTargetKeyPoints(int subTarget) const 
{   
    // TODO this could be better - may require redesign of target model

    std::vector<cv::Point3d> keyPoints;

    if (subTarget == 0)
    {
        keyPoints = std::vector<cv::Point3d> {
            {34.978, 0, 0},
            {84.16, 12.719, 0},
            {0, 135.25, 0},
            {49.182, 147.97, 0}
        };
    }
    else
    {
        keyPoints = std::vector<cv::Point3d> {
            {336.542, 0, 0},
            {287.36, 12.719, 0},
            {371.52, 135.25, 0},
            {322.338, 147.97, 0}
        };
    }

    return keyPoints;
    
}