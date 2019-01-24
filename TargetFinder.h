#pragma once

#include <memory>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

#include "TargetModel.h"
#include "CameraModel.h"

namespace Lightning
{

class VisionData;

// TODO move to other file
typedef std::pair<cv::Point2d, cv::Point2d> TargetCorner;

// TODO new file
class TargetSection
{
public:
    std::vector<cv::Point2f> corners;
    double angle;
    double score; 
    cv::Point2f center;
};

class Target
{
public:
    std::vector<TargetSection> sections;

    cv::Point2f center;

    double distance;
    double yaw;
};

class TargetFinder
{

public:

    TargetFinder(std::shared_ptr<spdlog::logger>, TargetModel, CameraModel);

    bool Process(cv::Mat&, VisionData&);

    void ShowDebugImages();

private:

    void ConvertImage(const cv::Mat&, cv::Mat&, cv::Mat&);

    void FilterOnColor(const cv::Mat&, cv::Mat&, const cv::Scalar, const cv::Scalar, const int iter);

    bool FindContours(const cv::Mat&, std::vector<std::vector<cv::Point>>&);

    void ApproximateContours(const std::vector<std::vector<cv::Point>>&, std::vector<std::vector<cv::Point>>&, cv::Mat&);

    void TargetSectionsFromContours(const std::vector<std::vector<cv::Point>>&, std::vector<TargetSection>&, const cv::Size);

    void SortTargetSections(const std::vector<TargetSection>&, std::vector<Target>&);

    void RefineTargetCorners(std::vector<Target>&, const cv::Mat&);

    void FindTargetTransforms(std::vector<Target>&, const TargetModel&, const CameraModel&, std::vector<cv::Point2d>&);

    double Distance(const cv::Point2d&, const cv::Point2d&);

    cv::Vec3d EulerAnglesFromRotationMaxtrix(const cv::Mat&);

    std::shared_ptr<spdlog::logger> _logger;

    TargetModel _targetModel;
    CameraModel _cameraModel;

    std::vector<std::pair<std::string, cv::Mat>> _debugImages;
};

}