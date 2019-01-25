#pragma once

#include <memory>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

#include "TargetModel.h"
#include "CameraModel.h"
#include "VisionData.hpp"

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
    cv::RotatedRect rect;
    double score; 
    cv::Point2f center;
};

class Target
{
public:
    std::vector<TargetSection> sections;
    cv::Point2f center;
    VisionData data;
    cv::Mat rvec;
    cv::Mat tvec;

    void GetInverseTransforms(cv::Mat&, cv::Mat&);
};

class TargetFinder
{

public:

    TargetFinder(std::shared_ptr<spdlog::logger>, TargetModel, CameraModel);

    bool Process(cv::Mat&, std::vector<VisionData>&);

    int ShowDebugImages();

private:

    void ConvertImage(const cv::Mat&, cv::Mat&, cv::Mat&);

    void FilterOnColor(const cv::Mat&, cv::Mat&, const cv::Scalar, const cv::Scalar, const int iter);

    bool FindContours(const cv::Mat&, std::vector<std::vector<cv::Point>>&);

    void ApproximateContours(const std::vector<std::vector<cv::Point>>&, std::vector<std::vector<cv::Point>>&, cv::Mat&);

    void TargetSectionsFromContours(const std::vector<std::vector<cv::Point>>&, std::vector<TargetSection>&, const cv::Size);

    void SortTargetSections(const std::vector<TargetSection>&, std::vector<Target>&);

    void RefineTargetCorners(std::vector<Target>&, const cv::Mat&);

    void FindTargetTransforms(std::vector<Target>&, const TargetModel&, const CameraModel&, const cv::Size);

    double Distance(const cv::Point2d&, const cv::Point2d&);

    cv::Vec3d EulerAnglesFromRotationMaxtrix(const cv::Mat&);

    void DrawDebugImage(cv::Mat&, const std::vector<Target>&);

    std::shared_ptr<spdlog::logger> _logger;

    TargetModel _targetModel;
    CameraModel _cameraModel;

    std::vector<std::pair<std::string, cv::Mat>> _debugImages;
};

}