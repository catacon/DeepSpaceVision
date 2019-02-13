#pragma once

#include <memory>

#include <opencv2/opencv.hpp>
#include "spdlog/spdlog.h"

#include "TargetModel.h"
#include "CameraModel.h"
#include "VisionData.hpp"
#include "Target.h"

namespace Lightning
{

class TargetFinder
{

public:

    TargetFinder(std::vector<spdlog::sink_ptr>, std::string, std::unique_ptr<TargetModel>, std::unique_ptr<CameraModel>);

    bool Process(cv::Mat&, std::vector<VisionData>&);

    void ShowDebugImages();

private:

    void ConvertImage(const cv::Mat&, cv::Mat&, cv::Mat&);

    void FilterOnColor(const cv::Mat&, cv::Mat&, const cv::Scalar, const cv::Scalar, const int iter);

    bool FindContours(const cv::Mat&, std::vector<std::vector<cv::Point>>&);

    void ApproximateContours(const std::vector<std::vector<cv::Point>>&, std::vector<std::vector<cv::Point>>&, cv::Mat&);

    void TargetSectionsFromContours(const std::vector<std::vector<cv::Point>>&, std::vector<TargetSection>&, const cv::Size);

    void SortTargetSections(const std::vector<TargetSection>&, std::vector<Target>&);

    void RefineTargetCorners(std::vector<Target>&, const cv::Mat&);

    void FindTargetTransforms(std::vector<Target>&, const cv::Size&);

    double Distance(const cv::Point2d&, const cv::Point2d&);

    bool ClockwiseSort(const cv::Point2f&, const cv::Point2f&, const cv::Point2f&);

    cv::Vec3d EulerAnglesFromRotationMaxtrix(const cv::Mat&);

    void DrawDebugImage(cv::Mat&, const std::vector<Target>&);

    std::shared_ptr<spdlog::logger> _logger;

    std::unique_ptr<TargetModel> _targetModel;
    std::unique_ptr<CameraModel> _cameraModel;

    std::vector<std::pair<std::string, cv::Mat>> _debugImages;

    std::string _name;
};

}