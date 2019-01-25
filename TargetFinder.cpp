#include "TargetFinder.h"
#include "CameraModel.h"
#include "TargetModel.h"
#include "Setup.h"
#include "VisionData.hpp"

using namespace Lightning;

TargetFinder::TargetFinder(std::shared_ptr<spdlog::logger> logger, TargetModel targetModel, CameraModel cameraModel)
    : _logger(logger)
    , _targetModel(targetModel)
    , _cameraModel(cameraModel)
{

}

void TargetFinder::ConvertImage(const cv::Mat& image, cv::Mat& hsv, cv::Mat& gray)
{
    // Convert image to HSV and gray
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
}

void TargetFinder::FilterOnColor(const cv::Mat& hsv, cv::Mat& ranged, const cv::Scalar low, const cv::Scalar high, const int iter)
{
    // Filter based on color
    cv::inRange(hsv, low, high, ranged);

    // Close disconnected contours
    cv::morphologyEx(ranged, ranged, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), iter);

    // Blur?
}

bool TargetFinder::FindContours(const cv::Mat& image, std::vector<std::vector<cv::Point>>& contours)
{
    // Find contours
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // Filter out small contours
    std::vector<std::vector<cv::Point>>::iterator itc = contours.begin();

    while (itc != contours.end())
    {
        if (itc->size() < Setup::Processing::ContourSizeThreshold)
        {
            itc = contours.erase(itc);
        }
        else
        {
            ++itc;
        }
    }

    if (contours.size() <= 0)
    {
        _logger->debug("FindContours(): No contours found.");
        return false;
    }

    return true;
}

void TargetFinder::ApproximateContours(const std::vector<std::vector<cv::Point>>& contours, std::vector<std::vector<cv::Point>>& approximation, cv::Mat& image)
{

    if (Setup::Diagnostics::DisplayDebugImages)
    {
        image = cv::Mat::zeros(image.size(), CV_8UC1);
    }

    // Approximate
    approximation.resize(contours.size());

    for (int i = 0; i < contours.size(); ++i)
    {
        cv::approxPolyDP(contours[i], approximation[i], Setup::Processing::ContourApproximationAccuracy, true);

        if (Setup::Diagnostics::DisplayDebugImages)
        {
            cv::drawContours(image, approximation, i, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
        }
    }

}

bool TargetFinder::Process(cv::Mat& image, std::vector<VisionData>& data)
{
    using namespace Setup::HSVFilter;

    // Convert image to HSV and gray
    cv::Mat hsvImage, grayImage;
    ConvertImage(image, hsvImage, grayImage);

    // Filter based on color
    cv::Mat rangedImage;

    FilterOnColor(hsvImage, rangedImage, cv::Scalar(LowH, LowS, LowV), cv::Scalar(HighH, HighS, HighV), MorphologyIterations);

    // Detect contours
    std::vector<std::vector<cv::Point>> contours;

    if (!FindContours(rangedImage, contours))
    {
        // No contours, so nothing to process
        return false;
    }

    // Approximate contours
    std::vector<std::vector<cv::Point>> approx(contours.size());
    cv::Mat contourImage = cv::Mat(image.size(), CV_8UC1);

    ApproximateContours(contours, approx, contourImage);

    std::vector<TargetSection> targetSections;

    TargetSectionsFromContours(approx, targetSections, cv::Size(image.cols, image.rows));

    std::vector<Target> targets;

    SortTargetSections(targetSections, targets);

    RefineTargetCorners(targets, grayImage);

    std::vector<cv::Point2d> projectedPoints;
    FindTargetTransforms(targets, _targetModel, _cameraModel, projectedPoints);

    // for (int i = 0; i < targets.size(); ++i)
    // {
    //     _logger->debug("Target {0} Distance: {1}, Yaw: {2}", i, targets[i].distance, targets[i].yaw);
    // }

    for (int i = 0; i < (int)targets.size(); ++i)
    {
        targets[i].data.cameraId = 0;
        targets[i].data.targetId = i;

        targets[i].data.imageX = (targets[i].center.x - (image.cols / 2)) / (image.cols / 2);
        targets[i].data.imageY = ((image.rows / 2) - targets[i].center.y) / (image.rows / 2);

        data.push_back(targets[i].data);
    }

    if (Setup::Diagnostics::DisplayDebugImages)
    {
        cv::RNG rng(12345);
        for (auto& target : targets)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

            /*
            cv::circle(image, target.sections[0].corners[0], 5, cv::Scalar(255,0,0), 1, cv::LINE_AA);
            cv::circle(image, target.sections[1].corners[0], 5, cv::Scalar(255,0,0), 1, cv::LINE_AA);
            cv::circle(image, target.sections[0].corners[1], 5, cv::Scalar(0,255,0), 1, cv::LINE_AA);
            cv::circle(image, target.sections[1].corners[1], 5, cv::Scalar(0,255,0), 1, cv::LINE_AA);
            cv::circle(image, target.sections[0].corners[2], 5, cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::circle(image, target.sections[1].corners[2], 5, cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::circle(image, target.sections[0].corners[3], 5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
            cv::circle(image, target.sections[1].corners[3], 5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
            */
            
            for (auto& pt : projectedPoints)
            {
                cv::circle(image, pt, 3, cv::Scalar(255,0,0), 1, cv::LINE_AA);
            }            

            auto textX = fmt::format("X: {0}", target.data.x);
            auto textY = fmt::format("Y: {0}", target.data.y);
            auto textZ = fmt::format("Z: {0}", target.data.z);

            auto textRoll = fmt::format("Roll: {0}", target.data.roll);
            auto textPitch = fmt::format("Pitch: {0}", target.data.pitch);
            auto textYaw = fmt::format("Yaw: {0}", target.data.yaw);

            auto textImageX = fmt::format("ImageX: {0}", target.data.imageX);
            auto textImageY = fmt::format("ImageY: {0}", target.data.imageY);

            cv::putText(image, textX, cv::Point(10,30), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0));
            cv::putText(image, textY, cv::Point(10,50), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0));
            cv::putText(image, textZ, cv::Point(10,70), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0));
            cv::putText(image, textRoll, cv::Point(10,90), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0));
            cv::putText(image, textPitch, cv::Point(10,110), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0));
            cv::putText(image, textYaw, cv::Point(10,130), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0));
            cv::putText(image, textImageX, cv::Point(10,150), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0));
            cv::putText(image, textImageY, cv::Point(10,170), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,0));
        }

        _debugImages.clear();
        _debugImages.push_back(std::make_pair("Raw", image));
        _debugImages.push_back(std::make_pair("Contours", contourImage));
    }

    return true;
}

void TargetFinder::TargetSectionsFromContours(const std::vector<std::vector<cv::Point>>& contours, std::vector<TargetSection>& sections, const cv::Size size)
{
    sections.clear();

    // Evaluate each contour to see if it is a target section
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        
        double area = cv::contourArea(contours[i], false);  
        double perimeter = cv::arcLength(contours[i], true);

        double shapeFactor = (4 * CV_PI * area) / std::pow(perimeter, 2);

        if (shapeFactor > Setup::Processing::ShapeFactorMin && shapeFactor < Setup::Processing::ShapeFactorMax)
        {
            // Get bounding box and angle
            auto rect = cv::minAreaRect(contours[i]);

            if (rect.size.width < rect.size.height)
            {
                rect.angle += 180;
            }
            else
            {
                rect.angle += 90;
            }

            cv::Mat contourImage = cv::Mat::zeros(size, CV_8UC1);
        
            cv::drawContours( contourImage, contours, i, cv::Scalar(255), cv::FILLED, cv::LINE_AA);

            cv::GaussianBlur( contourImage, contourImage, cv::Size(3,3), 2);

            // Get corners

            std::vector<cv::KeyPoint> keyPoints;
            cv::FAST(contourImage, keyPoints, Setup::Processing::FastThreshold, false, cv::FastFeatureDetector::DetectorType::TYPE_9_16);

            std::vector<cv::Point2d> points;
            for (int i = 0; i < keyPoints.size(); ++i)
            {
                points.push_back(keyPoints[i].pt);
            }

            // Combine close corner points
            std::vector<int> labels;
            int numLabels = cv::partition(points, labels, [this](cv::Point2d p1, cv::Point2d p2){ return (Distance(p1, p2) < Setup::Processing::CornerDistanceThreshold); });

            if (numLabels == 4) // TODO define number of corners?
            {
                std::vector<cv::Point2f> combinedPoints(numLabels);

                cv::Point2d center(0, 0);

                for (size_t i = 0; i < combinedPoints.size(); ++i)
                {
                    cv::Point2d newPoint;
                    int count(0);

                    for (size_t j = 0; j < points.size(); ++j)
                    {
                        if (labels[j] == i)
                        {
                            newPoint.x += points[j].x;
                            newPoint.y += points[j].y;

                            ++count;
                        }
                    }

                    if (count <= 0)
                    {
                        break;
                    }

                    newPoint.x /= count;
                    newPoint.y /= count;

                    combinedPoints[i] = newPoint;

                    center.x += newPoint.x;
                    center.y += newPoint.y;
                }

                center.x /= combinedPoints.size();
                center.y /= combinedPoints.size();

                TargetSection section { combinedPoints, rect.angle, shapeFactor, center };

                sections.push_back(section);
            }
        }
    }
}

void TargetFinder::SortTargetSections(const std::vector<TargetSection>& sections, std::vector<Target>& targets)
{
    targets.clear();

    for (int i = 0; i < (int)sections.size(); ++i)
    {
        for (int j = i + 1; j < (int)sections.size(); ++j)
        {
            // TODO this needs work
            double angleDiff = std::abs(sections[i].angle - sections[j].angle);
            double centerDiff = Distance(sections[i].center, sections[j].center);

            if (angleDiff > Setup::Processing::MinAngleDiff && angleDiff < Setup::Processing::MaxAngleDiff && centerDiff < Setup::Processing::MaxTargetSeparation)
            {
                Target newTarget;

                newTarget.sections.push_back(sections[i]);
                newTarget.sections.push_back(sections[j]);

                std::sort(newTarget.sections.begin(), newTarget.sections.end(), [](TargetSection& s1, TargetSection& s2){ return (s1.center.x < s2.center.x);});


                targets.push_back(newTarget);

                break;
            }
        }
    }
}

void TargetFinder::RefineTargetCorners(std::vector<Target>& targets, const cv::Mat& image)
{
    for (auto& target : targets)
    {   
        target.center.x = 0;
        target.center.y = 0;

        // Get sub pixels for each corner
        for (auto& section : target.sections)
        {
            if (section.corners.size() <= 0)
            {
                continue;
            }

            try
            {
                cv::cornerSubPix(image, section.corners, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, Setup::Processing::MaxCornerSubPixelIterations, Setup::Processing::CornerSubPixelThreshold));
            }
            catch (cv::Exception ex)
            {
                _logger->error("RefineTargetCorners() caught exception: {0}", ex.what());
                continue;
            }
            target.center.x += section.center.x;
            target.center.y += section.center.y;

            std::sort(section.corners.begin(), section.corners.end(), [](cv::Point2f p1, cv::Point2f p2){ return p1.x < p1.x && p1.y > p2.y;});
        }

        target.center.x /= target.sections.size();
        target.center.y /= target.sections.size();
    }
}

void TargetFinder::FindTargetTransforms(std::vector<Target>& targets, const TargetModel& targetModel, const CameraModel& cameraModel, std::vector<cv::Point2d>& projectedPoints)
{
    // Get model key points
    auto keyPoints = targetModel.GetKeyPoints();

    for (auto& target : targets)
    {
        // Set image points
        std::vector<cv::Point2d> imagePoints
        {
            target.sections[0].corners[0],
            target.sections[1].corners[0],
            target.sections[0].corners[1],
            target.sections[1].corners[1],
            target.sections[0].corners[2],
            target.sections[1].corners[2],
            target.sections[0].corners[3],
            target.sections[1].corners[3]
        };

        static cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);     
        static cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
        static bool guessOk = false; 

        if (keyPoints.size() <= 0 || imagePoints.size() <= 0)
        {
            _logger->error("KeyPoints and ImagePoints can not be empty. {0}, {1}", keyPoints.size(), imagePoints.size());
            continue;
        }

        // Find transform
        bool solved = cv::solvePnP(keyPoints, imagePoints, cameraModel.GetCameraMatrix(), cameraModel.GetDistanceCoefficients(), rvec, tvec, guessOk, cv::SOLVEPNP_ITERATIVE);

        if (!solved)
        {
            _logger->debug("Failed to find target transform");  // TODO target id?
            continue;
        }

        guessOk = true;

        // convert rotation vector to rotation matrix
        cv::Mat Rc;
        cv::Rodrigues(rvec, Rc);

        // Get Euler angles from rotation maxtrix - TODO use decompose matrix
        cv::Vec3d euler = EulerAnglesFromRotationMaxtrix(Rc);

        euler[0] *= (180/CV_PI);
        euler[1] *= (180/CV_PI);
        euler[2] *= (180/CV_PI);

        target.data.status = VisionStatus::TargetFound;
        target.data.x = tvec.at<double>(0,0);
        target.data.y = tvec.at<double>(1,0);
        target.data.z = tvec.at<double>(2,0);
        target.data.roll = euler[2];
        target.data.pitch = euler[0];
        target.data.yaw = euler[1];
        target.data.imageX = (double)target.center.x;
        target.data.imageY = (double)target.center.y;

        //_logger->debug("Translation: {0}, {1}, {2}", tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
        //_logger->debug("Euler Angles: {0}, {1}, {2}", euler[0], euler[1], euler[2]);

        // TODO Translate from camera to robot

        // Do I need the inverse transform for anything?
        // Should all calculations be done in target/world coordinate?

        // Project target axis points onto image using the transformation
        if (Setup::Diagnostics::DisplayDebugImages)
        {
            auto targetAxes = targetModel.GetTargetAxes();
            cv::projectPoints(targetAxes, rvec, tvec, cameraModel.GetCameraMatrix(), cameraModel.GetDistanceCoefficients(), projectedPoints);

        }


    }

}

double TargetFinder::Distance(const cv::Point2d& pt1, const cv::Point2d& pt2)
{
    double dX(pt1.x - pt2.x);
    double dY(pt1.y - pt2.y);

    return std::sqrt(std::pow(dX, 2) + std::pow(dY, 2));
}

void TargetFinder::ShowDebugImages()
{
    for (auto& image : _debugImages)
    {
        cv::namedWindow(image.first, cv::WINDOW_KEEPRATIO);
        cv::imshow(image.first, image.second);
    }

    cv::waitKey(10);
}

cv::Vec3d TargetFinder::EulerAnglesFromRotationMaxtrix(const cv::Mat& R)
{     
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    double x, y, z;
    if (sy > 1e-6)  // Is the matrix singular
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3d(x, y, z);
  
}