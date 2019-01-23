#include "TargetFinder.h"
#include "CameraModel.h"
#include "TargetModel.h"
#include "Setup.hpp"

using namespace Lightning;

TargetFinder::TargetFinder(std::shared_ptr<Setup> setup, std::shared_ptr<spdlog::logger> logger, TargetModel targetModel, CameraModel cameraModel)
    : _setup(setup)
    , _logger(logger)
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
        if (itc->size() < 10)   // TODO define threshold
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
        _logger->trace("FindContours(): No contours found.");
        return false;
    }

    return true;
}

void TargetFinder::ApproximateContours(const std::vector<std::vector<cv::Point>>& contours, std::vector<std::vector<cv::Point>>& approximation, cv::Mat& image)
{

    if (_setup->DebugImages)
    {
        image = cv::Mat::zeros(image.size(), CV_8UC1);
    }

    // Approximate
    approximation.resize(contours.size());

    for (int i = 0; i < contours.size(); ++i)
    {
        cv::approxPolyDP(contours[i], approximation[i], 2.5, true);     // TODO threshold

        if (_setup->DebugImages)
        {
            cv::drawContours(image, approximation, i, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
        }
    }

}

bool TargetFinder::Process(cv::Mat& image, VisionData& data)
{
    // Convert image to HSV and gray
    cv::Mat hsvImage, grayImage;
    ConvertImage(image, hsvImage, grayImage);

    // Filter based on color
    cv::Mat rangedImage;

    // TODO define ranges and iterations 40, 30, 50 -> 100, 255, 255
    FilterOnColor(hsvImage, rangedImage, cv::Scalar(40, 30, 50), cv::Scalar(100, 255, 200), 2);

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

    FindTargetTransforms(targets, _targetModel, _cameraModel);

    for (int i = 0; i < targets.size(); ++i)
    {
        _logger->debug("Target {0} Distance: {1}, Yaw: {2}", i, targets[i].distance, targets[i].yaw);
    }

    if (_setup->DebugImages)
    {
        cv::RNG rng(12345);
        for (auto& target : targets)
        {
            cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

                cv::circle(image, target.sections[0].corners[0], 5, cv::Scalar(255,0,0), 1, cv::LINE_AA);
                cv::circle(image, target.sections[1].corners[0], 5, cv::Scalar(0,255,0), 1, cv::LINE_AA);
                cv::circle(image, target.sections[0].corners[1], 5, cv::Scalar(0,0,255), 1, cv::LINE_AA);
                cv::circle(image, target.sections[1].corners[1], 5, cv::Scalar(255,255,0), 1, cv::LINE_AA);
                cv::circle(image, target.sections[0].corners[2], 5, cv::Scalar(255,0,255), 1, cv::LINE_AA);
                cv::circle(image, target.sections[1].corners[2], 5, cv::Scalar(0,255,255), 1, cv::LINE_AA);
                cv::circle(image, target.sections[0].corners[3], 5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
                cv::circle(image, target.sections[1].corners[3], 5, cv::Scalar(127,127,127), 1, cv::LINE_AA);

            

            cv::circle(image, target.center, 5, color, cv::FILLED, cv::LINE_AA);
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
        
        double area = cv::contourArea(contours[i], false);      // TODO params?
        double perimeter = cv::arcLength(contours[i], true);

        double shapeFactor = (4 * CV_PI * area) / std::pow(perimeter, 2);

        if (shapeFactor > 0.4 && shapeFactor < 0.8)    // TODO define thresholds
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
            cv::FAST(contourImage, keyPoints, 30, false, cv::FastFeatureDetector::DetectorType::TYPE_9_16);   // TODO thresholds

            std::vector<cv::Point2f> points;
            for (int i = 0; i < keyPoints.size(); ++i)
            {
                points.push_back(keyPoints[i].pt);
            }

            // Combine close corner points
            std::vector<int> labels;
            int numLabels = cv::partition(points, labels, [this](cv::Point2d p1, cv::Point2d p2){ return (Distance(p1, p2) < 2); });    // TODO threshold

            if (numLabels == 4)
            {
                std::vector<cv::Point2f> combinedPoints(numLabels);

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
                }

                cv::Point2f center(0, 0);

                for (auto& pt : combinedPoints)
                {
                    center.x += pt.x;
                    center.y += pt.y;
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
            double angleDiff = (sections[i].angle - sections[j].angle);
            double centerDiff = Distance(sections[i].center, sections[j].center);

            if (angleDiff > 90 && angleDiff < 180 && centerDiff < 160)   // TODO threshold
            {
                Target newTarget;

                newTarget.sections.push_back(sections[i]);
                newTarget.sections.push_back(sections[j]);

                std::sort(newTarget.sections.begin(), newTarget.sections.end(), [](TargetSection& s1, TargetSection& s2){ return (s1.center. x < s2.center.x);});


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
            cv::cornerSubPix(image, section.corners, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 100, 0.1)); // TODO params

            target.center.x += section.center.x;
            target.center.y += section.center.y;

            std::sort(section.corners.begin(), section.corners.end(), [](cv::Point2f p1, cv::Point2f p2){ return p1.x < p1.x && p1.y > p2.y;});

            // Sort corners in relation to center
            std::vector<cv::Point2f> sortedCorners(section.corners.size());
            for (auto c : section.corners)
            {
                if (c.y < target.center.y)
                {
                    if (c.x < section.center.x)
                    {
                        sortedCorners[0] = c;
                    }
                    else
                    {
                        sortedCorners[1] = c;
                    }
                }
                else
                {
                    if (c.x < section.center.x)
                    {
                        sortedCorners[3] = c;
                    }
                    else
                    {
                        sortedCorners[2] = c;
                    }
                }
            }

            //section.corners = sortedCorners;
        }

        target.center.x /= target.sections.size();
        target.center.y /= target.sections.size();
    }
}

void TargetFinder::FindTargetTransforms(std::vector<Target>& targets, const TargetModel& targetModel, const CameraModel& cameraModel)
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

        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);     
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1); 

        if (keyPoints.size() <= 0 || imagePoints.size() <= 0)
        {
            _logger->error("KeyPoints and ImagePoints can not be empty. {0}, {1}", keyPoints.size(), imagePoints.size());
            continue;
        }

        // Find transform
        bool solved = cv::solvePnP(keyPoints, imagePoints, cameraModel.GetCameraMatrix(), cameraModel.GetDistanceCoefficients(), rvec, tvec, false, cv::TermCriteria::MAX_ITER);

        if (!solved)
        {
            _logger->debug("Failed to find target transform");  // TODO target id?
            continue;
        }

        // convert rotation vector to rotation matrix
        cv::Mat Rc;
        cv::Rodrigues(rvec, Rc);

        // target center in target coordinates
        cv::Mat TTp = cv::Mat::zeros(4, 1, CV_64FC1);
        TTp.at<double>(0,0) = 185;   // TODO Same as below??
        TTp.at<double>(1,0) = 73.5;
        TTp.at<double>(2,0) = 0;
        TTp.at<double>(3,0) = 1;    // 1?

        cv::Mat TT(4, 4, Rc.type());    // T is 4x4
        TT(cv::Range(0,3), cv::Range(0,3)) = Rc * 1;    // copies R into T
        TT(cv::Range(0,3), cv::Range(3,4)) = tvec * 1;  // copies tvec into T

        // fill last row of T
        double * p = TT.ptr<double>(3);
        p[0] = p[1] = p[2] = 0;
        p[3] = 1;

        cv::Mat Tp = TT * TTp;

        cv::Point3d targetPoint(Tp.at<double>(0,0), Tp.at<double>(1,0), Tp.at<double>(2,0));
        cv::Point3d shooterPoint(0, 10, 0);    // TODO change
        cv::Point3d unitPoint(0,10,0);          // TODO change

        //cv::Vec3d shooterVector(shooterPoint.x - unitPoint.x, 0, shooterPoint.z - unitPoint.z);
        cv::Vec3d shooterVector(0, 10, 0);
        //cv::Vec3d targetVector(targetPoint.x - shooterPoint.x, 0, targetPoint.z - shooterPoint.z);
        cv::Vec3d targetVector(targetPoint.x, targetPoint.y, targetPoint.z);

        double dot = targetVector.dot(shooterVector);
        double shooterMagnitude = std::sqrt(std::pow(shooterVector[0], 2) 
                + std::pow(shooterVector[1], 2) + std::pow(shooterVector[2], 2));
        double targetMagnitude = std::sqrt(std::pow(targetVector[0], 2) 
                + std::pow(targetVector[1], 2) + std::pow(targetVector[2], 2));

        double newRobotYaw =  (targetVector[0] > 0 ? 1 : -1) 
            * ((180 / CV_PI) * std::acos(dot / (shooterMagnitude * targetMagnitude)) - 180);

        // Center of shooter in camera coordinates
        cv::Mat Sp = cv::Mat::zeros(4, 1, CV_64FC1); 
        Sp.at<double>(0,0) = 0;
        Sp.at<double>(1,0) = 0;
        Sp.at<double>(2,0) = 0;
        Sp.at<double>(3,0) = 1;

        // Inverse of rotation matrix
        cv::Mat Rt = Rc.t();

        // Find inverse camera transform
        cv::Mat t = -Rt * tvec;
        cv::Mat T(4, 4, Rt.type()); // T is 4x4
        T( cv::Range(0,3), cv::Range(0,3) ) = Rt * 1; // copies R into T
        T( cv::Range(0,3), cv::Range(3,4) ) = t * 1; // copies tvec into T

        // fill the last row of T (NOTE: depending on your types, use float or double)
        p = T.ptr<double>(3);
        p[0] = p[1] = p[2] = 0; 
        p[3] = 1;		

        // Center of shooter in target coordinates
        cv::Mat TSp = T * Sp;

        // Center of target in target coordinates
        cv::Mat Cp = cv::Mat::zeros(4, 1, CV_64FC1); 
        Cp.at<double>(0,0) = 185.76;    // TODO change
        Cp.at<double>(1,0) = 73.5;
        Cp.at<double>(2,0) = 0;
        Cp.at<double>(3,0) = 1;

        // Distance from shooter center to target center
        double newRobotDistance = std::sqrt(std::pow(TSp.at<double>(0,0) - Cp.at<double>(0,0), 2) 
                + std::pow(TSp.at<double>(1,0) - Cp.at<double>(1,0), 2) 
                + std::pow(TSp.at<double>(2,0) - Cp.at<double>(2,0), 2));


        target.distance = newRobotDistance;
        target.yaw = newRobotYaw;

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