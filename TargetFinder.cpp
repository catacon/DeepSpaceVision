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
    // Convert image to HSV and gray
    cv::Mat hsvImage, grayImage;
    ConvertImage(image, hsvImage, grayImage);

    // Filter based on color
    cv::Mat rangedImage;

    FilterOnColor(hsvImage, rangedImage, cv::Scalar(Setup::HSVFilter::LowH, Setup::HSVFilter::LowS, Setup::HSVFilter::LowV), cv::Scalar(Setup::HSVFilter::HighH, Setup::HSVFilter::HighS, Setup::HSVFilter::HighV), MorphologyIterations);

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

    //ApproximateContours(contours, approx, contourImage);

    // Find target sections
    std::vector<TargetSection> targetSections;

    TargetSectionsFromContours(contours, targetSections, cv::Size(image.cols, image.rows));

    // Create targets from sections
    std::vector<Target> targets;

    SortTargetSections(targetSections, targets);

    // Get subpixel measurement on target corners
    RefineTargetCorners(targets, grayImage);

    // Find the camera to target tranform
    FindTargetTransforms(targets, _targetModel, _cameraModel, cv::Size(image.cols, image.rows));

    // Sort targets by horizontal position in image
    std::sort(targets.begin(), targets.end(), [](Target t1, Target t2){ return (t1.data.z < t2.data.z); });

    // Add targets to data packet - TODO this could be cleaner - redo VisionPacket?
    for (int i = 0; i < (int)targets.size(); ++i)
    {
        targets[i].data.cameraId = 0;   // TODO
        targets[i].data.targetId = i;

        data.push_back(targets[i].data);
    }

    if (Setup::Diagnostics::DisplayDebugImages)
    {
        DrawDebugImage(image, targets);

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
            cv::FAST(contourImage, keyPoints, Setup::Processing::FastThreshold, true, cv::FastFeatureDetector::DetectorType::TYPE_9_16);

            // Combine close corner points - TODO with non-max suppression on is this necessary?
            std::vector<int> labels;
            int numLabels = cv::partition(keyPoints, labels, [this](cv::KeyPoint p1, cv::KeyPoint p2){ return (Distance(p1.pt, p2.pt) < Setup::Processing::CornerDistanceThreshold); });

            if (numLabels == 4) // TODO define number of corners?
            {
                std::vector<cv::Point2f> combinedPoints(numLabels);

                cv::Point2d center(0, 0);

                for (size_t i = 0; i < combinedPoints.size(); ++i)
                {
                    cv::Point2d newPoint;
                    int count(0);

                    for (size_t j = 0; j < keyPoints.size(); ++j)
                    {
                        if (labels[j] == i)
                        {
                            newPoint += points[j].pt;
                            ++count;
                        }
                    }

                    if (count <= 0)
                    {
                        break;
                    }

                    newPoint /= count;

                    combinedPoints[i] = newPoint;

                    center += newPoint;
                }

                center / (int)combinedPoints.size();

                TargetSection section { combinedPoints, rect, shapeFactor, center };

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
            double angleDiff = std::abs(sections[i].rect.angle - sections[j].rect.angle);
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

void TargetFinder::FindTargetTransforms(std::vector<Target>& targets, const TargetModel& targetModel, const CameraModel& cameraModel, const cv::Size imageSize)
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
        bool solved = cv::solvePnP(keyPoints, imagePoints, cameraModel.GetCameraMatrix(), cameraModel.GetDistanceCoefficients(), rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

        if (!solved)
        {
            _logger->debug("Failed to find target transform");  // TODO target id?
            continue;
        }

        // TODO Translate from camera to robot

        // Convert rotation vector to rotation matrix
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        // Compute inverse of transform - this gives camera position in target coordinates
        if (Setup::Processing::UseWorldCoordinates)
        {
            R = R.t();              // transpose of R which is also the inverse
            tvec = -Rt * tvec;      // inverse of tvec
        }
        
        // Build transform matrix - not used currently
        /*
        cv::Mat P = cv::Mat::eye(4, 4, R.type());           // T is 4x4
        P( cv::Range(0,3), cv::Range(0,3) ) = R * 1;        // copies R into T
        P( cv::Range(0,3), cv::Range(3,4) ) = tvec * 1;     // copies tvec into T
        */

        // Get Euler angles from rotation maxtrix
        cv::Vec3d euler = EulerAnglesFromRotationMaxtrix(R);

        euler[0] *= (180/CV_PI);
        euler[1] *= (180/CV_PI);
        euler[2] *= (180/CV_PI);

        target.data.status = VisionStatus::TargetFound;
        target.data.x = tvec.at<double>(0,0);
        target.data.y = tvec.at<double>(1,0);
        target.data.z = tvec.at<double>(2,0);
        target.data.pitch = euler[0];
        target.data.yaw = euler[1];
        target.data.roll = euler[2];
        target.data.imageX = (target.center.x - (imageSize.width / 2)) / (imageSize.width / 2);
        target.data.imageY = ((imageSize.height / 2) - target.center.y) / (imageSize.height / 2);
        target.rvec = rvec;
        target.tvec = tvec;
    }

}

double TargetFinder::Distance(const cv::Point2d& pt1, const cv::Point2d& pt2)
{
    double dX(pt1.x - pt2.x);
    double dY(pt1.y - pt2.y);

    return std::sqrt(std::pow(dX, 2) + std::pow(dY, 2));
}

int TargetFinder::ShowDebugImages()
{
    for (auto& image : _debugImages)
    {
        cv::namedWindow(image.first, cv::WINDOW_KEEPRATIO);
        cv::imshow(image.first, image.second);
    }

    return cv::waitKey(1);
}

cv::Vec3d TargetFinder::EulerAnglesFromRotationMaxtrix(const cv::Mat& R)
{     
    double sy = std::sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    cv::Vec3d vec;
    if (sy > 1e-6)  // Is the matrix singular
    {
        vec[0] = atan2(R.at<double>(2,1) , R.at<double>(2,2));  // x
        vec[1] = atan2(-R.at<double>(2,0), sy);                 // y
        vec[2] = atan2(R.at<double>(1,0), R.at<double>(0,0));   // x
    }
    else
    {
        vec[0] = atan2(-R.at<double>(1,2), R.at<double>(1,1));  // x
        vec[1] = atan2(-R.at<double>(2,0), sy);                 // y
        vec[2] = 0;                                             // z
    }

    return vec;  
}

void TargetFinder::DrawDebugImage(cv::Mat& image, const std::vector<Target>& targets)
{
    RNG rng(12345);
    for (auto& target : targets)
    {            
        cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

        // Project target points back onto image
        cv::Mat rvec, tvec;
        if (Setup:Processing::UseWorldCoordinates)
        {
            target.GetInverseTransforms(rvec, tvec);
        }
        else
        {
            rvec = target.rvec;
            tvec = target.tvec;
        }
        
        std::vector<cv::Point2d> projectedPoints;
        cv::projectPoints(_targetModel.GetKeyPoints(), rvec, tvec, _cameraModel.GetCameraMatrix(), _cameraModel.GetDistanceCoefficients(), projectedPoints);

        for (auto& pt : projectedPoints)
        {
            cv::circle(image, pt, 3, color, 1, cv::LINE_AA);
        }         

        // Show target section bounding boxes
        for (auto& section : target.sections)
        {
            Point2f vertices[4]; 
            section.rect.points( vertices );

            for( int j = 0; j < 4; j++ )
            {
                cv::line( image, vertices[j], vertices[(j+1)%4], color, 1, cv::LINE_AA );
            }
        }

        // Show target data
        std::vector<std::string> imageText;   

        // TODO make this into a function?
        imageText.push_back(fmt::format("X: {0}", target.data.x));
        imageText.push_back(fmt::format("Y: {0}", target.data.y));
        imageText.push_back(fmt::format("Z: {0}", target.data.z));

        imageText.push_back(fmt::format("Roll: {0}", target.data.roll));
        imageText.push_back(fmt::format("Pitch: {0}", target.data.pitch));
        imageText.push_back(fmt::format("Yaw: {0}", target.data.yaw));

        imageText.push_back(fmt::format("ImageX: {0}", target.data.imageX));
        imageText.push_back(fmt::format("ImageY: {0}", target.data.imageY));

        for (int i = 0; i < (int)imageText.size(); ++i)
        {
            cv::putText(image, imageText[i], cv::Point(10,30 + i*20), cv::FONT_HERSHEY_PLAIN, 1.0, color);
        }
    }
}

void Target::GetInverseTransforms(cv::Mat& rvec, cv::Mat& tvec)
{
    // Get rotation matrix from vector
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Transpose R to get inverse
    R = R.t();        

    // Inverse of tvec      
    tvec = -Rt * tvec;

    // Convert rotation vector back to matrix      
    cv::Rodrigues(R, rvec);
}