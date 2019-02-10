#include "Target.h"

using namespace Lightning;

void Target::GetInverseTransforms(cv::Mat& rvec, cv::Mat& tvec) const
{
    // Get rotation matrix from vector
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Transpose R to get inverse
    R = R.t();        

    // Inverse of tvec      
    tvec = -R * tvec;

    // Convert rotation vector back to matrix      
    cv::Rodrigues(R, rvec);
}