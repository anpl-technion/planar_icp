/**
 * @file planarICP_matlab.h
 * Header files for MATLAB-friendly planar ICP
 * Author Jason Antico
 */

#include <planarICP.h>

/**
 * @brief A for loop of singleICP iterations to be computed while the returned distance squared
 *      is less than some predefined
 */
ReturnValue laserScanICPMatlab(const gtsam::Matrix& ps, const gtsam::Matrix& qs,
    const gtsam::Pose2& pTq_guess, int maxIters, double stoppingThresh,
    double inlierThreshSq);

/**
 * Function that wraps AndreaCensi's CSM implmenetation for easy use with Matlab
 */
std::pair<ReturnValue, gtsam::Matrix> PLICP(const gtsam::Matrix& ps,
    const gtsam::Matrix& qs, const gtsam::Pose2& pTq_guess, int maxIters);

