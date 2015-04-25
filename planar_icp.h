/**
 * Interface specification file for wrapping the planar_icp library in matlab
 * Author: Jason Antico
 */

#include <planarICP_matlab.h>
#include <Saliency.h>
#include <FilteringUtility.h>

// Declare some types we need
virtual class gtsam::Point2;
virtual class gtsam::Pose2;
class gtsam::Values;

// Wrap the ReturnValue class, see icp_3.h
class ReturnValue {
  ReturnValue();
  gtsam::Pose2 getDelta() const;
  int getNumIters() const;
  double getStopping() const;
  double getStatus() const;
};

ReturnValue laserScanICPMatlab(Matrix pXY, Matrix qXY,
    const gtsam::Pose2& pTq_guess, int iters, double stoppingThresh,
    double inlierThreshSq);

// CSM version
pair<ReturnValue, Matrix> PLICP(Matrix pXY, Matrix qXY,
    const gtsam::Pose2& pTq_guess, int maxIters);

double calculateScanSaliency(Matrix scan, int seed,
		double sigma_pos, double sigma_rot, int N, int maxIters,
		double stoppingThresh, double inlierThreshSq, bool use_plicp_delta);


gtsam::Values filterByChr(const gtsam::Values& values, char robotId);



