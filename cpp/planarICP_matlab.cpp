/**
 * @file icp_matlab.cpp
 * @author Jason Antico
 * @brief Implements planar icp algorithm.
 */

// #include <minmax_def.h>
#include <planarICP_matlab.h>
#include <csm_interface.h>
#include <GslErrorHandler.h>

#include <gtsam/base/Matrix.h>

#include <cstdlib>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace gtsam;

/* *************************************************************** */
ReturnValue laserScanICPMatlab(const Matrix& pXY, const Matrix& qXY,
    const Pose2& pTq_guess, int maxIters, double stoppingThresh,
    double inlierThreshSq) {

  vector<Point2> ps, qs;
  for (int j = 0; j < pXY.cols(); j++)
    ps.push_back(Point2(pXY(0, j), pXY(1, j)));
  for (int j = 0; j < qXY.cols(); j++)
    qs.push_back(Point2(qXY(0, j), qXY(1, j)));

  ReturnValue value = laserScanICP(ps, qs, pTq_guess, maxIters, stoppingThresh,
      inlierThreshSq);

  return value;
}

/* *************************************************************** */
pair<ReturnValue,Matrix> PLICP(const Matrix& pXY, const Matrix& qXY, const Pose2& pTq_guess,
    int maxIters) {

  vector<Point2> ps, qs;
  for (int j = 0; j < pXY.cols(); j++)
    ps.push_back(Point2(pXY(0, j), pXY(1, j)));
  for (int j = 0; j < qXY.cols(); j++)
    qs.push_back(Point2(qXY(0, j), qXY(1, j)));

  // create parameters with defaults
  sm_params params = defaultParametersCSM();

  // Convert laserscans
  params.laser_ref = convertCSM(ps);
  params.laser_ref->odometry[0] = 0;
  params.laser_ref->odometry[1] = 0;
  params.laser_ref->odometry[2] = 0;
  params.laser_sens = convertCSM(qs);
  params.laser_sens->odometry[0] = pTq_guess.x();
  params.laser_sens->odometry[1] = pTq_guess.y();
  params.laser_sens->odometry[2] = pTq_guess.theta();
  params.first_guess[0] = pTq_guess.x();
  params.first_guess[1] = pTq_guess.y();
  params.first_guess[2] = pTq_guess.theta();

  params.max_iterations = maxIters;
  params.do_compute_covariance = 1;

  sm_result result;

  GslErrorHandler errorHandler;
  errorHandler.enable();

  bool noErrors = true;
  try{
	  //------- Call Andrea's PLICP matcher -------------
	  //  printParametersCSM(params);
	  sm_icp(&params, &result);
	  //  cout << "x: " << friendly_pose(result.x) << endl;
	  //-------------------------------------------------
  } catch (GslException& e) {
	  noErrors = false;
  }

  ReturnValue value;
  Matrix cov(eye(3));
  if (noErrors && result.valid) {
	  value.delta = Pose2(result.x[0], result.x[1], result.x[2]);
	  value.numIters = result.iterations;
	  value.stopping = result.error;
	  value.status = 1;

	  const double* const data = gsl_matrix_const_ptr(result.cov_x_m, 0, 0);
	  MatrixRowMajor A(3,3);
	  copy(data, data+3*3, A.data());
	  cov = Matrix(A);

  } else {
	  if(noErrors){
		  value.status = -3;
	  } else {
		  value.status = -4;
	  }
  }


#ifdef WRITE_FILES
  FILE* f = fopen("/Users/dellaert/icp/laser_ref.carmen","w");
  ld_write_as_carmen(params.laser_ref, f);
  fclose(f);
  f = fopen("/Users/dellaert/icp/laser_sens.carmen","w");
  ld_write_as_carmen(params.laser_sens, f);
  fclose(f);
#endif
  ld_free(params.laser_ref);
  ld_free(params.laser_sens);

  return make_pair(value,cov);
}

/* *************************************************************** */



