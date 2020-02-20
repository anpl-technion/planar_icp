/**
 * @file Saliency.cpp
 *
 * @date Feb 6, 2014
 * @author Daniel Keyes
 * @author Vadim Indelman
 */

#include <Saliency.h>

#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <vector>
#include <utility>
#include <cmath>

using namespace gtsam;
using namespace std;


double calculateScanSaliency(const Matrix& scan, int seed){
  double sigma_pos = 0.5;
  double sigma_rot = 0.5/180*M_PI;
  int N = 20;

  int maxIters = 50;
  double stoppingThresh = 1E-6;
  double inlierThreshSq = 0.5;
  bool use_plicp_delta = false;

  return calculateScanSaliency(scan, seed, sigma_pos, sigma_rot, N, maxIters,
		  stoppingThresh, inlierThreshSq, use_plicp_delta);
}

double calculateScanSaliency(const Matrix& scan, int seed,
		double sigma_pos, double sigma_rot, int N, int maxIters,
		double stoppingThresh, double inlierThreshSq, bool use_plicp_delta){
  Matrix x[N];
  vector<const Matrix*> xVec;
  Matrix xSum = zeros(3,1);
  Matrix covSum = zeros(3,3);

  boost::mt19937 generator(seed);
  boost::normal_distribution<double> normalDistribution(0.0, 1.0);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<double> >
  	randNorm(generator, normalDistribution);

  for(int k=0; k < N; k++){
    //generate sample for initial solution for ICP around origin
	double deltaX = randNorm()*sigma_pos;
	double deltaY = randNorm()*sigma_pos;
	double deltaTh = randNorm()*sigma_rot;
	Pose2 initial(deltaX, deltaY, deltaTh);

    Matrix cov;
    Pose2 delta;
    if(use_plicp_delta) {
  	  pair<ReturnValue,Matrix> plicpResult = PLICP(scan, scan, initial, maxIters);
  	  if(plicpResult.first.status < 0){
  	    continue;
      }

  	  delta = plicpResult.first.delta;
      cov = plicpResult.second;
    } else {
      ReturnValue planarIcpResult = laserScanICPMatlab(scan, scan, initial,
        maxIters, stoppingThresh, inlierThreshSq);
      // only accept ICP result if more than 70% nearest neighbor are found
      if(planarIcpResult.status <= 0.7){
     	continue;
      }

      Pose2 planarIcpDelta = planarIcpResult.delta;
	  pair<ReturnValue,Matrix> plicpResult = PLICP(scan, scan, planarIcpDelta, maxIters);
	  if(plicpResult.first.status < 0){
	    continue;
      }

	  delta = planarIcpResult.delta;
      cov = plicpResult.second;
    }

    x[k] = Matrix(3,1);
    x[k] << delta.translation().x(), delta.translation().y(), delta.theta();
    xVec.push_back(&x[k]);
    xSum += x[k];
    covSum += cov;
  }

  int n = xVec.size();
  //cout << "x_mat" << x_mat << endl;

  Matrix x_mat_mean = xSum/n;

  // center the matrices
  vector<const Matrix*> xVecCentered;
  Matrix x_centered[n];
  for(int k=0; k < n; k++){
	  x_centered[k] = *xVec[k] - x_mat_mean;
	  xVecCentered.push_back(&x_centered[k]);
  }

  Matrix x_mat_centered = collect(xVecCentered, 3, 1);
  Matrix x_mat_cov = x_mat_centered * x_mat_centered.transpose() / n;

  Matrix R = covSum/n + x_mat_cov;
  //cout << "R: " << R << endl;

  double saliency = 1/R.trace();
  return saliency;
}
