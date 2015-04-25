/**
 * @file testSaliency.cpp
 *
 * @date Feb 6, 2014
 * @author Daniel Keyes
 */

#include <Saliency.h>

#include <fstream>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;


/* ***************************************************** */
// Read in scan file dumped from matlab
Matrix readScan(const string& filename, size_t length) {
  ifstream is(filename.c_str());
  if (!is.good())
    throw runtime_error(filename + ": file not found");
  Matrix scan(2, length);
  for (size_t i = 0; i < length; i++)
    is >> scan(0, i) >> scan(1, i);
  return scan;
}

TEST( planar_icp, calculateScanSaliency ) {
  // this is robot 1, scan 1 of the 14-01-20 dataset
  Matrix scan = readScan("../tests/scan7.txt", 721);

  int seed = 12345;
  // this value was obtained by manually running the matlab code on these seeded perturbations
  double expected = 3.8057;

  double actual;
  actual = calculateScanSaliency(scan, seed);
  EXPECT_DOUBLES_EQUAL(expected, actual, 1e-4);

  // same test, but specifying custom parameters
  double sigma_pos = 0.5;
  double sigma_rot = 0.5/180*M_PI;
  int N = 20;
  int maxIters = 50;
  double stoppingThresh = 1E-6;
  double inlierThreshSq = 0.5;
  bool use_plicp_delta = true;

  // value differs when we used slightly different parameters
  expected = 110.355;
  actual = calculateScanSaliency(scan, seed, sigma_pos, sigma_rot, N,
    maxIters, stoppingThresh, inlierThreshSq, use_plicp_delta);
  EXPECT_DOUBLES_EQUAL(expected, actual, 1e-2);


  // this is robot 1, scan 619 of the 14-01-20 dataset
  scan = readScan("../tests/scan8.txt", 721);
  expected = 11.9677;

  actual = calculateScanSaliency(scan, seed);
  EXPECT_DOUBLES_EQUAL(expected, actual, 1e-4);

}



/* ***************************************************** */
int main(int argc, char** argv) {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ***************************************************** */

