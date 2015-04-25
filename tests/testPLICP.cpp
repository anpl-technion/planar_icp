/**
 * @file testPLICP.cpp
 * Try Andrea Censi's PLICP scan matcher
 * Written by : Frank Dellaert
 * Date: March 2013
 *
 */

#include <fstream>
#include <stdexcept>

#include <planarICP.h>
#include <csm_interface.h>

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>

#include <CppUnitLite/TestHarness.h>


using namespace std;
using namespace gtsam;

/* ***************************************************** */
extern void sm_options(struct sm_params*p, struct option*ops);

TEST( planar_icp, laserScanICP ) {

  sm_params params = defaultParametersCSM();

  FILE* f = fopen("../csm/laser_ref.carmen", "r");
  if (!f)
    throw runtime_error("file not found");
  params.laser_ref = ld_read_smart(f);
  fclose(f);
  f = fopen("../csm/laser_sens.carmen", "r");
  if (!f)
    throw runtime_error("file not found");
  params.laser_sens = ld_read_smart(f);
  fclose(f);

  params.use_point_to_line_distance = 1;
  params.use_corr_tricks = 1;
  params.max_iterations = 10;
  params.do_compute_covariance = 1;

  sm_result result;

  //------- Call Andrea's PLICP matcher -------------
  sm_icp(&params, &result);
  //-------------------------------------------------

  if (1) {
    printParametersCSM(params);
    cout << "cov_x_m: " << result.cov_x_m->size1 << "," << result.cov_x_m->size2
        << endl;
    double std[3];
    for (int i = 0; i < 3; i++) {
      std[i] = sqrt(gsl_matrix_get(result.cov_x_m, i, i));
      for (int j = 0; j < 3; j++)
        cout << gsl_matrix_get(result.cov_x_m, i, j) << " ";
      cout << endl;
    }
    cout << "x: " << friendly_pose(result.x) << endl;
    cout << "s: " << friendly_pose(std) << endl;
    cout << "iterations:" << result.iterations << endl;
    cout << "error:" << result.error << endl;
  }

  Pose2 actual(result.x[0], result.x[1], result.x[2]);

  ld_free(params.laser_ref);
  ld_free(params.laser_sens);

// should work
  EXPECT(assert_equal(Pose2(0.085, 0.0011, 0.029), actual, 1e-3));

  // the diagonals of a covariance matrix should be positive
  for (int i=0; i < 3; i++) {
	double val = sqrt(gsl_matrix_get(result.cov_x_m, i, i));
	EXPECT(val >= 0);
  }
}

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

// test problematic scans from Vadim
#include "../cpp/planarICP_matlab.h"
TEST( planar_icp, problemScans ) {
  // the path is relative to build directory
  Matrix ps = readScan("../tests/scan1.txt", 725);
  Matrix qs = readScan("../tests/scan2.txt", 725);
  Pose2 delta;
  int maxIters = 50;
  {
    // order ps, qs
    pair<ReturnValue, Matrix> result = PLICP(ps, qs, delta, maxIters);
    const ReturnValue &r = result.first;
    EXPECT_LONGS_EQUAL(195, r.numIters); // number of iterations used ?
    Pose2 expected(-0.915601, -0.0034645, 0.0543123);
    EXPECT(assert_equal(expected, r.delta, 1e-6)); // Actual result
    EXPECT_DOUBLES_EQUAL(39.320442965485697, r.stopping, 1e-9); // Actual squared error
    EXPECT_DOUBLES_EQUAL(1, r.status, 1e-9); // positive for successful ICP, -1 for failed nearest neighbors,-2 for failed inlier centroids
  }
  {
    // order qs, ps
    pair<ReturnValue, Matrix> result = PLICP(qs, ps, delta, maxIters);
    const ReturnValue &r = result.first;
    EXPECT_LONGS_EQUAL(136, r.numIters); // number of iterations used ?
    Pose2 expected(-0.0428304, -0.00816988, -0.0643578);
    EXPECT(assert_equal(expected, r.delta, 1e-6)); // Actual result
    EXPECT_DOUBLES_EQUAL(45.881055341851543, r.stopping, 1e-9); // Actual squared error
    EXPECT_DOUBLES_EQUAL(1, r.status, 1e-9); // positive for successful ICP, -1 for failed nearest neighbors,-2 for failed inlier centroids
  }

  // this algorithm is deterministic, so consecutive runs should produce the
  // same delta and coviariance
  {
	pair<ReturnValue, Matrix> result = PLICP(qs, ps, delta, maxIters);
	Pose2 firstDelta = result.first.delta;
	Matrix firstCov = result.second;
	pair<ReturnValue, Matrix> result2 = PLICP(qs, ps, delta, maxIters);
	Pose2 secondDelta = result2.first.delta;
	Matrix secondCov = result2.second;

	EXPECT(assert_equal(firstDelta, secondDelta));
	EXPECT(assert_equal(firstCov, secondCov));
  }
}

// test problematic scans from Vadim
TEST( planar_icp, problemScans2 ) {
  // the path is relative to build directory
  Matrix ps = readScan("../tests/scan5.txt", 688);
  Matrix qs = readScan("../tests/scan6.txt", 688);
  Pose2 delta;
  int maxIters = 50;
  {
    // order ps, qs
    pair<ReturnValue, Matrix> result = PLICP(ps, qs, delta, maxIters);
    const ReturnValue &r = result.first;

    //  std::cout<<"(ps, qs): values.status: "<<r.status<<std::endl;

    //    EXPECT_LONGS_EQUAL(195, r.numIters); // number of iterations used ?
    //    Pose2 expected(-0.915601, -0.0034645, 0.0543123);
    //    EXPECT(assert_equal(expected, r.delta, 1e-6)); // Actual result
    //    EXPECT_DOUBLES_EQUAL(39.320442965485697, r.stopping, 1e-9); // Actual squared error
    //    EXPECT_DOUBLES_EQUAL(1, r.status, 1e-9); // positive for successful ICP, -1 for failed nearest neighbors,-2 for failed inlier centroids
  }
  {
    // order qs, ps
    pair<ReturnValue, Matrix> result = PLICP(qs, ps, delta, maxIters);
    //    const ReturnValue &r = result.first;

    //    std::cout<<"(qs, ps): values.status: "<<r.status<<std::endl;

    //    EXPECT_LONGS_EQUAL(136, r.numIters); // number of iterations used ?
    //    Pose2 expected(-0.0428304, -0.00816988, -0.0643578);
    //    EXPECT(assert_equal(expected, r.delta, 1e-6)); // Actual result
    //    EXPECT_DOUBLES_EQUAL(45.881055341851543, r.stopping, 1e-9); // Actual squared error
    //    EXPECT_DOUBLES_EQUAL(1, r.status, 1e-9); // positive for successful ICP, -1 for failed nearest neighbors,-2 for failed inlier centroids
  }
}


// test CSM exception-safe behavior
TEST( planar_icp, gslErrorHandler ) {
  // the path is relative to build directory
  Matrix ps = readScan("../tests/scan3.txt", 721);
  Matrix qs = readScan("../tests/scan4.txt", 721);
  Pose2 initial(0,0,0);
  int maxIters = 50;
  double stoppingThresh = 1e-6;
  double inlierThreshSq = 0.5;
  {
	// the following few lines duplicate the logic in our matlab pipeline
	ReturnValue value = laserScanICPMatlab(ps, qs, initial, maxIters, stoppingThresh, inlierThreshSq);
	Pose2 delta = value.delta;
	// TODO: PLICP echos a warning message on this input. disable that somehow.
    pair<ReturnValue, Matrix> result = PLICP(ps, qs, delta, maxIters);
    const ReturnValue &r = result.first;
    EXPECT_LONGS_EQUAL(-4, r.status); // return value should indicate an error
    // note: the other values are not guaranteed to be empty, so make sure to
    // always check r.status!
  }
}



// After a change in the back-end matrix library, the returned covariance
// matrices were inconsistent with earlier calculations. This compares the
// returned matrix to the result produced from known working code.
// It might be worth it to find a case in which the covariance can be easily
// calculated by hand.
TEST( planar_icp, covMatrix ) {
  Matrix ps = readScan("../tests/scan1.txt", 725);
  Matrix qs = readScan("../tests/scan2.txt", 725);
  Pose2 delta;
  int maxIters = 50;
  {
	// order ps, qs
	pair<ReturnValue, Matrix> result = PLICP(ps, qs, delta, maxIters);
	const ReturnValue &r = result.first;
	EXPECT_LONGS_EQUAL(195, r.numIters); // number of iterations used ?
	Pose2 expected(-0.915601, -0.0034645, 0.0543123);
	EXPECT(assert_equal(expected, r.delta, 1e-6)); // Actual result
	EXPECT_DOUBLES_EQUAL(39.320442965485697, r.stopping, 1e-9); // Actual squared error
	EXPECT_DOUBLES_EQUAL(1, r.status, 1e-9); // positive for successful ICP, -1 for failed nearest neighbors,-2 for failed inlier centroids

	Matrix actualCov  = result.second;
	Matrix expectedCov(3,3);
	expectedCov << 0.0159866, -0.00806961, 0.00200321,
				   -0.00806961, 0.0187639, -0.00462308,
				   0.00200321, -0.00462308, 0.00118331;
	EXPECT(assert_equal(actualCov, expectedCov, 1e-7));
  }
}


/* ***************************************************** */
int main(int argc, char** argv) {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ***************************************************** */

