/**
 * @file testPlanarICP.cpp
 * Unit tests for all functions in icp_3.cpp and createPoints.cpp
 * (which is the ros-dependent function).
 * Written by : Jason Antico, Daniel Keyes, Frank Dellaert
 * Date: March 2013
 *
 */

#include <planarICP.h>

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

#include <fstream>

#include <CppUnitLite/TestHarness.h>

#define ROOT_TWO 1.41421356237

using namespace std;
using namespace gtsam;

/* ***************************************************** */
TEST( planar_icp, test_nearestNeighbors ) {
  vector<int> nn;

  vector<Point2> ps, qs;

  double inlierThresholdSq = 0.3;

  ps.push_back(Point2(10.5, 20.5));
  ps.push_back(Point2(11.5, 21.5));
  ps.push_back(Point2(12.5, 22.5));
  ps.push_back(Point2(13.5, 23.5));
  ps.push_back(Point2(14.5, 24.5));
  ps.push_back(Point2(15.5, 25.5));
  ps.push_back(Point2(16.5, 26.5));

  qs.push_back(Point2(110.6, 120.6));
  qs.push_back(Point2(111.6, 121.6));
  qs.push_back(Point2(12.6, 22.6));
  qs.push_back(Point2(113.6, 123.6));
  qs.push_back(Point2(14.6, 24.6));
  qs.push_back(Point2(15.6, 25.6));
  qs.push_back(Point2(16.6, 26.6));

  int output = nearestNeighbors(nn, ps, qs, inlierThresholdSq);

  EXPECT_DOUBLES_EQUAL(4, output, 0.0001);

  EXPECT_DOUBLES_EQUAL(7, nn.size(), 0.0001);
  EXPECT_DOUBLES_EQUAL(-1, nn[0], 0.0001);
  EXPECT_DOUBLES_EQUAL(-1, nn[1], 0.0001);
  EXPECT_DOUBLES_EQUAL(2, nn[2], 0.0001);
  EXPECT_DOUBLES_EQUAL(-1, nn[3], 0.0001);
  EXPECT_DOUBLES_EQUAL(4, nn[4], 0.0001);
  EXPECT_DOUBLES_EQUAL(5, nn[5], 0.0001);
  EXPECT_DOUBLES_EQUAL(6, nn[6], 0.0001);
}

/* ***************************************************** */
TEST( planar_icp, test_inlierCentroids ) {
  /*
   bool inlierCentroids(vector<Point2Pair>& uvs, vector<Point2> ps,
   vector<Point2> qs, vector<int>& nn, Point2Pair& centroids)
   */

  vector<Point2> xy1;
  vector<Point2> xy2;
  vector<Point2Pair> expected;

  xy1.push_back(Point2(-2, 0));
  xy1.push_back(Point2(-1, 2));
  xy1.push_back(Point2(0.5, 3));
  xy1.push_back(Point2(1, 2));
  xy1.push_back(Point2(2, 0));

  xy2.push_back(Point2(-2, 0.5));
  xy2.push_back(Point2(-1, 2.5));
  xy2.push_back(Point2(0.5, 4.5));
  xy2.push_back(Point2(1, 2.5));
  xy2.push_back(Point2(2, 0.5));

  expected.push_back(Point2Pair(Point2(-2, -1), Point2(-2, -1)));
  expected.push_back(Point2Pair(Point2(-1, 1), Point2(-1, 1)));
  expected.push_back(Point2Pair(Point2(1, 1), Point2(1, 1)));
  expected.push_back(Point2Pair(Point2(2, -1), Point2(2, -1)));

  vector<Point2Pair> uvs;

  Point2Pair centroids;

  vector<int> nn;
  nn.push_back(-1);
  nn.push_back(-1);
  nn.push_back(-1);
  nn.push_back(-1);
  nn.push_back(-1);

  inlierCentroids(uvs, xy1, xy2, nn, centroids);

  EXPECT(uvs.size()==0)

  nn.clear();

  nn.push_back(0);
  nn.push_back(1);
  nn.push_back(-1);
  nn.push_back(3);
  nn.push_back(4);

  inlierCentroids(uvs, xy1, xy2, nn, centroids);

  EXPECT(uvs.size() == 4);

  EXPECT(
      expected[0].first.equals(uvs[0].first)&&expected[0].second.equals( uvs[0].second));
  EXPECT(
      expected[1].first.equals(uvs[1].first)&&expected[1].second.equals( uvs[1].second));
  EXPECT(
      expected[2].first.equals(uvs[2].first)&&expected[2].second.equals( uvs[2].second));
  EXPECT(
      expected[3].first.equals(uvs[3].first)&&expected[3].second.equals( uvs[3].second));
}

/* ***************************************************** */
TEST( planar_icp, estimateRotation ) {

  // should return identity for empty list
  vector<Point2Pair> uvs;
  double actual1 = estimateRotation(uvs);
  EXPECT_DOUBLES_EQUAL(0, actual1, 1e-9);

  // real example
  double expected2 = 0;
  uvs.push_back(
      Point2Pair(Point2(12.0 - 14, 23.0 - 21),
          Point2(12.1 - 14.1, 23.1 - 21.1)));
  uvs.push_back(
      Point2Pair(Point2(14.5 - 14, 24.5 - 21),
          Point2(14.6 - 14.1, 24.6 - 21.1)));
  uvs.push_back(
      Point2Pair(Point2(15.5 - 14, 25.5 - 21),
          Point2(15.6 - 14.1, 25.6 - 21.1)));
  double actual2 = estimateRotation(uvs);
  EXPECT_DOUBLES_EQUAL(expected2, actual2, 1e-9);

  // real example
  double expected3 = -0.0001;
  vector<Point2Pair> uvs2;
  uvs2.push_back(Point2Pair(Point2(1, 0), Point2(1, 0.0001)));
  uvs2.push_back(Point2Pair(Point2(-1, 0), Point2(-1, -0.0001)));
  double actual3 = estimateRotation(uvs2);
  EXPECT_DOUBLES_EQUAL(expected3, actual3, 1e-9);

}

/* ***************************************************** */
TEST( planar_icp, estimateTransform ) {

  /*gtsam::Pose2 estimateTransform(vector<Point2>& ps, vector<Point2>& qs,
   vector<int>& nn, double inlierThresholdSq)
   */

  double inlierThresholdSq = .6;
  gtsam::Pose2 delta;
  int status = 1;

  vector<Point2> xy1, xy2;
  vector<int> nn;
  vector<Point2Pair> uvs;

  xy1.push_back(Point2(-2, 0));
  xy1.push_back(Point2(-1, 2));
  xy1.push_back(Point2(0.5, 3));
  xy1.push_back(Point2(1, 2));
  xy1.push_back(Point2(2, 0));

  xy2.push_back(Point2(-2, 0));
  xy2.push_back(Point2(-1, 2));
  xy2.push_back(Point2(0.5, 4));
  xy2.push_back(Point2(1, 2));
  xy2.push_back(Point2(2, 0));

  nn.push_back(0);
  nn.push_back(1);
  nn.push_back(-1);
  nn.push_back(3);
  nn.push_back(4);

  delta = estimateTransform(xy1, xy2, nn, inlierThresholdSq,status);
  cout << "status is: " << status << endl;

  EXPECT(assert_equal(Pose2(),delta,1e-8));

  xy2.clear();
  xy2.push_back(Point2(-2, 0.5));
  xy2.push_back(Point2(-1, 2.5));
  xy2.push_back(Point2(0.5, 4.5));
  xy2.push_back(Point2(1, 2.5));
  xy2.push_back(Point2(2, 0.5));

  double angle = M_PI / 30;
  xy2 = transformPoints(xy2, Pose2(0, 0, angle));
  delta = estimateTransform(xy1, xy2, nn, inlierThresholdSq,status);
  cout << "status is: " << status << endl;

  EXPECT(assert_equal(Pose2(0,-0.5,-angle),delta,1e-3));
}

/* ***************************************************** */
TEST( planar_icp, singleICP ) {
  /* double singleICP(vector<Point2>& ps, vector<Point2>& qs,
   double inlierThresholdSq, gtsam::Pose2& delta)
   */

  double inlierThresholdSq = .6;
  double angle = 0;
  gtsam::Pose2 delta;
  int status = 1;

  vector<Point2> xy1, xy2;

  xy1.push_back(Point2(-2, 0));
  xy1.push_back(Point2(-1, 2));
  xy1.push_back(Point2(0.5, 3));
  xy1.push_back(Point2(1, 2));
  xy1.push_back(Point2(2, 0));

  xy2.push_back(Point2(-2, 0));
  xy2.push_back(Point2(-1, 2));
  xy2.push_back(Point2(0.5, 4));
  xy2.push_back(Point2(1, 2));
  xy2.push_back(Point2(2, 0));

  delta = singleICP(xy1, xy2, inlierThresholdSq, status);
  cout << "status is: " << status << endl;

  EXPECT(assert_equal(Pose2(),delta,1e-8));

  angle = M_PI / 30;

  xy2.clear();
  xy2.push_back(Point2(-2, 0.5));
  xy2.push_back(Point2(-1, 2.5));
  xy2.push_back(Point2(0.5, 4.5));
  xy2.push_back(Point2(1, 2.5));
  xy2.push_back(Point2(2, 0.5));

  xy2 = transformPoints(xy2, Pose2(0, 0, angle));
  delta = singleICP(xy1, xy2, inlierThresholdSq, status);
  cout << "status is: " << status << endl;;

  // should work
  EXPECT(assert_equal(Pose2(0,-0.5,-angle),delta,1e-3));
}

/* ***************************************************** */
TEST( planar_icp, laserScanICP ) {
  /*int laserScanICP(std::vector<Point2>& ps, std::vector<Point2>& qs,
   gtsam::Pose2& delta, int& iters, double stoppingThresh,
   double inlierThreshSq)
   */

  double inlierThresholdSq = .6;
  double stoppingThresh = 1E-20;
  int iters = 100;

  gtsam::Pose2 delta;
  ReturnValue values, values2;

  vector<Point2> xy1, xy2;

  xy1.push_back(Point2(-2, 0));
  xy1.push_back(Point2(-1, 2));
  xy1.push_back(Point2(0.5, 3));
  xy1.push_back(Point2(1, 2));
  xy1.push_back(Point2(2, 0));

  xy2.push_back(Point2(-2, 0));
  xy2.push_back(Point2(-1, 2));
  xy2.push_back(Point2(0.5, 4));
  xy2.push_back(Point2(1, 2));
  xy2.push_back(Point2(2, 0));

  values = laserScanICP(xy1, xy2, delta, iters, stoppingThresh,
      inlierThresholdSq);

  EXPECT(assert_equal(Pose2(),delta,1e-8));

  double angle = M_PI / 30;
  delta = gtsam::Pose2(0, 0, 0);

  xy2.clear();
  xy2.push_back(Point2(-2, 0.5));
  xy2.push_back(Point2(-1, 2.5));
  xy2.push_back(Point2(0.5, 4.5));
  xy2.push_back(Point2(1, 2.5));
  xy2.push_back(Point2(2, 0.5));

  xy2 = transformPoints(xy2, Pose2(0, 0, angle));
  values2 = laserScanICP(xy1, xy2, delta, iters, stoppingThresh,
      inlierThresholdSq);

  // should work
  EXPECT(assert_equal(Pose2(0,-0.5,-angle),values2.delta,1e-4));

  // better initial estimate
  delta = gtsam::Pose2(0, 0, - M_PI / 31);

  xy2.clear();
  xy2.push_back(Point2(-2, 0.5));
  xy2.push_back(Point2(-1, 2.5));
  xy2.push_back(Point2(0.5, 4.5));
  xy2.push_back(Point2(1, 2.5));
  xy2.push_back(Point2(2, 0.5));

  xy2 = transformPoints(xy2, Pose2(0, 0, angle));
  values2 = laserScanICP(xy1, xy2, delta, iters, stoppingThresh,
      inlierThresholdSq);

  // should work
  EXPECT(assert_equal(Pose2(0,-0.5,-angle),values2.delta,1e-4));
}

/* ***************************************************** */
TEST( planar_icp, test_get_Euler_angles) {
  Rot3 R(Quaternion(1, 0, 0, 0));
  double yaw = getYaw(R);
  double pitch = getPitch(R);
  double roll = getRoll(R);

  EXPECT_DOUBLES_EQUAL(0, yaw, 1E-9);
  EXPECT_DOUBLES_EQUAL(0, pitch, 1E-9);
  EXPECT_DOUBLES_EQUAL(0, roll, 1E-9);

  // at sequence 5857, time 67.721
  /*		x: -0.018763
   y:  0.021318
   z:  0.789437
   w:  0.613174
   *
   */

  Rot3 R2(Quaternion(0.613174, -0.018763, 0.021318, 0.789437));

  double yaw2 = getYaw(R2);
  double pitch2 = getPitch(R2);
  double roll2 = getRoll(R2);

  // expected values computed in matlab using equation from wiki
  EXPECT_DOUBLES_EQUAL(1.8211, yaw2, 3*M_PI/180);
  EXPECT_DOUBLES_EQUAL(0.0558, pitch2, 6*M_PI/180);
  EXPECT_DOUBLES_EQUAL(0.0107, roll2, 6*M_PI/180);
}

/* ***************************************************** */
// Read in scan file dumped from matlab
vector<Point2> readScan_PlanarICPformat(const string& filename, size_t length) {
  ifstream is(filename.c_str());
  if (!is.good())
    throw runtime_error(filename + ": file not found");

  vector<Point2> scan;

  double a,b;
  for (size_t i = 0; i < length; i++){
    is >> a >> b;
    scan.push_back(Point2(a, b));
  }
  return scan;
}

// test problematic scans from Vadim
#include "../cpp/planarICP_matlab.h"
TEST( planar_icp, problemScans ) {
  // the path is relative to build directory
  vector<Point2> ps = readScan_PlanarICPformat("../tests/scan5.txt", 688);
  vector<Point2> qs = readScan_PlanarICPformat("../tests/scan6.txt", 688);
  Pose2 delta;

  double inlierThresholdSq = .6;
  double stoppingThresh = 1E-20;
  int iters = 100;
  int maxIters = 50;

  ReturnValue values, values2;
  {
    // order ps, qs
    values = laserScanICP(ps, qs, delta, iters, stoppingThresh,
        inlierThresholdSq);
    std::cout<<"(ps, qs): values.status: "<<values.status<<std::endl;

    //    pair<ReturnValue, Matrix> result = PLICP(ps, qs, delta, maxIters);
    //    const ReturnValue &r = result.first;
    //    EXPECT_LONGS_EQUAL(195, r.numIters); // number of iterations used ?
    //    Pose2 expected(-0.915601, -0.0034645, 0.0543123);
    //    EXPECT(assert_equal(expected, r.delta, 1e-6)); // Actual result
    //    EXPECT_DOUBLES_EQUAL(39.320442965485697, r.stopping, 1e-9); // Actual squared error
    //    EXPECT_DOUBLES_EQUAL(1, r.status, 1e-9); // positive for successful ICP, -1 for failed nearest neighbors,-2 for failed inlier centroids
  }
  {
    // order qs, ps
    values = laserScanICP(qs, ps, delta, iters, stoppingThresh,
        inlierThresholdSq);
    std::cout<<"(qs, ps): values.status: "<<values.status<<std::endl;

    //    // order qs, ps
    //    pair<ReturnValue, Matrix> result = PLICP(qs, ps, delta, maxIters);
    //    const ReturnValue &r = result.first;
    //    EXPECT_LONGS_EQUAL(136, r.numIters); // number of iterations used ?
    //    Pose2 expected(-0.0428304, -0.00816988, -0.0643578);
    //    EXPECT(assert_equal(expected, r.delta, 1e-6)); // Actual result
    //    EXPECT_DOUBLES_EQUAL(45.881055341851543, r.stopping, 1e-9); // Actual squared error
    //    EXPECT_DOUBLES_EQUAL(1, r.status, 1e-9); // positive for successful ICP, -1 for failed nearest neighbors,-2 for failed inlier centroids
  }
}


/* ***************************************************** */
int main(int argc, char** argv) {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ***************************************************** */

