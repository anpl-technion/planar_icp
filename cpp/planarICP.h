/**
 * @file planarICP.h
 * Header files for planar ICP
 * Author Jason Antico
 */

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot3.h>


/**
 * Return value for Planar ICP
 */
struct ReturnValue {
	ReturnValue() {
	} ///< Needed for MATLAB
	int numIters; ///< number of iterations used ?
	gtsam::Pose2 delta; ///< Actual result
	double stopping; ///< Actual squared error
	double status; ///< positive for successful ICP, -1 for failed nearest neighbors,-2 for failed inlier centroids
	// status is -3 if PLICP fails

	/// Return ICP result
	gtsam::Pose2 getDelta() const {
		return delta;
	}

	/// Return number of iterations
	int getNumIters() const {
		return numIters;
	}

	// Get actual squared error
	double getStopping() const {
		return stopping;
	}
	double getStatus() const {
		return status;
	}
};

//struct gtsam::Point2 {
//	double x_, y_;
//	gtsam::Point2() :
//			x_(0), y_(0) {
//	}
//
//	gtsam::Point2(double x, double y) :
//			x_(x), y_(y) {
//	}
//
//	bool equals(const gtsam::Point2& q, double tol = 1E-9) const {
//		return (fabs(x_ - q.x_) < tol) && (fabs(y_ - q.y_) < tol);
//	}
//};

//struct Pose2 {
//	double x_, y_, theta_;
//	Pose2() :
//			x_(0), y_(0), theta_(0) {
//	}
//
//	Pose2(double x, double y, double theta) :
//			x_(x), y_(y), theta_(theta) {
//	}
//
//	bool equals(const Pose2& q, double tol = 1E-9) const {
//		return fabs(x_ - q.x_) < tol && fabs(y_ - q.y_) < tol
//				&& fabs(theta_ - q.theta_) < tol;
//	}
//};
//typedef std::pair<Point2, Point2> Point2Pair;

/**
 * @brief Finds the index of the nearest neighbor of a point in another set of points.
 *
 * Takes a point (as an index in a list of x,y values) and finds the nearest
 * neighbor to that point in another list of x,y values. Returns the index of
 * that nearest neighbor, or -1 if no nearest neighbor exists.
 *
 * @param fromInd : index of current point in ps that is being matched to the nearest
 * 		  point in qs
 * @param nn : Vector for returning the indices.
 * @param ps : vector of Point2 coordinates for all X and Y that the we are
 * 		  matching nearest neighbors to.
 * @param qs : vector of Point2 coordinates for all X and Y that we want to match with
 * 		  our ps coordinates
 * @param inlierThresholdSq The (squared) threshold value used to determine
 *        inliers.
 */
int nearestNeighbor(const gtsam::Point2& p,
		const std::vector<gtsam::Point2>& qs, double inlierThresholdSq);

/**
 * @brief Finds the indices of the nearest neighbors of all points in a set, in another set of points.
 *
 * Takes a list of points (as one list of x-values, one of y-values) and finds
 * the nearest neighbors of each of those points in another list of x,y values.
 * Returns in a vector passed by reference, the indices of those nearest
 * neighbors, where -1 is in place of those indices if no nearest neighbor
 * exists for that index. Returns by value the number of non-negative indice in
 * the vector.
 *
 * @param nn Vector for returning the indices.
 * @param ps vector of Point2 coordinates for all X and Y that the we are
 * 		  matching nearest neighbors to.
 * @param qs : vector of Point2 coordinates for all X and Y that we want to match with
 * 		  our ps coordinates
 * @param inlierThresholdSq The (squared) threshold value used to determine
 *        inliers.
 */
int nearestNeighbors(std::vector<int>& nn, const std::vector<gtsam::Point2>& ps,
		const std::vector<gtsam::Point2>& qs, double inlierThresholdSq);

/**
 * @brief Computes an estimate for how much to rotate the fromX,fromY points to match up with the toX,toY points.
 *
 * Computes the rotation estimate between two sets of already paired points.  These points are of the data type
 * vector<gtsam::Point2Pair>, so each element in the vector corresponds to a pair of Point2 types.  The first is at time t,
 * and the second is at time t+1.  Each point has already had its respective centroid subtracted.
 *
 * @param uvs : vector<gtsam::Point2Pair> of u and v point coordinates which correspond to each other, and already have
 * 		  had the centroids subtracted away from the coordinates.
 */
double estimateRotation(const std::vector<gtsam::Point2Pair>& uvs);

/**
 * @brief Computes the centroids of the inliers in a nearest neighbor matching,
 * and returns a vector<gtsam::Point2Pair> of the matching coordinates (each still in
 * their own frame) with the centroids subtracted.
 *
 * nn Specifies a nearest neighbor matching between the points of ps
 * to qs. Those points i for which nn[i]==-1 are not inliers, all others
 * are. Returns by value whether there were any inliers to compute the centroid of.
 *
 * @param uvs : reference to vector<gtsam::Point2Pair> of the matched u and v Point2 coordinates
 * 			    after the centroids have been subtracted.
 * @param ps : vector<Point2> of all XY coordinates obtained from the 1st scan.
 * @param qs : vector<Point2> of all XY coordinates obtained from the 2nd scan.
 * @param nn Nearest neighbor matching from ps to qs.
 */
bool inlierCentroids(std::vector<gtsam::Point2Pair>& uvs,
		const std::vector<gtsam::Point2>& ps,
		const std::vector<gtsam::Point2>& qs, const std::vector<int>& nn,
		gtsam::Point2Pair& centroids);

/**
 * @brief Transform all points by given pose
 */
std::vector<gtsam::Point2> transformPoints(const std::vector<gtsam::Point2>& qs,
		const gtsam::Pose2& pTq);

/* @brief Estimates the and performs the transformation from sets of points P to Q, for a
 * 		  single iteration.
 *
 * @param ps Vector of Point2 for the first scan
 * @param qs Vector of Point2 for the second scan
 * @param nn Vector of integers such that nn[i] is the nearest neighbor in the
 * 			 second scan to point i in the first scan
 * @param inlierThresholdSq Squared maximum distance allowable for inlier matching
 *
 * @return deltaIter For a single iteration, this is the calculated transformation between sets of points
 *
 */
gtsam::Pose2 estimateTransform(const std::vector<gtsam::Point2>& ps,
		const std::vector<gtsam::Point2>& qs, const std::vector<int>& nn,
		double inlierThresholdSq, int& status);

/**
 * @brief A single ICP iteration following the algorithm defined in the
 * 		  accompanying PDF document.  Returns stopping value equal to the
 * 		  distance translated plus the change in rotation^2 for the current
 * 		  iteration.
 *
 * @param ps : all XY coordinates for each point in set p.
 * @param qs : all XY coordinates for each point in set q.
 * @param inlierThresholdSq : Threshold for inlier matching
 */
gtsam::Pose2 singleICP(const std::vector<gtsam::Point2>& ps,
		const std::vector<gtsam::Point2>& qs, double inlierThresholdSq, int& status);

//gtsam::Pose2 ransacTransform(std::vector<gtsam::Point2>& ps, std::vector<gtsam::Point2>& qs,
//    int& iters, int inlierThresholdSq);


/**
 * @brief A for loop of singleICP iterations to be computed while the returned distance squared
 * 			is less than some predefined
 * @param ps : all XY coordinates for each point in set p.
 * @param qs : all XY coordinates for each point in set q.
 * @param pTq_guess : estimate for pTq transform
 * @param maxIters : maximum number of iterations
 * @param stoppingThresh : stopping criterion
 * @param inlierThresholdSq : Threshold for inlier matching
 */
ReturnValue laserScanICP(const std::vector<gtsam::Point2>& ps,
		const std::vector<gtsam::Point2>& qs, const gtsam::Pose2& pTq_guess,
		int maxIters, double stoppingThresh, double inlierThreshSq);

double getYaw(const gtsam::Rot3& R);
double getPitch(const gtsam::Rot3& R);
double getRoll(const gtsam::Rot3& R);
