/**
 * @file planarICP.cpp
 * @author Jason Antico
 * @brief Implements planar icp algorithm.
 */
// #include <minmax_def.h>
#include <planarICP.h>

// Using numerical derivative to calculate d(Pose3::Expmap)/dw

#include <boost/foreach.hpp>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <cmath>

using namespace std;
using namespace gtsam;

/* ************************************************************** */
int nearestNeighbor(const Point2& p, const vector<Point2>& qs,
		double inlierThresholdSq) {

	int returnInd = -1;
	double minDistSq = inlierThresholdSq;

	for (unsigned int i = 0; i < qs.size(); i++) {

		double dx = p.x() - qs[i].x();
		double dy = p.y() - qs[i].y();

		double distSq = dx * dx + dy * dy;
//      cout << distSq << endl;

		if (distSq <= minDistSq) {
			minDistSq = distSq;
			returnInd = i;
		}
	}

	return returnInd;
}

/* ************************************************************** */
int nearestNeighbors(vector<int>& nn, const vector<Point2>& ps,
		const vector<Point2>& qs, double inlierThresholdSq) {

	int totalIndices = 0;

	for (unsigned int i = 0; i < ps.size(); i++) {
		int ind = nearestNeighbor(ps[i], qs, inlierThresholdSq);
		if ((i > 0) && (nn.back() == ind)) {
			nn.back() = -1;
			nn.push_back(-1);
		} else
			nn.push_back(ind);

		totalIndices += (ind < 0) ? 0 : 1;
	}

	return totalIndices;
}

/* ************************************************************** */
double estimateRotation(const vector<Point2Pair>& uvs) {
	double numer = 0, denom = 0;

	for (unsigned int i = 0; i < uvs.size(); i++) {

		const Point2& ui = uvs[i].first;
		const Point2& vi = uvs[i].second;
		numer += -vi.y() * (ui.x() - vi.x()) + vi.x() * (ui.y() - vi.y());
		denom += vi.y() * vi.y() + vi.x() * vi.x();
	}

	double rot = 0;
	if (denom != 0)
		rot = numer / denom;

	return rot;
}

/* ************************************************************** */
bool inlierCentroids(vector<Point2Pair>& uvs, const vector<Point2>& ps,
		const vector<Point2>& qs, const vector<int>& nn,
		Point2Pair& centroids) {

	bool success = false;
	int inlierCount = 0;

	double pCentroidX = 0;
	double pCentroidY = 0;
	double qCentroidX = 0;
	double qCentroidY = 0;

	uvs.clear();

	for (unsigned int i = 0; i < nn.size(); i++) {

		if (nn[i] == -1)
			continue;

		pCentroidX += ps[i].x();
		pCentroidY += ps[i].y();
		qCentroidX += qs[nn[i]].x();
		qCentroidY += qs[nn[i]].y();
		uvs.push_back(Point2Pair(ps[i], qs[nn[i]]));
		inlierCount += 1;
	}

	if (inlierCount > 0) {
		pCentroidX /= inlierCount;
		pCentroidY /= inlierCount;
		qCentroidX /= inlierCount;
		qCentroidY /= inlierCount;

		centroids.first = Point2(pCentroidX, pCentroidY);
		centroids.second = Point2(qCentroidX, qCentroidY);

		success = true;
	}

	if (success) {
		for (int i = 0; i < inlierCount; i++) {
			uvs[i].first += Point2(pCentroidX, pCentroidY).inverse();
			uvs[i].second += Point2(qCentroidX, qCentroidY).inverse();
		}
	}
	return success;
}

/* ************************************************************** */
vector<Point2> transformPoints(const vector<Point2>& qs, const Pose2& sTq) {
	vector<Point2> result;
	BOOST_FOREACH(const Point2& q, qs) result.push_back(sTq * q);
	return result;
}

/* ************************************************************** */
Pose2 estimateTransform(const vector<Point2>& ps, const vector<Point2>& qs,
		const vector<int>& nn, double inlierThresholdSq, int& status) {

	// Compute and subtract centroids for inliers only
	vector<Point2Pair> uvs; // filled in call below
	Point2Pair centroids; // filled in call below
	if (!inlierCentroids(uvs, ps, qs, nn, centroids)) {
		status = -2;
		return Pose2::identity();
	}

	// estimate rotation, translation, and create Pose2
	double omega = estimateRotation(uvs);
	Point2 t = centroids.first - Rot2(omega) * centroids.second;
	return Pose2(omega, t);
}

/* ************************************************************** */
//Pose2 ransacTransform(vector<Point2>& ps, vector<Point2>& qs,
//		int& iters, int inlierThresholdSq) {
//	int sampSize = floor(ps.size() / 4);
//	int itersMax = iters;
//	iters = 0;
//	int inliers = 0;
//
//	int inliersMax = 0;
//	double inlierRatio = 0.0;
//
//	vector<int> values, nn;
//	vector<Point2> randXYp, randXYq, tempXYq;
//
//	Pose2 delta, deltaBest;
//	Point2 temp;
//
//	for (int i = 0; i < nn.size(); i++)
//		values[i] = i;
//
//	while (inlierRatio < 0.8 && iters < itersMax) {
//
//		// update
//		inliers = 0;
//		tempXYq = qs;
//
//		// randomly shuffle vector of integers, then subsample ps and qs
//		random_shuffle(values.begin(), values.end());
//
//		for (int i = 0; i < sampSize; i++) {
//			randXYp[i] = ps[values[i]];
//			randXYq[i] = qs[values[i]];
//		}
//
//		nn.clear();
//
//		nearestNeighbors(nn, randXYp, randXYq, inlierThresholdSq);
//
//
//		delta = estimateTransform(randXYp, randXYq, nn, inlierThresholdSq);
//
//		rotatePoints(tempXYq, delta.theta());
//		translatePoints(tempXYq, delta.x(), delta.y());
//
//		for (int j = 0; j < tempXYq.size(); j++) {
//			temp = ps[j] - tempXYq[j];
//			double dist = temp.norm();
//			if (dist < 1E-4)
//				inliers++;
//		}
//
//		if (inliers > inliersMax) {
//			deltaBest = delta;
//			inliersMax = inliers;
//		}
//
//		inlierRatio = inliers / tempXYq.size();
//	}
//	return deltaBest;
//}
/* ************************************************************** */
Pose2 singleICP(const vector<Point2>& ps, const vector<Point2>& qs,
		double inlierThresholdSq, int& status) {

	vector<int> nn;

// perform nearest neighbors matching
	status = nearestNeighbors(nn, ps, qs, inlierThresholdSq);
	if ( status < 1)
		return Pose2::identity();

	return estimateTransform(ps, qs, nn, inlierThresholdSq,status);
}



/* ************************************************************** */
ReturnValue laserScanICP(const vector<Point2>& ps, const vector<Point2>& qs,
		const Pose2& pTq_guess, int maxIters, double stoppingThresh,
		double inlierThreshSq) {

	int status = 1;
	double stopping = stoppingThresh + 1;
	Pose2 sTq = pTq_guess;
	int i;
	for (i = 0; (i < maxIters) && (stopping > stoppingThresh); i++) {
		// transform points S = sTq*Q
		vector<Point2> S = transformPoints(qs, sTq);
		// do NN and estimate transform that makes P==pTs*S
		Pose2 pTs = singleICP(ps, S, inlierThreshSq, status);
		// update pose sTq
		sTq = pTs.compose(sTq);
		// calculate stopping
		stopping = pTs.x() * pTs.x() + pTs.y() * pTs.y()
				+ pTs.theta() * pTs.theta();
	}




	ReturnValue value;
	value.delta = sTq;
	value.numIters = i;
	value.stopping = stopping;
	if (status > 0)
	value.status = (double)status/ps.size();
	else value.status = (double) status;


	return value;
}

/* *************************************************************** */

double getYaw(const Rot3& R) {
	return R.yaw();
}
double getPitch(const Rot3& R) {
	return R.pitch();
}
double getRoll(const Rot3& R) {
	return R.roll();
}
/* *************************************************************** */

