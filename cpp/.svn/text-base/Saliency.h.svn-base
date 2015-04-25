/**
 * For determining the likely informativeness of 2D laser scans, for use in
 * laser scan ICP matching. This is adapted from earlier matlab code.
 *
 * @file Saliency.h
 *
 * @date Feb 6, 2014
 * @author Daniel Keyes
 * @author Vadim Indelman
 */

#pragma once

#include <planarICP_matlab.h>
#include <gtsam/base/Matrix.h>

double calculateScanSaliency(const gtsam::Matrix& scan, int seed);

double calculateScanSaliency(const gtsam::Matrix& scan, int seed,
		double sigma_pos, double sigma_rot, int N, int maxIters,
		double stoppingThresh, double inlierThreshSq, bool use_plicp_delta);

