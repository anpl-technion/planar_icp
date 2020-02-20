/**
 * Utility matlab wrapper for gtsam filtering function.
 * @file filteringUtility.h
 *
 * @date Feb 6, 2014
 * @author Daniel Keyes
 * @author Vadim Indelman
 */

#include <FilteringUtility.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>

using namespace gtsam;

Values filterByChr(const Values& values, char robotId){
	return values.filter<Pose2>(Symbol::ChrTest(robotId));
}

