/**
 * @file testFilteringUtility.cpp
 *
 * @date Feb 6, 2014
 * @author Daniel Keyes
 */


#include <FilteringUtility.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Testable.h>

#include <boost/foreach.hpp>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

TEST( planar_icp, filtering ) {
	Pose2 A101(1,2,3);
	Pose2 A102(4,5,6);
	Pose2 B105(7,8,9);

	Values values;
	values.insert(Symbol('A', 101), A101);
	values.insert(Symbol('A', 102), A102);
	values.insert(Symbol('B', 105), B105);
	{
		Values result = filterByChr(values, 'A');
		EXPECT_LONGS_EQUAL(result.size(), 2); // A101, A102
		EXPECT_LONGS_EQUAL(values.size(), 3); // should not change
		KeyVector l = result.keys();

		int i=0;
		BOOST_FOREACH(const Values::KeyValuePair& key_value, result) {
			if(i == 0) {
			  EXPECT_LONGS_EQUAL(Symbol('A', 101), (long)key_value.key);
			  EXPECT(assert_equal(A101, dynamic_cast<const Pose2&>(key_value.value)));
			} else if(i == 1) {
			  EXPECT_LONGS_EQUAL(Symbol('A', 102), (long)key_value.key);
			  EXPECT(assert_equal(A102, dynamic_cast<const Pose2&>(key_value.value)));
			} else {
			  EXPECT(false);
			}
			i++;
		}
	}
}

/* ***************************************************** */
int main(int argc, char** argv) {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ***************************************************** */


