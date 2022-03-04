

#include <gtest/gtest.h>
#include <ocs2_core/misc/Lookup.h>

using namespace ocs2;
using namespace lookup;

TEST(testLookup, findIndexInTimeArray) {
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(findIndexInTimeArray(timeArray, -2.0), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArray, 0.0), 1);
  ASSERT_EQ(findIndexInTimeArray(timeArray, 2.0), 1);
  ASSERT_EQ(findIndexInTimeArray(timeArray, 2.5), 2);
  ASSERT_EQ(findIndexInTimeArray(timeArray, 3.0), 2);
  ASSERT_EQ(findIndexInTimeArray(timeArray, 4.0), 3);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(findIndexInTimeArray(timeArrayRepeated, 1.9), 1);
  ASSERT_EQ(findIndexInTimeArray(timeArrayRepeated, 2.0), 1);
  ASSERT_EQ(findIndexInTimeArray(timeArrayRepeated, 2.1), 4);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(findIndexInTimeArray(timeArraySingle, 0.0), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArraySingle, 1.0), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArraySingle, 2.0), 1);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(findIndexInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArrayEmpty, 0.0), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArrayEmpty, 1.0), 0);
}

TEST(testLookup, findIndexInTimeArray_precision_lowNumbers) {
  std::vector<double> timeArray{0.0};
  double tQuery = timeArray.front();

  // Test next representable number in both directions
  double tQueryMinus = std::nextafter(tQuery, std::numeric_limits<double>::lowest());
  double tQueryPlus = std::nextafter(tQuery, std::numeric_limits<double>::max());

  // Make sure the tested values have the correct ordering
  ASSERT_TRUE(tQueryMinus <= timeArray.front());
  ASSERT_TRUE(tQuery <= timeArray.front());
  ASSERT_TRUE(tQueryPlus > timeArray.front());

  // Test lookup
  ASSERT_EQ(findIndexInTimeArray(timeArray, tQuery), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArray, tQueryMinus), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArray, tQueryPlus), 1);
}

TEST(testLookup, findIndexInTimeArray_precision_highNumbers) {
  std::vector<double> timeArray{10000.0};
  double tQuery = timeArray.front();

  // Test next representable number in both directions
  double tQueryMinus = std::nextafter(tQuery, std::numeric_limits<double>::lowest());
  double tQueryPlus = std::nextafter(tQuery, std::numeric_limits<double>::max());

  // Make sure the tested values have the correct ordering
  ASSERT_TRUE(tQueryMinus <= timeArray.front());
  ASSERT_TRUE(tQuery <= timeArray.front());
  ASSERT_TRUE(tQueryPlus > timeArray.front());

  // Test lookup
  ASSERT_EQ(findIndexInTimeArray(timeArray, tQuery), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArray, tQueryMinus), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArray, tQueryPlus), 1);
}

TEST(testLookup, findIntervalInTimeArray) {
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(findIntervalInTimeArray(timeArray, -2.0), -1);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, -1.0), -1);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, 4.0), 2);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(findIntervalInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(findIntervalInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(findIntervalInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(findIntervalInTimeArray(timeArraySingle, 0.0), -1);
  ASSERT_EQ(findIntervalInTimeArray(timeArraySingle, 1.0), -1);
  ASSERT_EQ(findIntervalInTimeArray(timeArraySingle, 2.0), 0);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(findIntervalInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(findIntervalInTimeArray(timeArrayEmpty, 0.0), 0);
  ASSERT_EQ(findIntervalInTimeArray(timeArrayEmpty, 1.0), 0);
}

TEST(testLookup, findActiveIntervalInTimeArray) {
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArray, -2.0), -1);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArray, 4.0), 2);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArraySingle, 0.0), -1);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArraySingle, 1.0), 0);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArraySingle, 2.0), 0);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArrayEmpty, 0.0), 0);
  ASSERT_EQ(findActiveIntervalInTimeArray(timeArrayEmpty, 1.0), 0);
}

TEST(testLookup, findBoundedActiveIntervalInTimeArray) {
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_ANY_THROW(findBoundedActiveIntervalInTimeArray(timeArray, -2.0));  // throws on -1
  ASSERT_EQ(findBoundedActiveIntervalInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(findBoundedActiveIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(findBoundedActiveIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(findBoundedActiveIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(findBoundedActiveIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_ANY_THROW(findBoundedActiveIntervalInTimeArray(timeArray, 4.0));  // throws on 2

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(findBoundedActiveIntervalInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(findBoundedActiveIntervalInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(findBoundedActiveIntervalInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time -> always throws
  std::vector<double> timeArraySingle{1.0};
  ASSERT_ANY_THROW(findBoundedActiveIntervalInTimeArray(timeArraySingle, 0.0));
  ASSERT_ANY_THROW(findBoundedActiveIntervalInTimeArray(timeArraySingle, 1.0));
  ASSERT_ANY_THROW(findBoundedActiveIntervalInTimeArray(timeArraySingle, 2.0));

  // empty time -> always throws
  std::vector<double> timeArrayEmpty;
  ASSERT_ANY_THROW(findBoundedActiveIntervalInTimeArray(timeArrayEmpty, -1.0));
  ASSERT_ANY_THROW(findBoundedActiveIntervalInTimeArray(timeArrayEmpty, 0.0));
  ASSERT_ANY_THROW(findBoundedActiveIntervalInTimeArray(timeArrayEmpty, 1.0));
}
