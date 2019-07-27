//
// Created by rgrandia on 27.07.19.
//

#include <gtest/gtest.h>
#include <ocs2_core/misc/Lookup.h>

using namespace ocs2;
using namespace Lookup;

TEST(testLookup, findIndexInTimeArray)
{
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(findIndexInTimeArray(timeArray, -2.0), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArray, 0.0), 1);
  ASSERT_EQ(findIndexInTimeArray(timeArray, 2.0), 1);
  ASSERT_EQ(findIndexInTimeArray(timeArray, 2.5), 2);
  ASSERT_EQ(findIndexInTimeArray(timeArray, 3.0), 2);
  ASSERT_EQ(findIndexInTimeArray(timeArray,  4.0), 3);

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
  ASSERT_EQ(findIndexInTimeArray(timeArrayEmpty,  0.0), 0);
  ASSERT_EQ(findIndexInTimeArray(timeArrayEmpty,  1.0), 0);
}

TEST(testLookup, findIntervalInTimeArray)
{
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(findIntervalInTimeArray(timeArray, -2.0), -1);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, -1.0), -1);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(findIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_EQ(findIntervalInTimeArray(timeArray,  4.0), 2);

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
  ASSERT_EQ(findIntervalInTimeArray(timeArrayEmpty,  0.0), 0);
  ASSERT_EQ(findIntervalInTimeArray(timeArrayEmpty,  1.0), 0);
}

TEST(testLookup, findPartitionInTimeArray)
{
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(findPartitionInTimeArray(timeArray, -2.0), -1);
  ASSERT_EQ(findPartitionInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(findPartitionInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(findPartitionInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(findPartitionInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(findPartitionInTimeArray(timeArray, 3.0), 1);
  ASSERT_EQ(findPartitionInTimeArray(timeArray,  4.0), 2);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(findPartitionInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(findPartitionInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(findPartitionInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(findPartitionInTimeArray(timeArraySingle, 0.0), -1);
  ASSERT_EQ(findPartitionInTimeArray(timeArraySingle, 1.0), 0);
  ASSERT_EQ(findPartitionInTimeArray(timeArraySingle, 2.0), 0);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(findPartitionInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(findPartitionInTimeArray(timeArrayEmpty,  0.0), 0);
  ASSERT_EQ(findPartitionInTimeArray(timeArrayEmpty,  1.0), 0);
}

TEST(testLookup, findActivePartitionInTimeArray)
{
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_ANY_THROW(findActivePartitionInTimeArray(timeArray, -2.0)); // throws on -1
  ASSERT_EQ(findActivePartitionInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(findActivePartitionInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(findActivePartitionInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(findActivePartitionInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(findActivePartitionInTimeArray(timeArray, 3.0), 1);
  ASSERT_ANY_THROW(findActivePartitionInTimeArray(timeArray,  4.0)); // throws on 2

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(findActivePartitionInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(findActivePartitionInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(findActivePartitionInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time -> always throws
  std::vector<double> timeArraySingle{1.0};
  ASSERT_ANY_THROW(findActivePartitionInTimeArray(timeArraySingle, 0.0));
  ASSERT_ANY_THROW(findActivePartitionInTimeArray(timeArraySingle, 1.0));
  ASSERT_ANY_THROW(findActivePartitionInTimeArray(timeArraySingle, 2.0));

  // empty time -> always throws
  std::vector<double> timeArrayEmpty;
  ASSERT_ANY_THROW(findActivePartitionInTimeArray(timeArrayEmpty, -1.0));
  ASSERT_ANY_THROW(findActivePartitionInTimeArray(timeArrayEmpty,  0.0));
  ASSERT_ANY_THROW(findActivePartitionInTimeArray(timeArrayEmpty,  1.0));
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
