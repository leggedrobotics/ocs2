

#include <gtest/gtest.h>
#include <ocs2_mpc/MPC_OCS2.h>

using namespace ocs2;

TEST(MPC_OCS2, MPC_OCS2_default_construction){
  MPC_OCS2<1, 1> MPC_OCS2;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
