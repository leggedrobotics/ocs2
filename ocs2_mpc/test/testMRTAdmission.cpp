#include <gtest/gtest.h>
#include <ocs2_mpc/MRT_BASE.h>

class BufferedMrt final : public ocs2::MRT_BASE {
public:
  void setCurrentObservation(const ocs2::SystemObservation &) override {}
  void resetMpcNode(const ocs2::TargetTrajectories &) override {}
  void offer(double time) {
    auto command = std::make_unique<ocs2::CommandData>();
    command->mpcInitObservation_.time = time;
    moveToBuffer(std::move(command), std::make_unique<ocs2::PrimalSolution>(),
                 std::make_unique<ocs2::PerformanceIndex>());
  }
};

TEST(MrtAdmission, RejectedBufferPreservesActiveUntilAdmittedReplacement) {
  BufferedMrt mrt;
  mrt.offer(1.0);
  ASSERT_TRUE(mrt.updatePolicy());
  mrt.offer(2.0);
  const auto after_barrier = [](const ocs2::CommandData &command) {
    return command.mpcInitObservation_.time > 2.0;
  };
  EXPECT_FALSE(mrt.updatePolicy(after_barrier));
  EXPECT_DOUBLE_EQ(mrt.getCommand().mpcInitObservation_.time, 1.0);
  mrt.offer(3.0);
  EXPECT_TRUE(mrt.updatePolicy(after_barrier));
  EXPECT_DOUBLE_EQ(mrt.getCommand().mpcInitObservation_.time, 3.0);
  EXPECT_FALSE(mrt.updatePolicy(after_barrier));
}
