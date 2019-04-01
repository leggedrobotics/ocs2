//
// Created by ruben on 02.11.18.
//

#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_core/loopshaping/LoopshapingCost.h>
#include <ocs2_core/loopshaping/LoopshapingConstraint.h>
#include <ocs2_core/pathfile.h>
#include <gtest/gtest.h>

using namespace ocs2;

TEST(testLoopshapingDefinition, SISO_Definition) {
  std::string settingsFile = std::string(OCS2_CORE_PACKAGE_PATH) + "/test/loopshaping/loopshaping.conf";
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(settingsFile, pt);

  SISOFilterDefinition filter0(pt, "q_filter", "Filter0");
  std::cout << "\nFilter0" << std::endl;
  filter0.print();

  SISOFilterDefinition filter1(pt, "q_filter", "Filter1");
  std::cout << "\nFilter1" << std::endl;
  filter1.print();

  SISOFilterDefinition filter2(pt, "q_filter", "Filter2");
  std::cout << "\nFilter2" << std::endl;
  filter2.print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, MIMO_Definition) {
  std::string settingsFile = std::string(OCS2_CORE_PACKAGE_PATH) + "/test/loopshaping/loopshaping.conf";

  MIMOFilterDefinition filter;
  filter.loadSettings(settingsFile, "q_filter");
  filter.print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Loopshaping_Definition) {
  std::string settingsFile = std::string(OCS2_CORE_PACKAGE_PATH) + "/test/loopshaping/loopshaping.conf";

  LoopshapingDefinition filter;
  filter.loadSettings(settingsFile);
  filter.print();

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Cost_Augmentation) {
  std::string settingsFile = std::string(OCS2_CORE_PACKAGE_PATH) + "/test/loopshaping/loopshaping.conf";

  std::shared_ptr<LoopshapingDefinition> filter(new LoopshapingDefinition());
  filter->loadSettings(settingsFile);
  filter->print();

  constexpr size_t n_sys = 5;
  constexpr size_t m_sys = 3;
  constexpr size_t n_q = 0;
  constexpr size_t n_r = 4;
  constexpr size_t n_s = 4;
  constexpr size_t m_s = 3;
  Eigen::Matrix<double, n_sys, n_sys> Q;
  Eigen::Matrix<double, m_sys, n_sys> P;
  Eigen::Matrix<double, m_sys, m_sys> R;
  Eigen::Matrix<double, n_sys + n_q + n_r + n_s, n_sys + n_q + n_r + n_s> Q_augmented;
  Eigen::Matrix<double, m_sys + m_s, n_sys + n_q + n_r + n_s> P_augmented;
  Eigen::Matrix<double, m_sys + m_s, m_sys + m_s> R_augmented;
  Q.setIdentity();
  P.setZero();
  R.setIdentity();

  Eigen::Matrix<double, n_sys, 1> x_sys;
  Eigen::Matrix<double, m_sys, 1> u_sys;
  x_sys.setZero();
  u_sys.setZero();

  LoopshapingCost<n_sys, m_sys, n_q + n_r + n_s, m_s> loopshapingCost(filter, Q, R, x_sys, u_sys, Q, x_sys);
  loopshapingCost.getIntermediateCostSecondDerivativeState(Q_augmented);
  loopshapingCost.getIntermediateCostDerivativeInputState(P_augmented);
  loopshapingCost.getIntermediateCostSecondDerivativeInput(R_augmented);

  std::cout << "Q:\n" << Q << std::endl;
  std::cout << "Q_augmented:\n" << Q_augmented << std::endl;

  std::cout << "P:\n" << P << std::endl;
  std::cout << "P_augmented:\n" << P_augmented << std::endl;

  std::cout << "R:\n" << R << std::endl;
  std::cout << "R_augmented:\n" << R_augmented << std::endl;

  ASSERT_TRUE(true);
}

TEST(testLoopshapingDefinition, Constraint_augmentation) {
  std::string settingsFile = std::string(OCS2_CORE_PACKAGE_PATH) + "/test/loopshaping/loopshaping.conf";

  std::shared_ptr<LoopshapingDefinition> filter(new LoopshapingDefinition());
  filter->loadSettings(settingsFile);
  filter->print();

  constexpr size_t n_sys = 5;
  constexpr size_t m_sys = 3;
  constexpr size_t n_q = 0;
  constexpr size_t n_r = 4;
  constexpr size_t n_s = 4;
  constexpr size_t m_s = 3;

  Eigen::Matrix<double, m_sys + m_s, n_sys + n_q + n_r + n_s> C_augmented;
  Eigen::Matrix<double, m_sys + m_s, m_sys + m_s> D_augmented;

  LoopshapingConstraint<n_sys, m_sys, n_q + n_r + n_s, m_s> loopshapingConstraint(filter);
  loopshapingConstraint.getConstraint1DerivativesState(C_augmented);
  loopshapingConstraint.getConstraint1DerivativesControl(D_augmented);

  std::cout << "C_augmented:\n" << C_augmented << std::endl;
  std::cout << "D_augmented:\n" << D_augmented << std::endl;


  ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}