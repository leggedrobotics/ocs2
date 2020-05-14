/*
 * testFindIndex.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: farbod
 */

#include <ocs2_core/dynamics/TransferFunctionBase.h>

#include <gtest/gtest.h>

using namespace ocs2;

TEST(testTransferFunctionBase, noDelay) {
  Eigen::VectorXd num, den;
  Eigen::MatrixXd A, B, C, D;

  num.resize(2);
  den.resize(2);
  num << 0.01, 2.0;
  den << 0.2, 3.0;
  TransferFunctionBase tf1(num, den);
  tf1.getStateSpace(A, B, C, D);

  // Matlab
  // h = tf([0.01, 2.0], [0.2, 3.0])
  // [A, B, C, D] = tf2ss(h.num{:}, h.den{:})
  ASSERT_DOUBLE_EQ(A(0), -15);
  ASSERT_DOUBLE_EQ(B(0), 1);
  ASSERT_DOUBLE_EQ(C(0), 9.25);
  ASSERT_DOUBLE_EQ(D(0), 0.05);

  num.resize(2);
  den.resize(4);
  num << 2.0, 4.0;
  den << 0.2, 3.0, 0.3, 6.0;
  tf2ss(num, den, A, B, C, D, 0.0, false);

  // Matlab
  // h = tf([2.0, 4.0], [0.2, 3.0, 0.3, 6.0])
  // [A, B, C, D] = tf2ss(h.num{:}, h.den{:})
  ASSERT_DOUBLE_EQ(A(0, 0), -15);
  ASSERT_DOUBLE_EQ(A(0, 1), -1.5);
  ASSERT_DOUBLE_EQ(A(0, 2), -30.0);
  ASSERT_DOUBLE_EQ(A(1, 0), 1.0);
  ASSERT_DOUBLE_EQ(A(1, 1), 0.0);
  ASSERT_DOUBLE_EQ(A(1, 2), 0.0);
  ASSERT_DOUBLE_EQ(A(2, 0), 0.0);
  ASSERT_DOUBLE_EQ(A(2, 1), 1.0);
  ASSERT_DOUBLE_EQ(A(2, 2), 0.0);
  ASSERT_DOUBLE_EQ(B(0), 1.0);
  ASSERT_DOUBLE_EQ(C(0), 0.0);
  ASSERT_DOUBLE_EQ(C(1), 10.0);
  ASSERT_DOUBLE_EQ(C(2), 20.0);
  ASSERT_DOUBLE_EQ(D(0), 0.0);
}

TEST(testTransferFunctionBase, withDelay) {
  Eigen::VectorXd num, den;
  num.resize(2);
  den.resize(2);
  num << 0.01, 2.0;
  den << 0.2, 3.0;
  double delay = 0.25;
  TransferFunctionBase tf1(num, den, delay, false);
  Eigen::MatrixXd A, B, C, D;
  tf1.getStateSpace(A, B, C, D);

  // Matlab
  // h = tf([0.01, 2.0], [0.2, 3.0])*tf([-0.5*0.25, 1], [0.5*0.25, 1]);
  // [A, B, C, D] = tf2ss(h.num{:}, h.den{:})
  ASSERT_DOUBLE_EQ(A(0, 0), -23.0);
  ASSERT_DOUBLE_EQ(A(0, 1), -120.0);
  ASSERT_DOUBLE_EQ(A(1, 0), 1.0);
  ASSERT_DOUBLE_EQ(A(1, 1), 0.0);
  ASSERT_DOUBLE_EQ(B(0), 1.0);
  ASSERT_DOUBLE_EQ(C(0), -8.45);
  ASSERT_DOUBLE_EQ(C(1), 86.0);
  ASSERT_DOUBLE_EQ(D(0), -0.05);
}
