//
// Created by rgrandia on 14.06.19.
//

#include <gtest/gtest.h>
#include <Eigen/Dense>

TEST(QRhelpers, somestuff)
{
  // Some matrix
  const size_t m = 5;
  const size_t n = 2;
  Eigen::MatrixXd A(5, 2);
  A.setRandom();
  A.block(0, 0, n, n).setIdentity(); // Makes sure columns are independent

  Eigen::HouseholderQR<Eigen::MatrixXd> QRofA( A );
  Eigen::MatrixXd Q = QRofA.householderQ();
  Eigen::MatrixXd Qu = Q.rightCols(m - n);
  Eigen::MatrixXd Qc = Q.leftCols(n);
  Eigen::MatrixXd Rc = QRofA.matrixQR().topLeftCorner(n, n).template triangularView<Eigen::Upper>();

  std::cout << "A: \n" << A << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "Rc: \n" << Rc << std::endl;

  // Fast inverse of Rc
  Eigen::MatrixXd Rc_inv = Eigen::MatrixXd::Identity(n, n);
  Rc.template triangularView<Eigen::Upper>().solveInPlace(Rc_inv);

  std::cout << "Rc: \n" << Rc << std::endl;
  std::cout << "Rc_inv: \n" << Rc.inverse() << std::endl;
  std::cout << "Rc_fast_inv: \n" << Rc_inv << std::endl;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}