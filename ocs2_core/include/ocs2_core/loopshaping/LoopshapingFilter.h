//
// Created by rgrandia on 07.05.19.
//

#ifndef OCS2_LOOPSHAPINGFILTER_H
#define OCS2_LOOPSHAPINGFILTER_H

#include <vector>
#include <Eigen/Dense>
#include <ocs2_core/dynamics/TransferFunctionBase.h>

namespace ocs2 {

class Filter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Filter() {
        A_.resize(0,0);
        B_.resize(0,0);
        C_.resize(0,0);
        D_.resize(0,0);
    };

    Filter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D) :
    A_(A), B_(B), C_(C), D_(D), numStates_(A.rows()), numInputs_(B.cols()), numOutputs_(C.rows())
    {
        checkSize();
    }

    size_t getNumStates() const {return numStates_;};
    size_t getNumInputs() const {return numInputs_;};
    size_t getNumOutputs() const {return numOutputs_;};

    const Eigen::MatrixXd& getA() const {return A_;};
    const Eigen::MatrixXd& getB() const {return B_;};
    const Eigen::MatrixXd& getC() const {return C_;};
    const Eigen::MatrixXd& getD() const {return D_;};

    void print() const {
        std::cout << "numStates: " << numStates_ << std::endl;
        std::cout << "numInputs: " << numInputs_ << std::endl;
        std::cout << "numOutputs: " << numOutputs_ << std::endl;
        std::cout << "A: \n" << A_  << std::endl;
        std::cout << "B: \n" << B_  << std::endl;
        std::cout << "C: \n" << C_  << std::endl;
        std::cout << "D: \n" << D_  << std::endl;
    }

    template <typename DerivedOutput, typename DerivedState, typename DerivedInput>
    void findEquilibriumForOutput(const DerivedOutput& y, DerivedState& x, DerivedInput& u) const {
        // Solve (given y)
        // [0  =  [  A    B    [x
        //  y]       C    D  ]  u]
        Eigen::MatrixXd ABCD(numStates_ + numInputs_, numStates_ + numInputs_);
        ABCD << A_, B_,
                C_, D_;

        Eigen::VectorXd x_u;
        Eigen::VectorXd zero_y(numStates_ + numOutputs_);
        zero_y << Eigen::VectorXd::Zero(numStates_), y;

        x_u = ABCD.colPivHouseholderQr().solve(zero_y);

        x = x_u.segment(0, numStates_);
        u = x_u.segment(numStates_, numInputs_);
    }

    template <typename DerivedInput, typename DerivedState, typename DerivedOutput>
    void findEquilibriumForInput(const DerivedInput& u, DerivedState& x, DerivedOutput& y) const {
        // Solve (given u)
        // [0  =  [  A    B    [x
        //  y]       C    D  ]  u]
        x = - A_.colPivHouseholderQr().solve(B_ * u);
        y = C_ * x + D_ * u;
    }

private:
    void checkSize() const {
        bool correct = true;
        // check number of state
        correct &= numStates_  == A_.rows();
        correct &= numStates_  == B_.rows();
        correct &= numStates_  == C_.cols();
        // Check number of inputs
        correct &= numInputs_  == B_.cols();
        correct &= numInputs_  == D_.cols();
        // Check number of outputs
        correct &= numOutputs_ == C_.rows();
        correct &= numOutputs_ == D_.rows();
        if (!correct){
            print();
            throw std::runtime_error("Loopshaping: Filer: Matrix dimensions not consistent.");
        }
    }

    Eigen::MatrixXd A_, B_, C_, D_;
    size_t numStates_ = 0;
    size_t numInputs_ = 0;
    size_t numOutputs_ = 0;
};


} // namespace ocs2

#endif //OCS2_LOOPSHAPINGFILTER_H
