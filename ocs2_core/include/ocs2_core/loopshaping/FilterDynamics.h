//
// Created by ruben on 12.07.18.
//

#ifndef OCS2_FILTERDYNAMICS_H
#define OCS2_FILTERDYNAMICS_H

#include <Eigen/Dense>


/*
 *  Class that contains the dynamics of a filter:
 *  \dot{x} = A x + B u
 *  y = C x + D u
 */
class FilterDynamics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using A_t = Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM>;
    using B_t = Eigen::Matrix<SCALAR, STATE_DIM, INPUT_DIM>;
    using C_t = Eigen::Matrix<SCALAR, OUTPUT_DIM, STATE_DIM>;
    using D_t = Eigen::Matrix<SCALAR, OUTPUT_DIM, INPUT_DIM>;

    using state_vector_t = Eigen::Matrix<SCALAR, STATE_DIM, 1>;
    using input_vector_t = Eigen::Matrix<SCALAR, INPUT_DIM, 1>;
    using output_vector_t = Eigen::Matrix<SCALAR, OUTPUT_DIM, 1>;

    FilterDynamics(const A_t &A, const B_t &B, const C_t &C, const D_t &D) : A_(A), B_(B), C_(C), D_(D) {};
    FilterDynamics(const FilterSettings &filterSettings)
    {
        if (filterSettings.state_dim != STATE_DIM) {throw std::runtime_error("FILTERDYNAMICS state size error");};
        if (filterSettings.input_dim != INPUT_DIM) {throw std::runtime_error("FILTERDYNAMICS input size error");};
        if (filterSettings.output_dim != OUTPUT_DIM) {throw std::runtime_error("FILTERDYNAMICS output size error");};
        if (filterSettings.output_dim > filterSettings.state_dim) {throw std::runtime_error("FILTERDYNAMICS can't have more outputs than states");};

        A_ = filterSettings.A;
        B_ = filterSettings.B;
        C_ = filterSettings.C;
        D_ = filterSettings.D;
    };

    void computeFlowMap(const state_vector_t& state, const input_vector_t& input, state_vector_t& stateDerivative){
      stateDerivative = A_*state + B_*input;
    };

    int getNumStates() const {return static_cast<int>(STATE_DIM);};
    int getNumInputs() const {return static_cast<int>(INPUT_DIM);};
    int getNumOutputs() const {return static_cast<int>(OUTPUT_DIM);};

    inline A_t getA() const {return A_;};
    inline B_t getB() const {return B_;};
    inline C_t getC() const {return C_;};
    inline D_t getD() const {return D_;};

    template <typename Derived>
    void getA(Eigen::EigenBase<Derived>& A) const {A = A_;};

    template <typename Derived>
    void getB(Eigen::EigenBase<Derived>& B) const {B = B_;};

    template <typename Derived>
    void getC(Eigen::EigenBase<Derived>& C) const {C = C_;};

    template <typename Derived>
    void getD(Eigen::EigenBase<Derived>& D) const {D = D_;};


private:
    A_t A_;
    B_t B_;
    C_t C_;
    D_t D_;
};

#endif //OCS2_FILTERDYNAMICS_H