//
// Created by rgrandia on 07.05.19.
//

#ifndef OCS2_LOOPSHAPINGMISC_H
#define OCS2_LOOPSHAPINGMISC_H

#include <vector>
#include <Eigen/Dense>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
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

namespace LoopshapingPropertyTree {
    Filter readSISOFilter(const boost::property_tree::ptree& pt, std::string filterName, bool invert = false){
        // Get Sizes
        size_t numRepeats = pt.get<size_t>(filterName + ".numRepeats");
        size_t numPoles = pt.get<size_t>(filterName + ".numPoles");
        size_t numZeros = pt.get<size_t>(filterName + ".numZeros");
        double DCGain = pt.get<double>(filterName + ".DCGain");
        size_t numStates = numRepeats*numPoles;
        size_t numInputs = numRepeats;
        size_t numOutputs = numRepeats;

        // Setup Filter, convention a0*s^n + a1*s^(n-1) + ... + an
        Eigen::VectorXd numerator(numZeros+1);
        numerator.setZero();
        numerator(0) = 1.0;
        for (size_t z = 0; z<numZeros; z++){
            double zero = pt.get<double>(filterName + ".zeros." + "(" +std::to_string(z) + ")");
            numerator.segment(1, z+1) -= zero*numerator.segment(0, z+1).eval();
        }

        Eigen::VectorXd denominator(numPoles+1);
        denominator.setZero();
        denominator(0) = 1.0;
        for (size_t p = 0; p<numPoles; p++){
            double pole = pt.get<double>(filterName + ".poles." + "(" +std::to_string(p) + ")");
            denominator.segment(1, p+1) -= pole*denominator.segment(0, p+1).eval();
        }

        // Scale
        if (DCGain > 0) {
            double currentDCGain = numerator(numZeros) / denominator(numPoles);
            if (currentDCGain < 1e-6 || currentDCGain > 1e6) {
                throw std::runtime_error("Trouble rescaling transfer function, current DCGain: " + std::to_string(currentDCGain));
            }
            double scaling = DCGain / currentDCGain;
            numerator *= scaling;
        }

        if (invert) {
            Eigen::VectorXd temp;
            temp = numerator;
            numerator = denominator;
            denominator = temp;
        }

        // Convert to state space
        Eigen::MatrixXd a, b, c, d;
        ocs2::tf2ss(numerator, denominator, a, b, c, d);

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numStates, numStates);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(numStates, numInputs);
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(numInputs, numStates);
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(numInputs, numInputs);
        size_t statecount = 0;
        for (size_t r = 0; r<numRepeats; r++){
            A.block(statecount, statecount, numPoles, numPoles) = a;
            B.block(statecount, r, numPoles, 1) = b;
            C.block(r, statecount, 1, numPoles) = c;
            D.block(r, r, 1, 1) = d;
            statecount += numPoles;
        }

        return Filter(A, B, C, D);
    }

    Filter readMIMOFilter(const boost::property_tree::ptree& pt, std::string filterName, bool invert = false){
        size_t numFilters = pt.get<size_t>(filterName + ".numFilters");
        Eigen::MatrixXd A(0, 0), B(0, 0), C(0, 0), D(0, 0);
        if (numFilters > 0){
            // Read the sisoFilters
            std::vector<Filter> sisoFilters;
            size_t numStates(0), numInputs(0), numOutputs(0);
            for (size_t i = 0; i<numFilters; ++i){
                // Read filter
                std::string sisoFilterName = filterName + ".Filter" + std::to_string(i);
                sisoFilters.emplace_back(readSISOFilter(pt, sisoFilterName,  invert));

                // Track sizes
                numStates += sisoFilters.back().getNumStates();
                numInputs += sisoFilters.back().getNumInputs();
                numOutputs += sisoFilters.back().getNumOutputs();
            }

            // Concatenate siso matrices into one MIMO filter
            A = Eigen::MatrixXd::Zero(numStates, numStates);
            B = Eigen::MatrixXd::Zero(numStates, numInputs);
            C = Eigen::MatrixXd::Zero(numOutputs, numStates);
            D = Eigen::MatrixXd::Zero(numOutputs, numInputs);
            size_t statecount(0), inputcount(0), outputcount(0);
            for (const auto &filt : sisoFilters) {
                A.block(statecount, statecount, filt.getNumStates(), filt.getNumStates()) = filt.getA();
                B.block(statecount, inputcount, filt.getNumStates(), filt.getNumInputs()) = filt.getB();
                C.block(outputcount, statecount, filt.getNumOutputs(), filt.getNumStates()) = filt.getC();
                D.block(outputcount, inputcount, filt.getNumOutputs(), filt.getNumInputs()) = filt.getD();
                statecount += filt.getNumStates();
                inputcount += filt.getNumInputs();
                outputcount += filt.getNumOutputs();
            }
        }
        return Filter(A, B, C, D);
    }
}

} // namespace ocs2

#endif //OCS2_LOOPSHAPINGMISC_H
