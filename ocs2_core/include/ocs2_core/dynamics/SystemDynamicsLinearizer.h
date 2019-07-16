/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef SYSTEMDYNAMICSLINEARIZER_OCS2_H_
#define SYSTEMDYNAMICSLINEARIZER_OCS2_H_

#include <cmath>
#include <algorithm>
#include <memory>
#include <Eigen/Dense>

#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include "ocs2_core/dynamics/DerivativesBase.h"

namespace ocs2{

/**
 * A class for linearizing system dynamics. The linearized system dynamics is defined as: \n
 *
 * - Linearized system:                        \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class SystemDynamicsLinearizer : public DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  	using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>;
  	using Base = DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>;
	using typename Base::scalar_t;
	using typename Base::state_vector_t;
	using typename Base::state_matrix_t;
	using typename Base::input_vector_t;
	using typename Base::state_input_matrix_t;

	/**
	 * Constructor
	 */
	explicit SystemDynamicsLinearizer(const std::shared_ptr<controlled_system_base_t>& nonlinearSystemPtr_,
			bool doubleSidedDerivative=true, bool isSecondOrderSystem=false)
	: nonlinearSystemPtr_(nonlinearSystemPtr_),
	  doubleSidedDerivative_(doubleSidedDerivative),
	  isSecondOrderSystem_(isSecondOrderSystem)
	{}

	/**
	 * Copy constructor
	 *
	 * @param [in] other: Instance of the other class.
	 */
	SystemDynamicsLinearizer(const SystemDynamicsLinearizer& other)

	: nonlinearSystemPtr_(other.nonlinearSystemPtr_->clone()),
	  doubleSidedDerivative_(other.doubleSidedDerivative_),
	  isSecondOrderSystem_(other.isSecondOrderSystem_)
	{}

	/**
	 * operator=
	 *
	 * @param [in] other: Instance of the other class.
	 * @return SystemDynamicsLinearizer&:
	 */
	SystemDynamicsLinearizer& operator=(const SystemDynamicsLinearizer&other)  {
		nonlinearSystemPtr_ = typename controlled_system_base_t::Ptr(other.nonlinearSystemPtr_->clone());
		doubleSidedDerivative_ = other.doubleSidedDerivative_;
		isSecondOrderSystem_ = other.isSecondOrderSystem_;
	}

	/**
	 * Default destructor
	 */
	virtual ~SystemDynamicsLinearizer()= default;

	/**
	 * Sets the current time, state, and control inout.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [in] u: Current input.
	 */
	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override  {

		Base::setCurrentStateAndControl(t, x, u);

		if (!doubleSidedDerivative_) {
			nonlinearSystemPtr_->computeFlowMap(Base::t_, Base::x_, Base::u_, f_);
		}
	}

	/**
	 * The A matrix at a given operating point for the linearized system,
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] A: \f$ A(t) \f$ matrix.
	 */
	void getDerivativeState(state_matrix_t& A) override {

		for (size_t i=0; i<STATE_DIM; i++)  {

			// inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
			double h = eps_ * std::max(fabs(Base::x_(i)), 1.0);

			state_vector_t xPlusPerturbed = Base::x_;
			xPlusPerturbed(i) += h;

			// get evaluation of f(x,u)
			state_vector_t fPlusPerturbed;
			nonlinearSystemPtr_->computeFlowMap(Base::t_, xPlusPerturbed, Base::u_, fPlusPerturbed);

			if (doubleSidedDerivative_)  {

				state_vector_t xMinusPerturbed = Base::x_;
				xMinusPerturbed(i) -= h;

				state_vector_t fMinusPerturbed;
				nonlinearSystemPtr_->computeFlowMap(Base::t_, xMinusPerturbed, Base::u_, fMinusPerturbed);

				if(isSecondOrderSystem_)  {
					A.template topLeftCorner<STATE_DIM/2, STATE_DIM/2>().setZero();
					A.template topRightCorner<STATE_DIM/2, STATE_DIM/2>().setIdentity();
					A.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - fMinusPerturbed.template tail<STATE_DIM/2>()) / (2.0*h);
				}
				else {
					A.col(i) = (fPlusPerturbed - fMinusPerturbed) / (2.0*h);
				}
			}
			else  {
				if(isSecondOrderSystem_)  {
					A.template topLeftCorner<STATE_DIM/2, STATE_DIM/2>().setZero();
					A.template topRightCorner<STATE_DIM/2, STATE_DIM/2>().setIdentity();
					A.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - f_.template tail<STATE_DIM/2>()) / h;
				}
				else {
					A.col(i) = (fPlusPerturbed - f_) / h;
				}
			}
		}  // end of i loop

	}

	/**
	 * The B matrix at a given operating point for the linearized system,
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] B: \f$ B(t) \f$ matrix.
	 */
	void getDerivativesControl(state_input_matrix_t& B) override  {

		Eigen::Matrix<double, STATE_DIM, INPUT_DIM> tempB;

		for (size_t i=0; i<INPUT_DIM; i++) {

			//			std::cin.get();

			// inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
			double h = eps_ * std::max(fabs(Base::u_(i)), 1.0);

			input_vector_t uPlusPerturbed = Base::u_;
			uPlusPerturbed(i) += h;

			// get evaluation of f(x,u)
			state_vector_t fPlusPerturbed;
			nonlinearSystemPtr_->computeFlowMap(Base::t_, Base::x_, uPlusPerturbed, fPlusPerturbed);

			if (doubleSidedDerivative_)  {

				input_vector_t uMinusPerturbed = Base::u_;
				uMinusPerturbed(i) -= h;

				state_vector_t fMinusPerturbed;
				nonlinearSystemPtr_->computeFlowMap(Base::t_, Base::x_, uMinusPerturbed, fMinusPerturbed);

				if(isSecondOrderSystem_)  {
					tempB.template topRows<STATE_DIM/2>().setZero();
					tempB.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - fMinusPerturbed.template tail<STATE_DIM/2>()) / (2.0*h);
				}
				else {
					tempB.col(i) = (fPlusPerturbed - fMinusPerturbed) / (2.0*h);

					//					std::cout << ">>>> The " << i << " element out of " << INPUT_DIM << std::endl;
					//					std::cout << "u+ :" << uPlusPerturbed.transpose() << std::endl;
					//					std::cout << "f+ :" << fPlusPerturbed.transpose() << std::endl;
					//					std::cout << "u- :" << uMinusPerturbed.transpose() << std::endl;
					//					std::cout << "f- :" << fMinusPerturbed.transpose() << std::endl << std::endl;
				}
			}
			else {
				if(isSecondOrderSystem_)  {
					tempB.template topRows<STATE_DIM/2>().setZero();
					tempB.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - f_.template tail<STATE_DIM/2>()) / h;
				}
				else {
					tempB.col(i) = (fPlusPerturbed - f_) / h;
				}
			}
		}  // end of i loop

		B = nonlinearSystemPtr_->computeOutputStateDerivative(Base::t_, Base::x_, Base::u_) * tempB;
	}

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	SystemDynamicsLinearizer<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const override {
		return new SystemDynamicsLinearizer<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
	}


private:
	const double eps_= sqrt(Eigen::NumTraits<double>::epsilon());

	typename controlled_system_base_t::Ptr nonlinearSystemPtr_;
	bool doubleSidedDerivative_;
	bool isSecondOrderSystem_;

	state_vector_t f_;

};

} // namespace ocs2

#endif /* SYSTEMDYNAMICSLINEARIZER_H_ */
