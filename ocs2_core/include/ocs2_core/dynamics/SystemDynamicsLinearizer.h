/*
 * SystemDynamicsLinearizer.h
 *
 * Created on: Jan 3, 2016
 *     Author: farbod
 */

#ifndef SYSTEMDYNAMICSLINEARIZER_OCS2_H_
#define SYSTEMDYNAMICSLINEARIZER_OCS2_H_

#include <cmath>
#include <algorithm>
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
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SystemDynamicsLinearizer : public DerivativesBase<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DerivativesBase<STATE_DIM, INPUT_DIM> Base;
	typedef typename Base::scalar_t scalar_t;
	typedef typename Base::state_vector_t state_vector_t;
	typedef typename Base::state_matrix_t state_matrix_t;
	typedef typename Base::control_vector_t control_vector_t;
	typedef typename Base::control_gain_matrix_t control_gain_matrix_t;

	/**
	 * Constructor
	 */
	SystemDynamicsLinearizer(const std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> >& nonlinearSystemPtr_,
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
		nonlinearSystemPtr_ = other.nonlinearSystemPtr_->clone();
		doubleSidedDerivative_ = other.doubleSidedDerivative_;
		isSecondOrderSystem_ = other.isSecondOrderSystem_;
	}

	/**
	 * Default destructor
	 */
	virtual ~SystemDynamicsLinearizer(){}

	/**
	 * Sets the current time, state, and control inout.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [in] u: Current input.
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) override  {

		Base::setCurrentStateAndControl(t, x, u);

		if (doubleSidedDerivative_==false)
			nonlinearSystemPtr_->computeDerivative(Base::t_, Base::x_, Base::u_, f_);
	}

	/**
	 * The A matrix at a given operating point for the linearized system,
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] A: \f$ A(t) \f$ matrix.
	 */
	virtual void getDerivativeState(state_matrix_t& A) override {

		Eigen::Matrix<double, STATE_DIM, STATE_DIM> A;

		for (size_t i=0; i<STATE_DIM; i++)  {

			// inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
			double h = eps_ * std::max(fabs(Base::x_(i)), 1.0);

			state_vector_t xPlusPerturbed = Base::x_;
			xPlusPerturbed(i) += h;

			// get evaluation of f(x,u)
			state_vector_t fPlusPerturbed;
			nonlinearSystemPtr_->computeDerivative(Base::t_, xPlusPerturbed, Base::u_, fPlusPerturbed);

			if (doubleSidedDerivative_)  {

				state_vector_t xMinusPerturbed = Base::x_;
				xMinusPerturbed(i) -= h;

				state_vector_t fMinusPerturbed;
				nonlinearSystemPtr_->computeDerivative(Base::t_, xMinusPerturbed, Base::u_, fMinusPerturbed);

				if(isSecondOrderSystem_)  {
					A.template topLeftCorner<STATE_DIM/2, STATE_DIM/2>().setZero();
					A.template topRightCorner<STATE_DIM/2, STATE_DIM/2>().setIdentity();
					A.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - fMinusPerturbed.template tail<STATE_DIM/2>()) / (2.0*h);
				}
				else
					A.col(i) = (fPlusPerturbed - fMinusPerturbed) / (2.0*h);
			}
			else  {
				if(isSecondOrderSystem_)  {
					A.template topLeftCorner<STATE_DIM/2, STATE_DIM/2>().setZero();
					A.template topRightCorner<STATE_DIM/2, STATE_DIM/2>().setIdentity();
					A.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - f_.template tail<STATE_DIM/2>()) / h;
				}
				else
					A.col(i) = (fPlusPerturbed - f_) / h;
			}
		}  // end of i loop

	}

	/**
	 * The B matrix at a given operating point for the linearized system,
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] B: \f$ B(t) \f$ matrix.
	 */
	virtual void getDerivativesControl(control_gain_matrix_t& B) override  {

		Eigen::Matrix<double, STATE_DIM, INPUT_DIM> tempB;

		for (size_t i=0; i<INPUT_DIM; i++) {

			//			std::cin.get();

			// inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
			double h = eps_ * std::max(fabs(Base::u_(i)), 1.0);

			control_vector_t uPlusPerturbed = Base::u_;
			uPlusPerturbed(i) += h;

			// get evaluation of f(x,u)
			state_vector_t fPlusPerturbed;
			nonlinearSystemPtr_->computeDerivative(Base::t_, Base::x_, uPlusPerturbed, fPlusPerturbed);

			if (doubleSidedDerivative_)  {

				control_vector_t uMinusPerturbed = Base::u_;
				uMinusPerturbed(i) -= h;

				state_vector_t fMinusPerturbed;
				nonlinearSystemPtr_->computeDerivative(Base::t_, Base::x_, uMinusPerturbed, fMinusPerturbed);

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
				else
					tempB.col(i) = (fPlusPerturbed - f_) / h;
			}
		}  // end of i loop

		B = nonlinearSystemPtr_->computeOutputStateDerivative(Base::t_, Base::x_, Base::u_) * tempB;
	}

	/**
	 * Returns pointer to DerivativesBase class.
	 *
	 * @return Base*: a shared_ptr pointer.
	 */
	std::shared_ptr<Base> clone() const  {
		typedef SystemDynamicsLinearizer<STATE_DIM, INPUT_DIM> system_lineaizer_t;
		return std::allocate_shared<system_lineaizer_t, Eigen::aligned_allocator<system_lineaizer_t>>(
				Eigen::aligned_allocator<system_lineaizer_t>(),*this );
	}


private:
	const double eps_= sqrt(Eigen::NumTraits<double>::epsilon());

	std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > nonlinearSystemPtr_;
	bool doubleSidedDerivative_;
	bool isSecondOrderSystem_;

	state_vector_t f_;

};

} // namespace ocs2

#endif /* SYSTEMDYNAMICSLINEARIZER_H_ */
