/*!
 * BallbotCostAD.h
 *
 *  Created on: Nov 10, 2018
 *      Author: Maria Vittoria Minniti
 */

#ifndef BALLBOTCOSTAD_OCS2_BALLBOT_H_
#define BALLBOTCOSTAD_OCS2_BALLBOT_H_

#include <ocs2_core/cost/CostFunctionBaseAD.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>
#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_ballbot_example/generated/transforms.h>
#include <ocs2_ballbot_example/generated/jacobians.h>

namespace ocs2{
    namespace ballbot{

        class BallbotCostAD : public ocs2::CostFunctionBaseAD<BallbotCostAD, ballbot::STATE_DIM_, ballbot::INPUT_DIM_>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW


            typedef std::shared_ptr<BallbotCostAD> Ptr;
            typedef std::shared_ptr<const BallbotCostAD> ConstPtr;

            typedef CostFunctionBaseAD<BallbotCostAD, ballbot::STATE_DIM_, ballbot::INPUT_DIM_> BASE;
            typedef typename BASE::scalar_t scalar_t;
            typedef typename BASE::state_vector_t state_vector_t;
            typedef typename BASE::state_matrix_t state_matrix_t;
            typedef typename BASE::input_vector_t input_vector_t;
            typedef typename BASE::input_matrix_t input_matrix_t;


            /**
             * Constructor for the running and final cost function defined as the following:
             * - \f$ L = 0.5(x-x_{n})' Q (x-x_{n}) + 0.5(u-u_{n})' R (u-u_{n}) + (u-u_{n})' P (x-x_{n}) \f$
             * - \f$ \Phi = 0.5(x-x_{f})' Q_{f} (x-x_{f}) \f$.
             * @param [in] Q: \f$ Q \f$
             * @param [in] R: \f$ R \f$
             * @param [in] xNominalIntermediate: \f$ x_{n}\f$
             * @param [in] uNominalIntermediate: \f$ u_{n}\f$
             * @param [in] xNominalFinal: \f$ x_{f}\f$
             * @param [in] QFinal: \f$ Q_{f}\f$
             */
            BallbotCostAD(
                    const state_matrix_t& Q,
                    const input_matrix_t& R,
                    const state_vector_t& x_nominal,
                    const input_vector_t& u_nominal,
                    const state_matrix_t& Q_final,
                    const state_vector_t& x_final,
                    const bool &dynamicLibraryIsCompiled = false)
                    : BASE(dynamicLibraryIsCompiled)
                    , Q_(Q)
                    , R_(R)
                    , x_nominal_(x_nominal)
                    , u_nominal_(u_nominal)
                    , Q_final_(Q_final)
                    , x_final_(x_final)
            {}

            ~BallbotCostAD() = default;


            /**
             * Interface method to the intermediate cost function. This method should be implemented by the derived class.
             *
             * @tparam scalar type. All the floating point operations should be with this type.
             * @param [in] time: time.
             * @param [in] state: state vector.
             * @param [in] input: input vector.
             * @param [in] stateDesired: desired state vector.
             * @param [in] inputDesired: desired input vector.
             * @param [in] logicVariable: logic variable vector.
             * @param [out] costValue: cost value.
             */
            template <typename SCALAR_T>
            void intermediateCostFunction(
                    const SCALAR_T &time,
                    const Eigen::Matrix<SCALAR_T, ballbot::STATE_DIM_, 1> &state,
                    const Eigen::Matrix<SCALAR_T, ballbot::INPUT_DIM_, 1> &input,
                    const Eigen::Matrix<SCALAR_T, ballbot::STATE_DIM_, 1> &stateDesired,
                    const Eigen::Matrix<SCALAR_T, ballbot::INPUT_DIM_, 1> &inputDesired,
                    const Eigen::Matrix<SCALAR_T, BASE::logic_variable_dim_, 1> &logicVariable,
                    SCALAR_T &costValue)
            {
                Eigen::Matrix<SCALAR_T, ballbot::JOINTS_DOF_NUM_, 1> jointState = state. template head<ballbot::JOINTS_DOF_NUM_>();
                // compute position
                typedef typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait trait_t;
                typename iit::Ballbot::tpl::HomogeneousTransforms<trait_t>::Type_fr_base0_X_fr_control homogeneousTransformWorldToControlFrame;
                homogeneousTransformWorldToControlFrame = homogeneousTransformWorldToControlFrame.update(jointState);
                Eigen::Matrix<SCALAR_T, 3, 1> positionWorldToControlFrameInWorldFrame = homogeneousTransformWorldToControlFrame.template topRightCorner<3, 1>();

                // compute velocity
                typename iit::Ballbot::tpl::Jacobians<trait_t>::Type_fr_base0_J_fr_control jacobianWorldToControlFrameInWorldFrame;
                jacobianWorldToControlFrameInWorldFrame = jacobianWorldToControlFrameInWorldFrame.update(jointState);
                Eigen::Matrix<SCALAR_T, 3, 1> linearVelocityWorldToControlFrameInWorldFrame = jacobianWorldToControlFrameInWorldFrame.template bottomRows<3>()*state. template segment<5>(5);

                // compute deviation between desired and measured state
                Eigen::Matrix<SCALAR_T, ballbot::STATE_DIM_, 1> stateDeviation = stateDesired - state;
                stateDeviation. template head<2>() = stateDesired.template head<2>() - positionWorldToControlFrameInWorldFrame. template head<2>();
                stateDeviation. template segment<2>(5) = stateDesired.template segment<2>(5) - linearVelocityWorldToControlFrameInWorldFrame. template head<2>();

                // compute value of the cost function
                costValue = 0.5 * stateDeviation.dot(Q_.template cast<SCALAR_T>() * stateDeviation) +
                            0.5 * input.dot(R_.template cast<SCALAR_T>() * input);
            }

            /**
             * Interface method to the terminal cost function. This method should be implemented by the derived class.
             *
             * @tparam scalar type. All the floating point operations should be with this type.
             * @param [in] time: time.
             * @param [in] state: state vector.
             * @param [in] stateDesired: desired state vector.
             * @param [in] logicVariable: logic variable vector.
             * @param [out] costValue: cost value.
             */
            template <typename SCALAR_T>
            void terminalCostFunction(
                    const SCALAR_T &time,
                    const Eigen::Matrix<SCALAR_T, ocs2::ballbot::STATE_DIM_, 1> &state,
                    const Eigen::Matrix<SCALAR_T, ocs2::ballbot::STATE_DIM_, 1> &stateDesired,
                    const Eigen::Matrix<SCALAR_T, BASE::logic_variable_dim_, 1> &logicVariable,
                    SCALAR_T &costValue)
            {

                Eigen::Matrix<SCALAR_T, 3, 1> eulerAnglesZyx = Eigen::Matrix<SCALAR_T, 3, 1>(state(2), state(3), state(4));
                const Eigen::Quaternion<SCALAR_T> quaternionBaseToWorld = getQuaternionFromEulerAnglesZyx(eulerAnglesZyx);
                Eigen::Quaternion<SCALAR_T> desiredQuaternionBaseToWorld;
                desiredQuaternionBaseToWorld.w() = SCALAR_T(1.0); desiredQuaternionBaseToWorld.vec().setZero();

                Eigen::Matrix<SCALAR_T, ballbot::STATE_DIM_, 1> stateDeviation = stateDesired - state;
                stateDeviation.template segment<3>(2) = quaternionDistance(quaternionBaseToWorld, desiredQuaternionBaseToWorld);

                // compute value of the cost function
                costValue = 0.5 * stateDeviation.dot(Q_final_.template cast<SCALAR_T>() * stateDeviation);
            }

            /**
             * Gets a user-defined logic variable based on the given logic rules.
             *
             * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
             * method findActiveSubsystemHandle returns a Lambda expression which can be used to
             * find the ID of the current active subsystem.
             * @param [in] partitionIndex: index of the time partition.
             * @param [in] algorithmName: The algorithm that class this class (default not defined).
             * @return The the logic variables.
              */
            logic_variable_t getlogicVariables(scalar_t time) {

                return logic_variable_t::Zero();
            }

            /**
             * Gets a user-defined desired state at the given time.
             *
             * @param [in] t: Current time.
             * @return The desired state at the given time.
             */
            state_vector_t getDesiredState(const scalar_t &t)
            {

                dynamic_vector_t xNominal;
                BASE::xNominalFunc_.interpolate(t, xNominal);

                if (xNominal.size() > ocs2::ballbot::STATE_DIM_)
                    throw std::runtime_error("Desired state size cannot be greater than state dimension.");
                else
                {
                    state_vector_t desiredState = state_vector_t::Zero();
                    desiredState.head(xNominal.size()) = xNominal;
                    return desiredState;
                }
            }

            /**
             * Gets a user-defined desired input at the given time.
             *
             * @param [in] t: Current time.
             * @return The desired input at the given time.
             */
            input_vector_t getDesiredInput(const scalar_t &t)
            {

                dynamic_vector_t uNominal;
                BASE::uNominalFunc_.interpolate(t, uNominal);

                if (uNominal.size() > ocs2::ballbot::INPUT_DIM_)
                    throw std::runtime_error("Desired input size cannot be greater than input dimension.");
                else
                {
                    input_vector_t desiredInput = input_vector_t::Zero();
                    desiredInput.head(uNominal.size()) = uNominal;
                    return desiredInput;
                }
            }
            /**
             * Compute the quaternion distance measure
             *
             * @param [in] eeQuaternion: measured end effector quaternion.
             * @param [in] commandQuaternion: desired end effector quaternion.
             * @return The desired input at the given time.
             */
            template <typename SCALAR_T>
            Eigen::Matrix<SCALAR_T, 3, 1> quaternionDistance(
                    const Eigen::Quaternion<SCALAR_T>& eeQuaternion,
                    const Eigen::Quaternion<SCALAR_T>& commandQuaternion) const {


                return eeQuaternion.w() * commandQuaternion.vec() - commandQuaternion.w() * eeQuaternion.vec() - commandQuaternion.vec().cross(eeQuaternion.vec());
            }

            /**
             * Compute the quaternion corresponding to euler angles zyx
             *
             * @param [in] eulerAnglesZyx
             * @return The corresponding quaternion
             */
            template <typename SCALAR_T>
            Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) const
            {
                SCALAR_T yaw = eulerAnglesZyx(0); SCALAR_T pitch = eulerAnglesZyx(1); SCALAR_T roll = eulerAnglesZyx(2);
                // Abbreviations for the various angular functions
                SCALAR_T cy = cos(yaw * 0.5);
                SCALAR_T sy = sin(yaw * 0.5);
                SCALAR_T cp = cos(pitch * 0.5);
                SCALAR_T sp = sin(pitch * 0.5);
                SCALAR_T cr = cos(roll * 0.5);
                SCALAR_T sr = sin(roll * 0.5);

                Eigen::Quaternion<SCALAR_T> quaternion;
                quaternion.w() = cy * cp * cr + sy * sp * sr;
                quaternion.x() = cy * cp * sr - sy * sp * cr;
                quaternion.y() = sy * cp * sr + cy * sp * cr;
                quaternion.z() = sy * cp * cr - cy * sp * sr;

                return quaternion;
            }


        private:
            state_matrix_t Q_;
            input_matrix_t  R_;
            state_vector_t x_nominal_;
            input_vector_t u_nominal_;
            state_matrix_t Q_final_;
            state_vector_t x_final_;

        };

    } //namespace ballbot
} //namespace ocs2



#endif /* BALLBOTCOSTAD_OCS2_BALLBOT_H_ */