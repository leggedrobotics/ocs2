#pragma once

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_double_slit_example/definitions.h"

namespace ocs2 {
namespace double_slit {

/**
 * This class implements MRT (Model Reference Tracking) communication interface using ROS.
 *
 * @tparam double_slit::STATE_DIM_: Dimension of the state space.
 * @tparam double_slit::INPUT_DIM_: Dimension of the control input space.
 */
class MRT_ROS_Double_Slit : public MRT_ROS_Interface<double_slit::STATE_DIM_, double_slit::INPUT_DIM_>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef MRT_ROS_Interface<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> BASE;

	typedef Dimensions<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t       scalar_t;
	typedef typename DIMENSIONS::scalar_array_t	scalar_array_t;
	typedef typename DIMENSIONS::size_array_t   size_array_t;
	typedef typename DIMENSIONS::state_vector_t       state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::input_vector_t       input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
	typedef typename DIMENSIONS::input_state_matrix_t       input_state_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_array_t input_state_matrix_array_t;

	typedef SystemObservation<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> system_observation_t;

	typedef RosMsgConversions<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> ros_msg_conversions_t;

	typedef LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > state_linear_interpolation_t;
	typedef LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > input_linear_interpolation_t;
	typedef LinearInterpolation<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t>> gain_linear_interpolation_t;

	/**
	 * Default constructor
	 */
	MRT_ROS_Double_Slit() = default;

	/**
	 * Constructor
	 *
	 * @param [in] logicRules: A logic rule class of derived from the hybrid logicRules base.
	 * @param [in] useFeedforwardPolicy: Whether to receive the MPC feedforward (true) or MPC feedback policy (false).
	 * @param [in] robotName: The robot's name.
	 */
	MRT_ROS_Double_Slit(
			const std::string& robotName = "robot_mpc")

	: BASE(NullLogicRules(), robotName)
	{}

	/**
	 * Destructor
	 */
	virtual ~MRT_ROS_Double_Slit() = default;

	/**
	 * This method will be called either after the very fist call of the class or after a call to reset().
	 * Users can use this function for any sort of initialization that they may need in the first call.
	 *
	 * @param [in] planObservation: The observation of the policy.
	 */
	virtual void initCall(
			const system_observation_t& planObservation)
	{}

private:

};

} // namespace double_slit
} // namespace ocs2

