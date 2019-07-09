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
class MrtRosDoubleSlit : public MRT_ROS_Interface<double_slit::STATE_DIM_, double_slit::INPUT_DIM_>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using BASE = MRT_ROS_Interface<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> ;

	using DIMENSIONS = Dimensions<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> ;
	using scalar_t = typename DIMENSIONS::scalar_t       ;
	using scalar_array_t = typename DIMENSIONS::scalar_array_t	;
	using size_array_t = typename DIMENSIONS::size_array_t   ;
	using state_vector_t = typename DIMENSIONS::state_vector_t       ;
	using state_vector_array_t = typename DIMENSIONS::state_vector_array_t ;
	using input_vector_t = typename DIMENSIONS::input_vector_t       ;
	using input_vector_array_t = typename DIMENSIONS::input_vector_array_t ;
	using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t       ;
	using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t ;

	using system_observation_t =  SystemObservation<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> ;

	using ros_msg_conversions_t =  RosMsgConversions<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> ;

	using state_linear_interpolation_t =  LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > ;
	using input_linear_interpolation_t =  LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > ;
	using gain_linear_interpolation_t =  LinearInterpolation<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t>> ;

	/**
	 * Default constructor
	 */
	MrtRosDoubleSlit() = default;

	/**
	 * Constructor
	 *
	 * @param [in] logicRules: A logic rule class of derived from the hybrid logicRules base.
	 * @param [in] useFeedforwardPolicy: Whether to receive the MPC feedforward (true) or MPC feedback policy (false).
	 * @param [in] robotName: The robot's name.
	 */
	explicit MrtRosDoubleSlit(
			const std::string& robotName = "robot_mpc")

	: BASE(NullLogicRules(), robotName)
	{}

	/**
	 * Destructor
	 */
	~MrtRosDoubleSlit() override = default;


	void initCall(
			const system_observation_t& ) override
	{}

private:

};

} // namespace double_slit
} // namespace ocs2
