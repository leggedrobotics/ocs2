// ocs2
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h>
#include <ocs2_double_integrator_noros_example/DoubleIntegratorInterface.h>

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorPyBindings final {
 public:
  using dim_t = ocs2::Dimensions<ocs2::double_integrator::double_integrator_dims::STATE_DIM_,
                                 ocs2::double_integrator::double_integrator_dims::INPUT_DIM_>;
  using mpc_t = ocs2::MPC_Interface<dim_t::STATE_DIM_, dim_t::INPUT_DIM_>;
  using state_vector_t = dim_t::state_vector_t;
  using input_vector_t = dim_t::input_vector_t;
  using scalar_array_t = dim_t::scalar_array_t;
  using state_vector_array_t = dim_t::state_vector_array_t;
  using input_vector_array_t = dim_t::input_vector_array_t;
  using state_matrix_t = dim_t::state_matrix_t;
  using state_input_matrix_t = dim_t::state_input_matrix_t;

  DoubleIntegratorPyBindings(const std::string& taskFileFolder);

  void setObservation(double t, Eigen::Ref<const state_vector_t> x);

  void advanceMpc();

  void getMpcSolution(scalar_array_t& t, state_vector_array_t& x, input_vector_array_t& u, state_vector_array_t& Vx);

  state_vector_t computeFlowMap(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  void setFlowMapDerivativeStateAndControl(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  state_matrix_t computeFlowMapDerivativeState();

  state_input_matrix_t computeFlowMapDerivativeInput();

 protected:
  std::unique_ptr<ocs2::double_integrator::DoubleIntegratorInterface> doubleIntegratorInterface_;
  std::unique_ptr<mpc_t> mpcInterface_;
  ocs2::NullLogicRules nullLogicRules_;

  std::unique_ptr<DoubleIntegratorDynamics> dynamics_;
  std::unique_ptr<DoubleIntegratorDynamicsDerivatives> dynamicsDerivatives_;
};

}  // namespace double_integrator
}  // namespace ocs2
