
// python bindings
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

// ocs2
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h>
#include <ocs2_double_integrator_noros_example/DoubleIntegratorInterface.h>


class DoubleIntegratorPyBindings final {

public:
  using dim_t = ocs2::Dimensions<ocs2::double_integrator::double_integrator_dims::STATE_DIM_, ocs2::double_integrator::double_integrator_dims::INPUT_DIM_>;
  using mpc_t = ocs2::MPC_Interface<dim_t::STATE_DIM_, dim_t::INPUT_DIM_>;
  using state_vector_t = dim_t::state_vector_t;
  using scalar_array_t = dim_t::scalar_array_t;
  using state_vector_array_t = dim_t::state_vector_array_t;
  using input_vector_array_t = dim_t::input_vector_array_t;

  DoubleIntegratorPyBindings(const std::string& taskFileFolder){
    doubleIntegratorInterface_.reset(new ocs2::double_integrator::DoubleIntegratorInterface(taskFileFolder));
    mpcInterface_.reset(new mpc_t(*doubleIntegratorInterface_->getMPCPtr(), nullLogicRules_, true));

    // initialize reference
    constexpr double time = 0.0;
    mpc_t::cost_desired_trajectories_t costDesiredTrajectories;
    costDesiredTrajectories.desiredTimeTrajectory().push_back(time);
    auto goalState = doubleIntegratorInterface_->getXFinal();
    costDesiredTrajectories.desiredStateTrajectory().push_back(goalState);
    auto desiredInput = mpc_t::input_vector_t::Zero();
    costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
    mpcInterface_->setTargetTrajectories(costDesiredTrajectories);
  }

  void setObservation(double t, Eigen::Ref<const state_vector_t> x){
    mpc_t::system_observation_t observation;
    observation.time() = t;
    observation.state() = x;
    mpcInterface_->setCurrentObservation(observation);
  }

  void advanceMpc(){
    mpcInterface_->advanceMpc();
  }

  void getMpcSolution(scalar_array_t& t, state_vector_array_t& x, input_vector_array_t& u){
    // TODO(jcarius): should we interpolate here to expose constant time steps?!
    mpcInterface_->getMpcSolution(t, x, u);
    //TODO(jcarius) also get cost-to-go
  }


protected:
  std::unique_ptr<ocs2::double_integrator::DoubleIntegratorInterface> doubleIntegratorInterface_;
  std::unique_ptr<mpc_t> mpcInterface_;
  ocs2::NullLogicRules nullLogicRules_;

};


using namespace pybind11::literals;
PYBIND11_MODULE(DoubleIntegratorPyBindings, m) {
  pybind11::class_<DoubleIntegratorPyBindings>(m, "mpc_interface")
      .def(pybind11::init<const std::string&>())
      .def("setObservation", &DoubleIntegratorPyBindings::setObservation, "t"_a, "x"_a.noconvert())
      .def("advanceMpc", &DoubleIntegratorPyBindings::advanceMpc)
      .def("getMpcSolution", &DoubleIntegratorPyBindings::getMpcSolution, "t"_a.noconvert(), "x"_a.noconvert(), "u"_a.noconvert());
}
