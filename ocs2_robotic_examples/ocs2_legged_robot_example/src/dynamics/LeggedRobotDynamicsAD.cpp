#include <ocs2_legged_robot_example/dynamics/LeggedRobotDynamicsAD.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotDynamicsAD::LeggedRobotDynamicsAD(const PinocchioInterface& pinocchioInterface,
                                             CentroidalModelPinocchioMapping<ad_scalar_t>& mapping, const std::string& modelName,
                                             const std::string& modelFolder /*= "/tmp/ocs2"*/, bool recompileLibraries /*= true*/,
                                             bool verbose /*= true*/)
    : Base(), pinocchioCentroidalDynamicsAd_(pinocchioInterface, mapping, modelName, modelFolder, recompileLibraries, verbose) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotDynamicsAD::LeggedRobotDynamicsAD(const LeggedRobotDynamicsAD& rhs) : Base(rhs) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LeggedRobotDynamicsAD::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input) {
  return pinocchioCentroidalDynamicsAd_.getSystemFlowMap(time, state, input);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LeggedRobotDynamicsAD::linearApproximation(scalar_t time, const vector_t& state, const vector_t& input) {
  return pinocchioCentroidalDynamicsAd_.getSystemFlowMapLinearApproximation(time, state, input);
}

}  // namespace legged_robot
}  // namespace ocs2
