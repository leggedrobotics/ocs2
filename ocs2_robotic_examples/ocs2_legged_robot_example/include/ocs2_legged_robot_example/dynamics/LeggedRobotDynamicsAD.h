#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_legged_robot_example/common/definitions.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>

namespace ocs2 {
namespace legged_robot {

class LeggedRobotDynamicsAD final : public SystemDynamicsBase {
 public:
  LeggedRobotDynamicsAD(const PinocchioInterface& pinocchioInterface, CentroidalModelPinocchioMapping<ad_scalar_t>& mapping,
                        const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                        bool verbose = true);
  ~LeggedRobotDynamicsAD() override = default;
  LeggedRobotDynamicsAD* clone() const override { return new LeggedRobotDynamicsAD(*this); }

  vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input) override;

  VectorFunctionLinearApproximation linearApproximation(scalar_t time, const vector_t& state, const vector_t& input) override;

 private:
  LeggedRobotDynamicsAD(const LeggedRobotDynamicsAD& rhs) = default;

  PinocchioCentroidalDynamicsAD pinocchioCentroidalDynamicsAd_;
};

}  // namespace legged_robot
}  // namespace ocs2
