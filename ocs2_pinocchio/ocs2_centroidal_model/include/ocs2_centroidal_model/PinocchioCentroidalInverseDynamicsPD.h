#pragma once

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"
#include "ocs2_centroidal_model/CentroidalModelRbdConversions.h"

namespace ocs2 {

/**
 * Centroidal inverse dynamics with PD control using Pinocchio:
 * torque = torque_inverse_dynamics + pGains * (qDesired - qMeasured) + dGains * (vDesired - vMeasured)
 * @note 6 DoF contacts are not yet implemented
 */
class PinocchioCentroidalInverseDynamicsPD final {
 public:
  /**
   * Constructor
   * @param [in] pinocchioInterface : The predefined pinocchio interface for the robot.
   * @param [in] centroidalModelInfo : The centroidal model information.
   * @param [in] contactNames3DoF : The names of the 3 DoF contacts.
   */
  explicit PinocchioCentroidalInverseDynamicsPD(PinocchioInterface& pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                                std::vector<std::string> contactNames3DoF);

  /**
   * Default destructor.
   */
  ~PinocchioCentroidalInverseDynamicsPD() = default;

  /**
   * Set the PD gains.
   * @param [in] pGains : The proportional gains.
   * @param [in] dGains : The derivative gains.
   */
  void setGains(const vector_t& pGains, const vector_t& dGains) {
    pGains_ = pGains.asDiagonal();
    dGains_ = dGains.asDiagonal();
  }

  /**
   * Get the torque.
   * @param [in] desiredState : The desired ocs2 state.
   * @param [in] desiredInput : The desired ocs2 input.
   * @param [in] measuredState : The measured ocs2 state (required for PD control).
   * @param [in] measuredInput : The measured ocs2 input (required for PD control).
   */
  vector_t getTorque(const vector_t& desiredState, const vector_t& desiredInput, const vector_t& measuredState,
                     const vector_t& measuredInput);

 private:
  PinocchioInterface* pinocchioInterfacePtr_;
  CentroidalModelPinocchioMapping centroidalModelPinocchioMapping_;
  CentroidalModelRbdConversions centroidalModelRbdConversions_;
  const std::vector<std::string> contactNames3DoF_;
  matrix_t pGains_;
  matrix_t dGains_;
};

}  // namespace ocs2
