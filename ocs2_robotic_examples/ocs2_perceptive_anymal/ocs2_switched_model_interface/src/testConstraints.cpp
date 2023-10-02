

#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h>

int main(int argc, char** argv) {
  using EndEffectorVelocityConstraint_t = switched_model::EndEffectorVelocityConstraint<3, 2>;

  // Set up constraint
  switched_model::EndEffectorVelocityConstraintSettings settings;
  EndEffectorVelocityConstraint_t EeVelconstraint(settings);

  // Set up (t, x, u, p)
  EndEffectorVelocityConstraint_t::scalar_t time;
  EndEffectorVelocityConstraint_t::state_vector_t state;
  EndEffectorVelocityConstraint_t::input_vector_t input;
  time = 0;
  state.setOnes();
  input.setOnes();
  settings.desiredEndEffectorVelocity.setConstant(1.5);

  // Evaluate and print
  auto constraints = EeVelconstraint.getValue(time, state, input);
  auto linearApproximation = EeVelconstraint.getLinearApproximation(time, state, input);

  std::cout << "Constraint values" << std::endl;
  for (auto& c : constraints) {
    std::cout << c << std::endl;
  }

  std::cout << "State Derivatives" << std::endl;
  for (auto& dcdx : linearApproximation.derivativeState) {
    std::cout << dcdx.transpose() << std::endl;
  }

  std::cout << "Input Derivatives" << std::endl;
  for (auto& dcdu : linearApproximation.derivativeInput) {
    std::cout << dcdu.transpose() << std::endl;
  }

  return 0;
}