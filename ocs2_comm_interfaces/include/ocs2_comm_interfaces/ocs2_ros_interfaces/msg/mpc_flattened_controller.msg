# Flattened controller: A serialized controller

# define controllerType Enum values
uint8 CONTROLLER_UNKNOWN=0 # safety mechanism: message initalization to zero
uint8 CONTROLLER_FEEDFORWARD=1
uint8 CONTROLLER_LINEAR=2

uint8                   controllerType         # what type of controller is this

mpc_observation         initObservation        # Plan initial observation

mpc_target_trajectories planTargetTrajectories # target trajectory in cost function
mpc_state[]             stateTrajectory        # optimized nominal state trajectory from planner
float64[]               timeTrajectory         # time trajectory for stateTrajectories
mode_sequence           modeSequence           # Optimal/Predefined MPC mode sequence

bool                    controllerIsUpdated


controller_data[]       data                   # the actual payload from flatten method: one vector of data per time step
