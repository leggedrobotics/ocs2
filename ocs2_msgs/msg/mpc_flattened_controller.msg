# Flattened controller: A serialized controller

# define controllerType Enum values
uint8 CONTROLLER_UNKNOWN=0 # safety mechanism: message initalization to zero
uint8 CONTROLLER_FEEDFORWARD=1
uint8 CONTROLLER_LINEAR=2

uint8                   controllerType         # what type of controller is this

mpc_observation         initObservation        # Plan initial observation

mpc_target_trajectories planTargetTrajectories # target trajectory in cost function
mpc_state[]             stateTrajectory        # optimized state trajectory from planner
mpc_input[]             inputTrajectory        # optimized input trajectory from planner
float32[]               timeTrajectory         # time trajectory for stateTrajectory and inputTrajectory
mode_schedule           modeSchedule           # Optimal/Predefined MPC mode sequence and event times

bool                    controllerIsUpdated

controller_data[]       data                   # the actual payload from flatten method: one vector of data per time step
