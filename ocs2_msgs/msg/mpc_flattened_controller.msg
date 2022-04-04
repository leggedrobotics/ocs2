# Flattened controller: A serialized controller

# define controllerType Enum values
uint8 CONTROLLER_UNKNOWN=0 # safety mechanism: message initalization to zero
uint8 CONTROLLER_FEEDFORWARD=1
uint8 CONTROLLER_LINEAR=2

uint8                   controllerType         # what type of controller is this

mpc_observation         initObservation        # plan initial observation

mpc_target_trajectories planTargetTrajectories # target trajectory in cost function
mpc_state[]             stateTrajectory        # optimized state trajectory from planner
mpc_input[]             inputTrajectory        # optimized input trajectory from planner
float64[]               timeTrajectory         # time trajectory for stateTrajectory and inputTrajectory
uint16[]                postEventIndices       # array of indices indicating the index of post-event time in the trajectories
mode_schedule           modeSchedule           # optimal/predefined MPC mode sequence and event times

controller_data[]       data                   # the actual payload from flatten method: one vector of data per time step

mpc_performance_indices performanceIndices     # solver performance indices
