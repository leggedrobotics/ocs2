# Flattened controller: A serialized controller

# define controllerType Enum values
uint8 CONTROLLER_UNKNOWN=0 # safety mechanism: message initalization to zero
uint8 CONTROLLER_FEEDFORWARD=1
uint8 CONTROLLER_LINEAR=2

uint8                   controller_type          # what type of controller is this

MPCObservation          init_observation         # plan initial observation

MPCTargetTrajectories   plan_target_trajectories # target trajectory in cost function
MPCState[]              state_trajectory         # optimized state trajectory from planner
MPCInput[]              input_trajectory         # optimized input trajectory from planner
float64[]               time_trajectory          # time trajectory for stateTrajectory and inputTrajectory
uint16[]                post_event_indices       # array of indices indicating the index of post-event time in the trajectories
ModeSchedule            mode_schedule            # optimal/predefined MPC mode sequence and event times

ControllerData[]        data                     # the actual payload from flatten method: one vector of data per time step

MPCPerformanceIndices   performance_indices      # solver performance indices
