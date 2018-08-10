# MPC feedforward policy (optimized trajecotories)

mpc_observation 		initObservation			# Plan initial observation
mpc_target_trajectories planTargetTrajectories 	# PLan target trajectory

bool 					controllerIsUpdated		# Either the controller is updated or not 

float32[] 				timeTrajectory			# Optimal MPC time trajectory
mpc_state[] 			stateTrajectory			# Optimal MPC state trajectory
mpc_input[] 			inputTrajectory			# Optimal MPC input trajectory

mode_sequence 			modeSequence			# Optimal/Predefined MPC mode sequence
