# MPC feedback policy

mpc_observation 		initObservation			# Plan initial observation
mpc_target_trajectories planTargetTrajectories 	# PLan target trajectory

bool 					controllerIsUpdated		# Either the controller is updated or not 

int8 				   dimState					# state dimension
int8 				   dimInput					# input dimension
slq_controller[]	   slqControllerTrajectory	# controller trajectory

mode_sequence 			modeSequence			# Optimal/Predefined MPC mode sequence
