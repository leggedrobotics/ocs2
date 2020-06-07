cmake_minimum_required(VERSION 3.0.0)
project(switched_model_msgs)

# Get the information about this package's buildtime dependencies

find_package(catkin REQUIRED
  COMPONENTS message_generation std_msgs ocs2_msgs)

# Declare the message files to be built
add_message_files(FILES
  gait.msg
  gait_sequence.msg
  scheduled_gait.msg
  )

# Declare the service files to be built
add_service_files(FILES
  trajectory_request.srv
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs ocs2_msgs)

# Declare that this catkin package's runtime dependencies
catkin_package(
 CATKIN_DEPENDS message_runtime std_msgs ocs2_msgs
)

