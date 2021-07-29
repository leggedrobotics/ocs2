cmake_minimum_required(VERSION 3.0.2)
project(ocs2_msgs)

set(CATKIN_PACKAGE_DEPENDENCIES std_msgs)

find_package(catkin REQUIRED
    COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        message_generation
)

add_message_files(
    FILES
        mpc_state.msg
        mpc_input.msg
        mode_schedule.msg
        mpc_observation.msg
        mpc_performance_indices.msg
        mpc_target_trajectories.msg
        controller_data.msg
        mpc_flattened_controller.msg
)

add_service_files(
    FILES
        reset.srv
)

generate_messages(
    DEPENDENCIES
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
    CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        message_runtime
)
