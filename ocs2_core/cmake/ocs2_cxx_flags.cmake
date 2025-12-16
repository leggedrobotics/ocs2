# The list of compiler flags used in ocs2 can be prefixed from the build system.
# Addition flags are to be separated by \;
# For example, to turn on architecture specific optimizations:
#   colcon build --cmake-args -DOCS2_CXX_FLAGS=-march=native\;-mtune=native
list(APPEND OCS2_CXX_FLAGS
  "-pthread"
  "-Wfatal-errors"
  "-Wl,--no-as-needed"
  )

# Force Boost dynamic linking
list(APPEND OCS2_CXX_FLAGS
  "-DBOOST_ALL_DYN_LINK"
  "-DBOOST_MPL_LIMIT_VECTOR_SIZE=30"
  "-DBOOST_MPL_LIMIT_MAP_SIZE=30"
  "-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS"
  )

# Add OpenMP flags
if (NOT DEFINED OpenMP_CXX_FOUND)
  find_package(OpenMP REQUIRED)
endif (NOT DEFINED OpenMP_CXX_FOUND)
list(APPEND OCS2_CXX_FLAGS
  ${OpenMP_CXX_FLAGS}
  )

# Cpp standard version (ROS 2 Jazzy uses C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
