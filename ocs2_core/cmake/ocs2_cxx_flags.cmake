# The list of compiler flags used in ocs2 can be prefixed with catkin config
# Addition flags are to be separated by \;
# For example, to turn on architecture specific optimizations:
#   catkin config --cmake-args -DOCS2_CXX_FLAGS=-march=native\;-mtune=native
list(APPEND OCS2_CXX_FLAGS
  "-pthread"
  "-Wfatal-errors"
  "-Wl,--no-as-needed"
  )

# Force Boost dynamic linking
list(APPEND OCS2_CXX_FLAGS
  "-DBOOST_ALL_DYN_LINK"
  )

# Add OpenMP flags
if (NOT DEFINED OpenMP_CXX_FOUND)
  find_package(OpenMP REQUIRED)
endif (NOT DEFINED OpenMP_CXX_FOUND)
list(APPEND OCS2_CXX_FLAGS
  ${OpenMP_CXX_FLAGS}
  )

# Cpp standard version
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-sign-compare -Wno-range-loop-construct -Wno-ignored-qualifiers)
  add_compile_options(-Wno-unused-parameter -Wno-unused-variable -Wno-range-loop-construct -Wno-empty-body -Wno-maybe-uninitialized)
  add_compile_options(-Wno-implicit-fallthrough -Wno-reorder -Wno-redundant-move -Wno-missing-field-initializers)
endif()
