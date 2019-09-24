cmake_minimum_required(VERSION 3.0)
project(ocs2_raisim_subproject)

include(ExternalProject)

# pull in Raisim
ExternalProject_Add(raisim
  GIT_REPOSITORY https://github.com/leggedrobotics/raisimLib.git
  PREFIX ${CMAKE_BINARY_DIR}/raisim
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/raisim/install"
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)
