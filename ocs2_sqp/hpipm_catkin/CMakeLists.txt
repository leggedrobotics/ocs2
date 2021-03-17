cmake_minimum_required(VERSION 3.10)
project(hpipm_catkin)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  blasfeo_catkin
)

include(ExternalProject)

set(HPIPM_INSTALL_PREFIX $ENV{HOME}/.local CACHE STRING "HPIPM install path")
set(HPIPM_INCLUDE_DIR ${HPIPM_INSTALL_PREFIX}/include)
set(HPIPM_LIB_DIR ${HPIPM_INSTALL_PREFIX}/lib)

file(MAKE_DIRECTORY ${HPIPM_INCLUDE_DIR})
file(MAKE_DIRECTORY ${HPIPM_LIB_DIR})

message(STATUS "BLASFEO_PATH: " ${BLASFEO_PATH})

ExternalProject_Add(hpipmDownload
  GIT_REPOSITORY https://github.com/giaf/hpipm
  UPDATE_COMMAND ""
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${HPIPM_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_SHARED_LIBS:STRING=ON
    -DBLASFEO_PATH:STRING=${BLASFEO_PATH} # Find blasfeo through exported path from blasfeo_catkin
    -DHPIPM_TESTING:BOOL=OFF # Don't build tests
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND $(MAKE) install
)

# Create a library to be used in the catkin_package LIBRARIES
add_library(${PROJECT_NAME} SHARED IMPORTED)
set_target_properties(${PROJECT_NAME} PROPERTIES IMPORTED_LOCATION ${HPIPM_LIB_DIR}/libhpipm.so)
add_dependencies(${PROJECT_NAME} hpipmDownload ${catkin_EXPORTED_TARGETS})

list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hpipmDownload)

catkin_package(
  INCLUDE_DIRS ${HPIPM_INCLUDE_DIR}
  CATKIN_DEPENDS blasfeo_catkin
  LIBRARIES ${PROJECT_NAME}
  CFG_EXTRAS hpipm-extras.cmake
)

install(DIRECTORY ${HPIPM_INCLUDE_DIR}/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY ${HPIPM_LIB_DIR}/
  DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
)



