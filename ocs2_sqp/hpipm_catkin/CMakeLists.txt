cmake_minimum_required(VERSION 3.10)
project(hpipm_catkin)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  blasfeo_catkin
)

include(ExternalProject)

set(HPIPM_INSTALL_PREFIX $ENV{HOME}/.local/hpipm CACHE STRING "HPIPM install path")
SET(HPIPM_INCLUDE_DIR ${HPIPM_INSTALL_PREFIX}/include)
SET(HPIPM_LIB_DIR ${HPIPM_INSTALL_PREFIX}/lib)

file(MAKE_DIRECTORY ${HPIPM_INCLUDE_DIR})

ExternalProject_Add(hpipmDownload
  GIT_REPOSITORY https://github.com/giaf/hpipm
  UPDATE_COMMAND ""
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${HPIPM_INSTALL_PREFIX}
    -DBUILD_SHARED_LIBS:STRING=ON
    -DBLASFEO_PATH:STRING=${BLASFEO_PATH} # Find blasfeo through exported path from blasfeo_catkin
    -DHPIPM_TESTING:BOOL=OFF # Don't build tests
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND $(MAKE) install
)

add_library(${PROJECT_NAME} SHARED IMPORTED)
set_target_properties(${PROJECT_NAME} PROPERTIES IMPORTED_LOCATION ${HPIPM_LIB_DIR}/libhpipm.so)

catkin_package(
  INCLUDE_DIRS ${HPIPM_INCLUDE_DIR}
  CATKIN_DEPENDS blasfeo_catkin
  LIBRARIES ${PROJECT_NAME}
  CFG_EXTRAS hpipm-extras.cmake
)



