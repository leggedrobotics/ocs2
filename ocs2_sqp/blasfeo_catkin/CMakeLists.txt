cmake_minimum_required(VERSION 3.10)
project(blasfeo_catkin)

find_package(catkin REQUIRED)

include(ExternalProject)

set(BLASFEO_INSTALL_PREFIX $ENV{HOME}/.local/blasfeo CACHE STRING "Blasfeo install path")
SET(BLASFEO_INCLUDE_DIR ${BLASFEO_INSTALL_PREFIX}/include)
SET(BLASFEO_LIB_DIR ${BLASFEO_INSTALL_PREFIX}/lib)

file(MAKE_DIRECTORY ${BLASFEO_INCLUDE_DIR})

ExternalProject_Add(blasfeoDownload
  GIT_REPOSITORY https://github.com/giaf/blasfeo
  UPDATE_COMMAND ""
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${BLASFEO_INSTALL_PREFIX}
    -DBUILD_SHARED_LIBS:STRING=ON
    -DTARGET:STRING=X64_AUTOMATIC
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND $(MAKE) install
)

add_library(${PROJECT_NAME} SHARED IMPORTED)
set_target_properties(${PROJECT_NAME} PROPERTIES IMPORTED_LOCATION ${BLASFEO_LIB_DIR}/libblasfeo.so)

catkin_package(
  INCLUDE_DIRS ${BLASFEO_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CFG_EXTRAS blasfeo-extras.cmake
)



