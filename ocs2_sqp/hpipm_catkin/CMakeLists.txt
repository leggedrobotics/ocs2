cmake_minimum_required(VERSION 3.14)
project(hpipm_catkin)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  blasfeo_catkin
)

include(FetchContent)

# Define directories
set(HPIPM_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX} CACHE STRING "HPIPM install path")
set(HPIPM_INCLUDE_DIR ${HPIPM_DEVEL_PREFIX}/include)
set(HPIPM_LIB_DIR ${HPIPM_DEVEL_PREFIX}/lib)
set(HPIPM_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(HPIPM_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${HPIPM_INCLUDE_DIR})
file(MAKE_DIRECTORY ${HPIPM_LIB_DIR})
file(MAKE_DIRECTORY ${HPIPM_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${HPIPM_BUILD_DIR})

message(STATUS "BLASFEO_PATH: " ${BLASFEO_PATH})

# HPIPM Settings
set(BLASFEO_PATH ${BLASFEO_PATH} CACHE STRING "BLASFEO installation path" FORCE)
set(BLASFEO_INCLUDE_DIR ${BLASFEO_PATH}/include CACHE STRING "Path to BLASFEO header files." FORCE)
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "Examples enabled")

# Download & build source
FetchContent_Declare(hpipmDownload
  GIT_REPOSITORY https://github.com/giaf/hpipm
  UPDATE_COMMAND ""
  SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
  BINARY_DIR ${HPIPM_BUILD_DIR}
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND "$(MAKE) install"
  )
FetchContent_MakeAvailable(hpipmDownload)

# Copy header to where catkin expects them
file(GLOB_RECURSE HEADERS "${HPIPM_DOWNLOAD_DIR}/include/*.h")
foreach(HEADER_FILE ${HEADERS})
  message(STATUS "FOUND HEADER: " ${HEADER_FILE})
  file(COPY ${HEADER_FILE} DESTINATION ${HPIPM_INCLUDE_DIR})
endforeach()

# Install the library where catkin expects them
set_target_properties(hpipm PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${HPIPM_LIB_DIR})

catkin_package(
  INCLUDE_DIRS ${HPIPM_INCLUDE_DIR}
  CATKIN_DEPENDS blasfeo_catkin
  LIBRARIES hpipm
)


