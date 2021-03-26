cmake_minimum_required(VERSION 3.14)
project(hpipm_catkin)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  blasfeo_catkin
  ocs2_core
  ocs2_qp_solver
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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
  GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf
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

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include ${HPIPM_INCLUDE_DIR}
  CATKIN_DEPENDS blasfeo_catkin
  LIBRARIES ${PROJECT_NAME} hpipm
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Hpipm interface
add_library(${PROJECT_NAME}
  src/HpipmInterface.cpp
  src/HpipmInterfaceSettings.cpp
  src/OcpSize.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  hpipm
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS
    ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############

catkin_add_gtest(test_${PROJECT_NAME}
  test/testHpipmInterface.cpp
)
add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  hpipm
  gtest_main
)
