cmake_minimum_required(VERSION 3.14)
project(blasfeo_catkin)

set(CMAKE_ASM_COMPILER "as")

find_package(ament_cmake REQUIRED)

include(FetchContent)

# Define directories

set(BLASFEO_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(BLASFEO_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${BLASFEO_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${BLASFEO_BUILD_DIR})

# BLASFEO Settings
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled")

# Download & build source
FetchContent_Declare(blasfeoDownload
	GIT_REPOSITORY https://github.com/giaf/blasfeo
	GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
	UPDATE_COMMAND ""
	SOURCE_DIR ${BLASFEO_DOWNLOAD_DIR}
	BINARY_DIR ${BLASFEO_BUILD_DIR}
	BUILD_COMMAND $(MAKE)
	INSTALL_COMMAND "$(MAKE) install"
)
FetchContent_MakeAvailable(blasfeoDownload)

# Propagate dependencies

ament_export_libraries(blasfeo)

ament_package()
