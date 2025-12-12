cmake_minimum_required(VERSION 3.14)
project(blasfeo_catkin)

find_package(ament_cmake REQUIRED)

include(FetchContent)

# BLASFEO settings
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
set(TARGET X64_AUTOMATIC CACHE STRING "Target architecture" FORCE)
set(BLASFEO_EXAMPLES OFF CACHE BOOL "Examples enabled" FORCE)

FetchContent_Declare(blasfeodownload
  GIT_REPOSITORY https://github.com/giaf/blasfeo
  GIT_TAG ae6e2d1dea015862a09990b95905038a756ffc7d
  UPDATE_COMMAND ""
)
FetchContent_MakeAvailable(blasfeodownload)

# Install blasfeo target and headers into this package prefix
install(
  TARGETS blasfeo
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  DIRECTORY ${blasfeodownload_SOURCE_DIR}/include/
  DESTINATION include
)

# Provide BLASFEO_PATH for downstream packages (used by hpipm_catkin)
set(BLASFEO_DEVEL_PREFIX ${CMAKE_INSTALL_PREFIX})
configure_file(
  cmake/blasfeo-extras.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/blasfeo-extras.cmake
  @ONLY
)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/blasfeo-extras.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package(CONFIG_EXTRAS "${CMAKE_CURRENT_BINARY_DIR}/blasfeo-extras.cmake")
