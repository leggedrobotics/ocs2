cmake_minimum_required(VERSION 3.10)
project(convex_plane_decomposition)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

find_package(CGAL REQUIRED)
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)

find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_filters_rsl REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/contour_extraction/ContourExtraction.cpp
  src/contour_extraction/Upsampling.cpp
  src/ransac/RansacPlaneExtractor.cpp
  src/sliding_window_plane_extraction/SlidingWindowPlaneExtractor.cpp
  src/ConvexRegionGrowing.cpp
  src/Draw.cpp
  src/GridMapPreprocessing.cpp
  src/LoadGridmapFromImage.cpp
  src/PlanarRegion.cpp
  src/PlaneDecompositionPipeline.cpp
  src/Postprocessing.cpp
  src/SegmentedPlaneProjection.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PUBLIC CGAL_HAS_THREADS)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME}
  grid_map_core
  grid_map_cv
  grid_map_filters_rsl
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  CGAL::CGAL
  gmp
  mpfr
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  CGAL
  Eigen3
  grid_map_core
  grid_map_cv
  grid_map_filters_rsl
)
ament_package()

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem system)
  ament_add_gtest(test_${PROJECT_NAME}
    test/testConvexApproximation.cpp
    test/testPipeline.cpp
    test/testPlanarRegion.cpp
    test/testRegionGrowing.cpp
    test/testUpsampling.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    Boost::filesystem
    Boost::system
  )
endif()
