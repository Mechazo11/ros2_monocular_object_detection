# NOTES import library from one package into library of another package https://answers.ros.org/question/403777/ros2-adding-a-library-from-another-package-to-a-library/

cmake_minimum_required(VERSION 3.8)
project(ros2_monocular_object_detection)

# Optional if you need to use python packages, this needs to be set manually
#set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/") # Must be set to match your installation

# Use C++17, required by rclcpp
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  add_definitions(-DCOMPILEDWITHC17)
  message(STATUS "Using flag -std=c++17.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED)
find_package(PCL 1.13 REQUIRED)
find_package(ros2_line_lbd REQUIRED) # Dr. Yang`s line_lbd library built as a standalone ros2 package
find_package(ros2_tictoc_profiler REQUIRED) # ROS 2 library only package, port of Daniel Mataurna`s tictoc_profiler package
# find_package(Boost REQUIRED COMPONENTS system) # Need to check if this works or not

# Header file locations, link libraries and definitions
include_directories(include) # This package include
include_directories(${EIGEN3_INCLUDE_DIRS}) # Include headers for eigen3
include_directories(${OpenCV_INCLUDE_DIRS}) # include headers for pangolin
include_directories(${PCL_INCLUDE_DIRS}) # include headers for PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Debug
#message("PCL Include Directories: ${PCL_INCLUDE_DIRS}") # Debug message
#message("PCL_SAMPLE_CONSESUS_LIBRARIES: " ${PCL_SAMPLE_CONSESUS_LIBRARIES})


# Include libraries from other packges and system-wide installed packages
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${ros2_line_lbd_INCLUDE_DIRS}
  ${ros2_tictoc_profiler_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  OpenCV
  Eigen3
  PCL
  # ros2_line_lbd
  # ros2_tictoc_profiler
)

# Setup internal libraries
#* Use the structure below. For more advanced example follow this https://github.com/Mechazo11/ros2_orb_slam3/blob/main/CMakeLists.txt

# -----------------------------------------------------------------------------------------
# Library to detect 3D cuboids from rgb images
add_library(cuboid_bbox_lib SHARED
    src/detect_cuboid_bbox.cpp
    src/matrix_utils.cpp
    src/object_3d_util.cpp
)

# Required, according to ChatGPT this sets the name of the libcuboid_bbox_lib to something like `libcuboid_bbox_lib.so.1.2.3` that helps linkers to find the correct version of this library
set_target_properties(cuboid_bbox_lib PROPERTIES VERSION "${cuboid_bbox_lib_VERSION}")

# Add necessary includes for this shared library
ament_target_dependencies(cuboid_bbox_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link external libraries with the shared internal library
target_link_libraries(cuboid_bbox_lib
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${ros2_line_lbd_LIBRARIES}
    ${ros2_tictoc_profiler_LIBRARIES}
    -lboost_system
  )

# -----------------------------------------------------------------------------------------


# Library to find planes (walls, ground etc.)
add_library(plane_det_lib SHARED
  src/plane_detection.cpp
)

# Required, according to ChatGPT this sets the name of the libcuboid_bbox_lib to something like `libcuboid_bbox_lib.so.1.2.3` that helps linkers to find the correct version of this library
set_target_properties(plane_det_lib PROPERTIES VERSION "${plane_det_lib_VERSION}")

# Add necessary includes for this shared library
ament_target_dependencies(plane_det_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link external libraries with the shared internal library
target_link_libraries(plane_det_lib
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    -lboost_system
    ${PCL_LIBRARIES}
    # ${ros2_line_lbd_LIBRARIES}
    # ${ros2_tictoc_profiler_LIBRARIES}
    
  )
# -----------------------------------------------------------------------------------------


# Library to work with SUN RGB-D dataset
add_library(sun_rgbd_lib SHARED
  src/sun_rgbd.cpp
)

# Required, according to ChatGPT this sets the name of the libcuboid_bbox_lib to something like `libcuboid_bbox_lib.so.1.2.3` that helps linkers to find the correct version of this library
set_target_properties(sun_rgbd_lib PROPERTIES VERSION "${sun_rgbd_lib_VERSION}")

# Add necessary includes for this shared library
ament_target_dependencies(sun_rgbd_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(sun_rgbd_lib
  ${EIGEN3_LIBS} 
  ${ros2_line_lbd_LIBRARIES}
  ${ros2_tictoc_profiler_LIBRARIES}
)

# -----------------------------------------------------------------------------------------

# Define executables and tie relevant .cpp files
add_executable(det_rgbd_opti_sun_node 
  src/sun_rgbd_driver.cpp
  src/sun_rgbd_example_class.cpp 
)

ament_target_dependencies(det_rgbd_opti_sun_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}

)

# Link a node with the internal shared libraries
target_link_libraries(det_rgbd_opti_sun_node PUBLIC 
  cuboid_bbox_lib
  plane_det_lib
  sun_rgbd_lib
  ${ros2_line_lbd_LIBRARIES}
  ${ros2_tictoc_profiler_LIBRARIES}
) 


# Install all the header files in package/package/include
install (DIRECTORY include/ 
        DESTINATION include
)

# Install our node and library
install(TARGETS det_rgbd_opti_sun_node cuboid_bbox_lib plane_det_lib sun_rgbd_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Tell downstream packages where to find the headers
ament_export_include_directories(include)

# Help downstream packages to find transitive dependencies
ament_export_dependencies(
  cuboid_bbox_lib
  plane_det_lib
  sun_rgbd_lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# If you have python nodes
# # Python node
# # Install Python modules, submodules
# ament_python_install_package(${PROJECT_NAME}) # Install the modules in ros2_orb_slam3/ros2_orb_slam3 folder

# # Install Python executibles
# install(PROGRAMS
#   scripts/mono_driver_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
  