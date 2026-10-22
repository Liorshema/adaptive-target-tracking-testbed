cmake_minimum_required(VERSION 3.16)
project(robot_simulation_actuator LANGUAGES CXX)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
add_library(robot_actuator SHARED actuator_system.cc)
target_compile_features(robot_actuator PRIVATE cxx_std_17)
target_compile_options(robot_actuator PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(robot_actuator PRIVATE gz-sim::gz-sim gz-plugin::register)
set_target_properties(robot_actuator PROPERTIES PREFIX "" SUFFIX ""
  OUTPUT_NAME "${ACTUATOR_OUTPUT_NAME}"
  LIBRARY_OUTPUT_DIRECTORY "${ACTUATOR_OUTPUT_DIRECTORY}")

# The recovery backend is compiled from pinned source, not linked against the
# stock plugin's private C++ implementation or fetched during a build.
find_package(gz_physics_vendor REQUIRED)
find_package(gz_dartsim_vendor REQUIRED)
find_package(gz-physics7 7.6.0 EXACT REQUIRED COMPONENTS dartsim sdf heightmap mesh)
find_package(DART 6.13.2 EXACT CONFIG REQUIRED COMPONENTS collision-bullet collision-ode)
find_package(gz-common5 REQUIRED COMPONENTS geospatial graphics profiler)
find_package(gz-math7 REQUIRED COMPONENTS eigen3)
file(GLOB recovery_sources CONFIGURE_DEPENDS "dartsim_recovery/*.cc")
add_library(robot_dartsim_recovery SHARED ${recovery_sources})
target_compile_features(robot_dartsim_recovery PRIVATE cxx_std_17)
target_compile_definitions(robot_dartsim_recovery PRIVATE DART_HAS_CONTACT_SURFACE)
target_link_libraries(robot_dartsim_recovery PRIVATE
  gz-physics7::dartsim gz-physics7::sdf gz-physics7::heightmap gz-physics7::mesh
  gz-common5::gz-common5 gz-common5::geospatial gz-common5::profiler
  gz-math7::eigen3 gz-plugin::register)
set_target_properties(robot_dartsim_recovery PROPERTIES
  PREFIX "" SUFFIX "" OUTPUT_NAME "${RECOVERY_OUTPUT_NAME}"
  LIBRARY_OUTPUT_DIRECTORY "${RECOVERY_OUTPUT_DIRECTORY}")
