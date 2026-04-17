cmake_minimum_required(VERSION 3.5)
project(surface_vehicle_model)

#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
#endif()

#if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(rml REQUIRED)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${ulisse_msgs_INCLUDE_DIRS}
)

add_library(surface_vehicle_model_lib src/surfacevehiclemodel.cpp)
target_link_libraries(surface_vehicle_model_lib rml)



# install artifacts
install(DIRECTORY
  conf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS surface_vehicle_model_lib
  EXPORT surface_vehicle_modelTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT surface_vehicle_modelTargets
  FILE surface_vehicle_modelTargets.cmake
  NAMESPACE surface_vehicle_model::
  DESTINATION lib/cmake/surface_vehicle_model
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/surface_vehicle_modelConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/surface_vehicle_modelConfig.cmake"
  INSTALL_DESTINATION lib/cmake/surface_vehicle_model
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/surface_vehicle_modelConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY AnyNewerVersion
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/surface_vehicle_modelConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/surface_vehicle_modelConfigVersion.cmake"
  DESTINATION lib/cmake/surface_vehicle_model
)
