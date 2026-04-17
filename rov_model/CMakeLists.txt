cmake_minimum_required(VERSION 3.5)
project(rov_model)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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
  ${rov_msgs_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()


add_library(rov_model_lib src/rov_model.cpp)
target_link_libraries(rov_model_lib rml)


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
  TARGETS rov_model_lib
  EXPORT rov_modelTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT rov_modelTargets
  FILE rov_modelTargets.cmake
  NAMESPACE rov_model::
  DESTINATION lib/cmake/rov_model
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rov_modelConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/rov_modelConfig.cmake"
  INSTALL_DESTINATION lib/cmake/rov_model
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/rov_modelConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY AnyNewerVersion
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/rov_modelConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/rov_modelConfigVersion.cmake"
  DESTINATION lib/cmake/rov_model
)
