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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
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
  EXPORT export_surface_vehicle_model  # <-- Add this line
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(surface_vehicle_model_lib)
ament_export_dependencies(rclcpp rml)
ament_export_targets(export_surface_vehicle_model)

ament_package()
