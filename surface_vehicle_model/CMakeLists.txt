cmake_minimum_required(VERSION 3.1)
project(surface_vehicle_model)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#if(NOT CMAKE_BUILD_TYPE)
#    set(CMAKE_BUILD_TYPE Debug)
#endif()


find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES surface_vehicle_model
    CATKIN_DEPENDS roscpp std_msgs
    #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/surfacevehiclemodel.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} rml)

## Mark executables for installation
#install(TARGETS ${PROJECT_NAME}_node
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )

## Mark libraries for installation
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING
#    PATTERN "*.h"
#    PATTERN "*.hpp"
#    PATTERN ".svn" EXCLUDE
#    )

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
    # myfile1
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    )
