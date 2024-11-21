cmake_minimum_required(VERSION 3.8)
project(6DOF_model)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(Eigen3 REQUIRED)
find_package(libconfig REQUIRED)
find_library(RML_LIBRARY NAMES rml)

# Prefer shared libraries over static ones
set(CMAKE_FIND_LIBRARY_SUFFIXES .so .a)

# Find qpOASES shared library
find_library(QPOASES_LIBRARY NAMES qpOASES REQUIRED)

# Output the found qpOASES library
message(STATUS "Found QPOASES_LIBRARY: ${QPOASES_LIBRARY}")

# Define the 6DOF_model library
add_library(${PROJECT_NAME} SHARED
  src/6DOF_model.cpp
)

# Add dependencies to the 6DOF_model library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${libconfig_INCLUDE_DIRS}
)

# Link additional libraries
target_link_libraries(${PROJECT_NAME}
  ${QPOASES_LIBRARY}
  ${RML_LIBRARY}
  config++
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
