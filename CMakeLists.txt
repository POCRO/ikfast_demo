cmake_minimum_required(VERSION 3.10)
project(ikfast_demo)

set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(LAPACK REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# Executable
add_executable(ikfast_demo main.cpp src/ikfast_wrapper.cpp)



# Include directories
target_include_directories(ikfast_demo PRIVATE
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /opt/openrobots/include
  src
  inc
)

# Libraries
target_link_libraries(ikfast_demo PRIVATE
  ${LAPACK_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
