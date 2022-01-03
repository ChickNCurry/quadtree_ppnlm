cmake_minimum_required(VERSION 3.20)
project(quadtree_ppnlm)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 14)

add_executable(quadtree_ppnlm src/main.cpp)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(quadtree_ppnlm Eigen3::Eigen)
target_link_libraries(quadtree_ppnlm ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
