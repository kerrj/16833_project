cmake_minimum_required(VERSION 3.12.0)
project(slam_project LANGUAGES CXX)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
    include
    ${GTSAM_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

add_executable(slam_project src/main.cpp src/hungarian.cpp)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    gtsam
    OpenMP::OpenMP_CXX
)

target_compile_options(slam_project PRIVATE -Wall -Wextra -Wno-reorder -Wno-unused-parameter -Wno-sign-compare)
