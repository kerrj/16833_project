cmake_minimum_required(VERSION 2.8.3)
project(slam_project)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
#find_package(Boost REQUIRED)
#find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(gtsam required)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
)

add_executable(slam_project src/main.cpp src/hungarian.cpp)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)
