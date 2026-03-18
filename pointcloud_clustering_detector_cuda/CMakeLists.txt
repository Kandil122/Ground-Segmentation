cmake_minimum_required(VERSION 3.10)
project(pointcloud_clustering_detector_cuda LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MPI)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

set(SOURCES
    src/main.cpp
    src/point_cloud_loader.cpp
    src/point_cloud_visualizer.cpp
    src/box_filter.cpp
    src/voxel_filter.cpp
    src/noise_filter.cpp
    src/ground_estimation.cpp
    src/wall_filter.cpp
    src/kalman_filter.cpp
    src/object_detector.cpp
    src/detected_object.cpp
    src/pose_exporter.cpp
    src/cpu_stubs.cpp
)

add_executable(pointcloud_clustering_detector_cuda ${SOURCES})

target_link_libraries(pointcloud_clustering_detector_cuda
    ${PCL_LIBRARIES}
    yaml-cpp
)