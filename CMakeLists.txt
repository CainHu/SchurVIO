cmake_minimum_required(VERSION 3.29)
project(VinsEKF)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -march=native -Xpreprocessor -fopenmp")
set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/opt/libomp/include")
set(OpenMP_C_LIB_NAMES "omp")
set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/opt/libomp/include")
set(OpenMP_CXX_LIB_NAMES "omp")
set(OpenMP_omp_LIBRARY "/opt/homebrew/opt/libomp/lib/libomp.dylib")

include_directories("/opt/homebrew/opt/libomp/include")

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package ( OpenMP REQUIRED )

include_directories(
#        ${CMAKE_SOURCE_DIR}/modules
#        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenMP_C_INCLUDE_DIRS}
)

add_executable(VinsEKF main.cpp
        data_structure/frame.cpp
        data_structure/frame.h
        data_structure/landmark.cpp
        data_structure/landmark.h
        data_structure/feature.cpp
        data_structure/feature.h
        data_structure/map.cpp
        data_structure/map.h
        type.h
        data_structure/pool.cpp
        data_structure/pool.h
        vio_frontend_simulator.cpp
        vio_frontend_simulator.h
        eskf/schur_vins.cpp
        eskf/schur_vins.h
        common.h
#        multi_fusion.h
#        multi_fusion2.h
)

target_link_libraries(VinsEKF
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${Pangolin_LIBRARIES}
        OpenMP::OpenMP_CXX)
