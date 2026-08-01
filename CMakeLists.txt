cmake_minimum_required(VERSION 3.29)
project(VinsEKF)

set(CMAKE_CXX_STANDARD 20)

if(MINGW)
    # 1. 允许汇编器生成大型目标文件（核心选项）
    # -Wa,传递选项给汇编器；-mbig-obj：突破默认.obj大小限制（默认约2GB）
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wa,-mbig-obj")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g0 -Wa,-mbig-obj")

    # 4. （可选）启用链接时优化（LTO），减少目标文件大小
    # 需确保MinGW版本支持（GCC 4.6+），可能略微增加编译时间
    # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -flto")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto")
    # set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif()


set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -march=native -fopenmp")

#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -march=native -Xpreprocessor -fopenmp")
#set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/opt/libomp/include")
#set(OpenMP_C_LIB_NAMES "omp")
#set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/opt/libomp/include")
#set(OpenMP_CXX_LIB_NAMES "omp")
#set(OpenMP_omp_LIBRARY "/opt/homebrew/opt/libomp/lib/libomp.dylib")

#include_directories("/opt/homebrew/opt/libomp/include")

#find_package(Eigen3 REQUIRED)
#find_package(Pangolin REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
#find_package ( OpenMP REQUIRED )

#include_directories(
##        ${CMAKE_SOURCE_DIR}/modules
##        ${PROJECT_SOURCE_DIR}/include
##        ${EIGEN3_INCLUDE_DIR}
##        ${CERES_INCLUDE_DIRS}
##        ${Pangolin_INCLUDE_DIRS}
##        ${Boost_INCLUDE_DIRS}
##        ${OpenMP_C_INCLUDE_DIRS}
#)

add_executable(VinsEKF main.cpp
        type.h
        data_structure/frame.cpp
        data_structure/frame.h
        data_structure/landmark.cpp
        data_structure/landmark.h
        data_structure/feature.cpp
        data_structure/feature.h
        data_structure/map.cpp
        data_structure/map.h
        data_structure/pool.cpp
        data_structure/pool.h
        vio_frontend_simulator.cpp
        vio_frontend_simulator.h
        eskf/schur_vins.cpp
        eskf/schur_vins.h
        common.h
        vio_frontend_simulator1.cpp
        vio_frontend_simulator1.h
        vio_representative_simulator.cpp
        vio_representative_simulator.h
        data_structure/sliding_window.cpp
        data_structure/sliding_window.h
        data_structure/observation.cpp
        data_structure/observation.h
        #        multi_fusion.h
#        multi_fusion2.h
)

target_include_directories(VinsEKF PUBLIC "E:/eigen-3.4.0/eigen-3.4.0")

# 分析工具: 导出 CSV 供 tools/report.html 可视化
add_executable(VinsAnalysis tools/analysis_main.cpp
        data_structure/frame.cpp
        data_structure/landmark.cpp
        data_structure/feature.cpp
        data_structure/map.cpp
        data_structure/pool.cpp
        vio_frontend_simulator.cpp
        vio_frontend_simulator1.cpp
        vio_representative_simulator.cpp
        eskf/schur_vins.cpp
        data_structure/sliding_window.cpp
        data_structure/observation.cpp
)
target_include_directories(VinsAnalysis PUBLIC "E:/eigen-3.4.0/eigen-3.4.0")

# 报告生成器: 把 out/*.csv 嵌进自包含的 HTML(无外部依赖)
add_executable(VinsReport tools/make_report.cpp tools/report_template.cpp)

# 可观性基的独立数值检查：验证左乘姿态误差下平移/偏航零空间的符号。
add_executable(ObservabilityBasisCheck tools/observability_basis_check.cpp)
target_include_directories(ObservabilityBasisCheck PUBLIC "E:/eigen-3.4.0/eigen-3.4.0")

#target_link_libraries(VinsEKF
##        ${OpenCV_LIBS}
##        ${CERES_LIBRARIES}
##        ${Pangolin_LIBRARIES}
##        OpenMP::OpenMP_CXX
#)
