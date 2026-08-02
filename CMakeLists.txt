cmake_minimum_required(VERSION 3.29)
project(VinsEKF)

set(CMAKE_CXX_STANDARD 20)

# Visual posterior scheduling policy. The corresponding public numeric macros
# live in eskf/visual_update_scheduler.h; this cache string keeps command-line
# builds readable while still producing a compile-time switch.
set(SCHUR_VIO_VISUAL_SCHEDULER "MSCKF" CACHE STRING
    "Visual scheduler: LEGACY, SCHURVINS, MSCKF, VINS_MONO, or RDVIO")
set_property(CACHE SCHUR_VIO_VISUAL_SCHEDULER PROPERTY STRINGS
             LEGACY SCHURVINS MSCKF VINS_MONO RDVIO)
string(TOUPPER "${SCHUR_VIO_VISUAL_SCHEDULER}" SCHUR_VIO_VISUAL_SCHEDULER_UPPER)
if(SCHUR_VIO_VISUAL_SCHEDULER_UPPER STREQUAL "LEGACY")
    set(SCHUR_VIO_VISUAL_SCHEDULER_VALUE 0)
elseif(SCHUR_VIO_VISUAL_SCHEDULER_UPPER STREQUAL "SCHURVINS")
    set(SCHUR_VIO_VISUAL_SCHEDULER_VALUE 1)
elseif(SCHUR_VIO_VISUAL_SCHEDULER_UPPER STREQUAL "MSCKF")
    set(SCHUR_VIO_VISUAL_SCHEDULER_VALUE 2)
elseif(SCHUR_VIO_VISUAL_SCHEDULER_UPPER STREQUAL "VINS_MONO")
    set(SCHUR_VIO_VISUAL_SCHEDULER_VALUE 3)
elseif(SCHUR_VIO_VISUAL_SCHEDULER_UPPER STREQUAL "RDVIO")
    set(SCHUR_VIO_VISUAL_SCHEDULER_VALUE 4)
else()
    message(FATAL_ERROR "Unknown SCHUR_VIO_VISUAL_SCHEDULER=${SCHUR_VIO_VISUAL_SCHEDULER}")
endif()

# Frame selection and clone-retention policy. AUTO selects the recommended
# scheduler-specific behavior, while explicit policies allow a fixed visual
# posterior (for example MSCKF) to compare frame strategies independently.
set(SCHUR_VIO_FRAME_POLICY "AUTO" CACHE STRING
    "Frame policy: AUTO, KEYFRAME_ONLY, FIFO, KEYFRAME_PRIORITY, VINS_MONO, or RDVIO")
set_property(CACHE SCHUR_VIO_FRAME_POLICY PROPERTY STRINGS
             AUTO KEYFRAME_ONLY FIFO KEYFRAME_PRIORITY VINS_MONO RDVIO)
string(TOUPPER "${SCHUR_VIO_FRAME_POLICY}" SCHUR_VIO_FRAME_POLICY_UPPER)
if(SCHUR_VIO_FRAME_POLICY_UPPER STREQUAL "AUTO")
    set(SCHUR_VIO_FRAME_POLICY_VALUE 0)
elseif(SCHUR_VIO_FRAME_POLICY_UPPER STREQUAL "KEYFRAME_ONLY")
    set(SCHUR_VIO_FRAME_POLICY_VALUE 1)
elseif(SCHUR_VIO_FRAME_POLICY_UPPER STREQUAL "FIFO")
    set(SCHUR_VIO_FRAME_POLICY_VALUE 2)
elseif(SCHUR_VIO_FRAME_POLICY_UPPER STREQUAL "KEYFRAME_PRIORITY")
    set(SCHUR_VIO_FRAME_POLICY_VALUE 3)
elseif(SCHUR_VIO_FRAME_POLICY_UPPER STREQUAL "VINS_MONO")
    set(SCHUR_VIO_FRAME_POLICY_VALUE 4)
elseif(SCHUR_VIO_FRAME_POLICY_UPPER STREQUAL "RDVIO")
    set(SCHUR_VIO_FRAME_POLICY_VALUE 5)
else()
    message(FATAL_ERROR "Unknown SCHUR_VIO_FRAME_POLICY=${SCHUR_VIO_FRAME_POLICY}")
endif()

# Zero selects the policy's historical clone budget. A positive value makes
# clone count identical across frame-policy ablations; Map::N_WIN is 30, so 29
# is the largest safe retained count with one slot left for the incoming clone.
set(SCHUR_VIO_FRAME_WINDOW_SIZE "0" CACHE STRING
    "Retained clone count for frame-policy comparison; 0 uses the policy default")
if(NOT SCHUR_VIO_FRAME_WINDOW_SIZE MATCHES "^[0-9]+$")
    message(FATAL_ERROR "SCHUR_VIO_FRAME_WINDOW_SIZE must be an integer in [0, 29]")
endif()
if(SCHUR_VIO_FRAME_WINDOW_SIZE GREATER 29)
    message(FATAL_ERROR "SCHUR_VIO_FRAME_WINDOW_SIZE must be <= 29")
endif()

# The map continues to store world XYZ. This option changes only the local
# three-DOF landmark coordinates used while constructing the Schur system.
set(SCHUR_VIO_LANDMARK_PARAMETERIZATION "WORLD_XYZ" CACHE STRING
    "Landmark coordinates: WORLD_XYZ, ANCHORED_XYZ, ANCHORED_INV_DEPTH, or ANCHORED_LOG_DEPTH")
set_property(CACHE SCHUR_VIO_LANDMARK_PARAMETERIZATION PROPERTY STRINGS
             WORLD_XYZ ANCHORED_XYZ ANCHORED_INV_DEPTH ANCHORED_LOG_DEPTH)
string(TOUPPER "${SCHUR_VIO_LANDMARK_PARAMETERIZATION}"
       SCHUR_VIO_LANDMARK_PARAMETERIZATION_UPPER)
if(SCHUR_VIO_LANDMARK_PARAMETERIZATION_UPPER STREQUAL "WORLD_XYZ")
    set(SCHUR_VIO_LANDMARK_PARAMETERIZATION_VALUE 0)
elseif(SCHUR_VIO_LANDMARK_PARAMETERIZATION_UPPER STREQUAL "ANCHORED_XYZ")
    set(SCHUR_VIO_LANDMARK_PARAMETERIZATION_VALUE 1)
elseif(SCHUR_VIO_LANDMARK_PARAMETERIZATION_UPPER STREQUAL "ANCHORED_INV_DEPTH")
    set(SCHUR_VIO_LANDMARK_PARAMETERIZATION_VALUE 2)
elseif(SCHUR_VIO_LANDMARK_PARAMETERIZATION_UPPER STREQUAL "ANCHORED_LOG_DEPTH")
    set(SCHUR_VIO_LANDMARK_PARAMETERIZATION_VALUE 3)
else()
    message(FATAL_ERROR
        "Unknown SCHUR_VIO_LANDMARK_PARAMETERIZATION=${SCHUR_VIO_LANDMARK_PARAMETERIZATION}")
endif()

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
        eskf/landmark_parameterization.cpp
        eskf/landmark_parameterization.h
        eskf/frame_selection_policy.cpp
        eskf/frame_selection_policy.h
        eskf/rdvio_constraints.cpp
        eskf/rdvio_constraints.h
        eskf/rdvio_scheduler.cpp
        eskf/rdvio_scheduler.h
        eskf/schur_vins.cpp
        eskf/schur_vins_imu.cpp
        eskf/schur_vins_shadow.cpp
        eskf/schur_vins_triangulation.cpp
        eskf/schur_vins_visual.cpp
        eskf/schur_vins.h
        eskf/visual_update_scheduler.h
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
target_compile_definitions(VinsEKF PRIVATE
        SCHUR_VIO_VISUAL_SCHEDULER=${SCHUR_VIO_VISUAL_SCHEDULER_VALUE}
        SCHUR_VIO_FRAME_POLICY=${SCHUR_VIO_FRAME_POLICY_VALUE}
        SCHUR_VIO_FRAME_WINDOW_SIZE=${SCHUR_VIO_FRAME_WINDOW_SIZE}
        SCHUR_VIO_LANDMARK_PARAMETERIZATION=${SCHUR_VIO_LANDMARK_PARAMETERIZATION_VALUE})

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
        eskf/landmark_parameterization.cpp
        eskf/frame_selection_policy.cpp
        eskf/rdvio_constraints.cpp
        eskf/rdvio_scheduler.cpp
        eskf/schur_vins.cpp
        eskf/schur_vins_imu.cpp
        eskf/schur_vins_shadow.cpp
        eskf/schur_vins_triangulation.cpp
        eskf/schur_vins_visual.cpp
        data_structure/sliding_window.cpp
        data_structure/observation.cpp
)
target_include_directories(VinsAnalysis PUBLIC "E:/eigen-3.4.0/eigen-3.4.0")
target_compile_definitions(VinsAnalysis PRIVATE
        SCHUR_VIO_VISUAL_SCHEDULER=${SCHUR_VIO_VISUAL_SCHEDULER_VALUE}
        SCHUR_VIO_FRAME_POLICY=${SCHUR_VIO_FRAME_POLICY_VALUE}
        SCHUR_VIO_FRAME_WINDOW_SIZE=${SCHUR_VIO_FRAME_WINDOW_SIZE}
        SCHUR_VIO_LANDMARK_PARAMETERIZATION=${SCHUR_VIO_LANDMARK_PARAMETERIZATION_VALUE})

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
