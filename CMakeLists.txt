cmake_minimum_required(VERSION 3.15)

project(run)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")
set(OpenCV_DIR /home/nsj/env/network/intel/openvino/thirdparty/fluid/modules/gapi/include/opencv2)
find_package(OpenCV 4.2.0)
# set(INC_DIR //home/nsj/env/network/intel/openvino/src/inference/include)
# set(LINK_DIR /home/nsj/.local/share/Trash/files/openvino/src/inference/include)
set(CMAKE_PREFIX_PATH /home/nsj/env/network/intel/openvino/build)
find_package(InferenceEngine REQUIRED)
# include_directories(${InferenceEngine_INCLUDE_DIRS})
# include_directories(${INC_DIR})
# link_directories(${LINK_DIR})
# link_libraries(libopenvino.so)
##寻找矩阵库
find_package(Eigen3 REQUIRED)

# Find system yaml-cpp
# find_package(yaml-cpp REQUIRED )
add_definitions(-DUSE_ROS_YAML_CPP=0)
include_directories(/usr/local/include/)
find_library(YAML_CPP_LIB yaml-cpp /usr/local/lib)
find_package(yaml-cpp REQUIRED)

# 定义CeresConfig.cmake所在文件目录
set(Ceres_DIR ${Ceres_LIBS_DIR}/cmake/Ceres)
find_package(Ceres REQUIRED)

find_package(glog REQUIRED)
find_package(fmt REQUIRED)

find_package(PythonLibs REQUIRED)
find_package(matplotlib_cpp REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${InferenceEngine_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})
include_directories(
        ${GLOB_INCLUDE_DIRS}
)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(
                run
                main.cpp
                ./tool/tool.hpp
                ./tool/tool.cpp
                thread/thread.cpp
                thread/thread.hpp
                camera/DaHeng/GxCamera.cpp
                camera/DaHeng/GxSDK/DxImageProc.h
                camera/DaHeng/GxCamera.h
                camera/DaHeng/GxSDK/GxIAPI.h
                serial/seial.hpp
                serial/seial.cpp
                coordsolver/coordsolver.hpp
                coordsolver/coordsolver.cpp
                filter/motion_model/motion_model.hpp
                filter/motion_model/motion_model.cpp
                filter/particle_filter/particle_filter.cpp
                filter/particle_filter/particle_filter.hpp
                filter/model_tool/model_tool.cpp
                filter/model_tool/model_tool.hpp
                filter/kalman_filter/kalman_filter.cpp
                filter/kalman_filter/kalman_filter.hpp
                filter/imm/imm.cpp filter/imm/imm.hpp
                filter/param_struct/param_struct.hpp
                autoaim/detector/inference/inference.hpp
                tool/param_struct.hpp
                autoaim/detector/param_struct/param_struct.hpp
                autoaim/detector/inference/inference.cpp
                autoaim/detector/spinning_detector/spinning_detector.cpp
                autoaim/detector/spinning_detector/spinning_detector.hpp
                autoaim/detector/tracker/tracker.cpp
                autoaim/detector/tracker/tracker.hpp
                autoaim/autoaim.cpp
                tool/enum.hpp
                autoaim/predictor/param/fitting.hpp
                autoaim/predictor/param/enum.hpp
                autoaim/predictor/param/param_struct.hpp
                autoaim/predictor/processor/processor.cpp
                autoaim/predictor/armor_prediction/armor_prediction.hpp
                autoaim/predictor/armor_prediction/armor_prediction.cpp
        )


target_link_libraries(run ${OpenCV_LIBRARIES} ${InferenceEngine_LIBRARIES} ${Eigen3_LIBRARIES}
yaml-cpp gxiapi Ceres::ceres fmt::fmt matplotlib_cpp::matplotlib_cpp ${GLOG_LIBRARIES} ${YAML_CPP_LIBRARIES})


