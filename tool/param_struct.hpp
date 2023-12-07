#ifndef TOOL_STRUCT_HPP_
#define TOOL_STRUCT_HPP_

#include "tool.hpp"
#include "enum.hpp"
#include <fstream>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <Eigen/Core>
#include <future>
#include <thread>
#include <future>

struct Object
{
    cv::Rect_<float> rect;
    int cls;
    int color;
    float prob;
    std::vector<cv::Point2f> pts;
};

struct ObjectBase
{
    int id;
    int color;
    double conf;
    std::string key;
    Eigen::Vector3d armor3d_cam;
    Eigen::Vector3d armor3d_world;
    Eigen::Vector3d euler;
    Eigen::Matrix3d rmat;
};


#endif