#ifndef TOOL_HPP_
#define TOOL_HPP_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <fmt/color.h>
#include <fmt/format.h>
#include <unistd.h>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <string>
#include <iostream>
#include <stdio.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
using namespace cv;
using namespace std;

const string camera_sn="KE1980";

//enum TargetType
//{
//    SMALL,
//    BIG,
//    BUFF
//};
struct TaskData
{
    int mode;
    double bullet_speed;
    Mat img;
    Eigen::Quaterniond quat;
    int timestamp;//单位：ms
};

struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};

template<typename T>
bool initMatrix(Eigen::MatrixXd &matrix,std::vector<T> &vector)
{
    int cnt = 0;
    for(int row = 0;row < matrix.rows();row++)
    {
        for(int col = 0;col < matrix.cols();col++)
        {
            matrix(row,col) = vector[cnt];
            cnt++;
        }
    }
    return true;
}

float calcTriangleArea(cv::Point2f pts[3]);
float calcTetragonArea(cv::Point2f pts[4]);
double rangedAngleRad(double &angle);

std::string symbolicToReal(string path);
std::string relativeToFull(string relative,string src);
string treeToPath(std::vector<string> &tree);
string getParent(string path);

std::vector<string> readLines(string file_path);
std::vector<string> generatePathTree(string path);

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);

Eigen::Vector3d calcDeltaEuler(Eigen::Vector3d euler1, Eigen::Vector3d euler2);
Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler);
Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta);
// 计算两点之间的距离
float distance(const cv::Point2f &p1, const cv::Point2f &p2);
#endif