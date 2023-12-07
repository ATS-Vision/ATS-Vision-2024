#ifndef ARMOR_STRUCT_HPP_
#define ARMOR_STRUCT_HPP_

#include "enum.hpp"
#include "../../../tool/tool.hpp"
#include "../../../tool/param_struct.hpp"



struct ArmorObject : Object
{
    int area;
    cv::Point2f apex[4];
};

struct Armor : ObjectBase
{
    int area;
    Rect roi;
    Rect rect;
    Point2f apex2d[4];
    RotatedRect rrect;
    cv::Point2f center2d;
    TargetType type;
    bool is_front;
    double rangle;

    Armor()
    {
        is_front = false;
    }
};

struct GyroParam
{
    int max_dead_buffer;  //允许因击打暂时熄灭的装甲板的出现次数
    double max_delta_dist;   //连续目标距离变化范围
    double max_delta_t;      //tracker未更新的时间上限，过久则删除
    double switch_max_dt;    //目标陀螺状态未更新的时间上限，过久则删除
    double hero_danger_zone; //英雄危险距离

    double anti_spin_judge_high_thresh; //大于该阈值认为该车已开启陀螺
    double anti_spin_judge_low_thresh;  //小于该阈值认为该车已关闭陀螺
    double anti_spin_max_r_multiple;

    double delta_x_3d_high_thresh;
    double delta_x_3d_higher_thresh;
    double delta_x_3d_low_thresh;
    double delta_x_3d_lower_thresh;

    double delta_y_3d_high_thresh;
    double delta_y_3d_higher_thresh;
    double delta_y_3d_low_thresh;
    double delta_y_3d_lower_thresh;

    double max_rotation_angle;
    double min_rotation_angle;
    double max_hop_period;
    double max_conf_dis;
    GyroParam()
    {
        max_delta_t = 100;
        switch_max_dt = 2000.0;
        delta_x_3d_high_thresh = 0.18;
        delta_x_3d_higher_thresh = 0.25;
        delta_x_3d_low_thresh = 0.10;
        delta_x_3d_lower_thresh = 0.05;

        delta_y_3d_high_thresh = 0.18;
        delta_y_3d_higher_thresh = 0.25;
        delta_y_3d_low_thresh = 0.10;
        delta_y_3d_lower_thresh = 0.05;

        max_rotation_angle = 10.0 * (M_PI / 180);
        min_rotation_angle = 3.0 * (M_PI / 180);
        max_hop_period = 3.0;
        max_conf_dis = 3.5;
    }

};

struct TimeInfo
{
    double last_timestamp;
    double new_timestamp;
};

struct DetectorInfo
{
    double last_x;
    double new_x;
    double last_y;
    double new_y;
    double last_add_tracker_timestamp;
    double new_add_tracker_timestamp;
};

struct GyroInfo
{
    double last_x_font;
    double last_x_back;
    double new_x_font;
    double new_x_back;
    double last_timestamp;
    double new_timestamp;
    double last_y_font;
    double last_y_back;
    double new_y_font;
    double new_y_back;
    Eigen::Matrix3d last_rmat;
    Eigen::Matrix3d new_rmat;
};

struct SpinState
{
    int64_t switch_timestamp;
    SpinHeading spin_state;
    SpinState()
    {
        switch_timestamp = 0;
        spin_state = UNKNOWN;
    }
};

struct SpinCounter
{
    int flag;
    int normal_gyro_status_counter;
    int switch_gyro_status_counter;
    SpinCounter()
    {
        normal_gyro_status_counter = 0;
        switch_gyro_status_counter = 0;
        flag = 0;
    }
};

struct SpinningMap
{
    std::map<std::string, SpinState> spin_status_map;    //反小陀螺，记录该车小陀螺状态
    std::map<std::string, SpinCounter> spin_counter_map; //记录装甲板旋转帧数，大于0为逆时针旋转，小于0为顺时针

    std::multimap<std::string, TimeInfo> spinning_time_map;
    std::multimap<std::string, GyroInfo> spinning_x_map;
};

#endif