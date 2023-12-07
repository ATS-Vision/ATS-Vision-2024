#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

//c++
#include <ctime>
#include <future>
#include <random>
#include <vector>

#include "./enum.hpp"
#include "../../../tool/tool.hpp"
#include "../../../filter/model_tool/model_tool.hpp"

using namespace filter;


struct FilterModelParam
{
    vector<double> imm_model_trans_prob_params;
    vector<double> imm_model_prob_params;
    vector<double> process_noise_params;
    vector<double> measure_noise_params;
    // vector<double> singer_model_params;
};

struct PredictParam
{
    double bullet_speed;    //弹速
    double shoot_delay;     //射击延迟
    double delay_coeff;     //延迟系数（放大时间提前量，缓解云台跟随滞后问题）
    int max_dt;             //最大时间跨度，大于该值则重置预测器(ms)
    int max_cost;           //回归函数最大cost
    int max_v;              //
    int min_fitting_lens;   //最短队列长度
    int window_size;        //滑窗大小
    filter::KFParam kf_param;       //卡尔曼滤波参数
    FilterModelParam filter_model_param; //滤波模型参数
    SystemModel system_model;
    double reserve_factor;
    double max_offset_value;

    PredictParam()
    {
        bullet_speed = 28;
        shoot_delay = 100.0;
        max_dt = 1000;
        max_cost = 509;
        max_v = 8;
        min_fitting_lens = 10;
        window_size = 3;
        system_model = CSMODEL;
        reserve_factor = 15.0;
        max_offset_value = 0.25;
    }
};

struct DebugParam

{
    bool show_img;
    bool draw_predict;
    bool show_predict;
    bool print_delay;
    bool show_aim_cross;
    bool show_fps;

    DebugParam()
    {
        show_img = false;
        show_fps = false;
        draw_predict = false;
        show_predict = false;
        print_delay = false;
        show_aim_cross = false;
    }
};


struct TargetInfo
{
    Eigen::Vector3d xyz;
    double rangle;
    double dist;
    double timestamp;
    double period;
    bool is_target_lost;
    bool is_target_switched;
    bool is_spinning;
    bool is_spinning_switched;
    bool is_clockwise;
    bool is_outpost_mode;
    SpinningStatus spinning_status;
    SystemModel system_model;
};