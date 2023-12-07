#ifndef IMM_HPP_
#define IMM_HPP_

#include "../param_struct/param_struct.hpp"
#include "../motion_model/motion_model.hpp"

namespace filter
{
    class IMM
    {
        Eigen::MatrixXd transfer_prob_;  //马尔可夫状态转移矩阵
        Eigen::MatrixXd P_;              //状态协方差矩阵
        Eigen::MatrixXd X_;              //目标状态矩阵

        Eigen::VectorXd c_;              //状态交互后各模型概率
        Eigen::VectorXd model_prob_;     //模型概率
        Eigen::VectorXd x_;              //目标状态向量
        int model_num_;      //模型数量
        int state_num_;      //状态向量元素个数
        std::vector<std::shared_ptr<KalmanFilter>> models_; //多模型

    public:
        IMM();
        ~IMM();
        IMM(const IMM& imm);
        IMM* Clone() { return new IMM(*this); }

        //初始化（目标状态、状态协方差、模型概率和马尔可夫状态转移矩阵）
        void init(const Eigen::VectorXd& x,
                  const Eigen::MatrixXd& P,
                const Eigen::MatrixXd& model_prob,
                const Eigen::MatrixXd& transfer_prob);
        void init(const Eigen::VectorXd& x,
                const Eigen::MatrixXd& model_prob,
                const Eigen::MatrixXd& transfer_prob);
        Eigen::VectorXd x() const { return this->x_; }
        //添加运动模型
        void addModel(const std::shared_ptr<KalmanFilter>& model);

        ////*************************-------------------IMM算法流程---------------*****************************////
        // 1:输入交互
        void stateInteraction();

        // 2:滤波
        void updateState();
        void updateOnce(const Eigen::VectorXd& z, const double& dt);
        void updateState(const Eigen::VectorXd& z, const double& dt);

        // 3:模型概率更新
        void updateModelProb();

        // 4:估计融合
        void estimateFusion();
    };
}

#endif