#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <cassert>
#include "../param_struct/param_struct.hpp"

using namespace std;
//using namespace Eigen;
using Eigen::VectorXd;
using Eigen::MatrixXd;
namespace filter {
    class KalmanFilter {
    public:
        KalmanFilter();

        ~KalmanFilter();

        KalmanFilter(KFParam kf_param);

        virtual KalmanFilter *Clone() {
            return new KalmanFilter(*this);
        }

        /////初始化
        void Init(int stateParams, int measureParams, int controlParams);


        ///预测状态向量和协方差矩阵
        void Predict();

        void Predict(const double &dt);

        ///更新状态转移矩阵
        virtual void updateF() {}

        virtual void updateF(Eigen::MatrixXd &Ft, double dt) {
            Ft = F_;
        }

        ///更新量测矩阵
        virtual void updateH() {}

        virtual void updateH(Eigen::MatrixXd &Ht, double dt) {
            Ht = H_;
        }

        ///更新状态向量
        void Update(const Eigen::VectorXd &z);

        void Update(const Eigen::VectorXd &z, int mp);

        void updateOnce(const double &dt, const Eigen::VectorXd *z);

        ///更新EKF
        void UpdateEKF(const Eigen::MatrixXd &z);

        ///系统模型的雅可比矩阵
        virtual void updateJf() {
            this->Jf_ = this->F_;
        }

        virtual void updateJf(Eigen::MatrixXd &Jft, double dt) {
            Jft = Jf_;
        }

        ///量测模型的雅可比矩阵
        virtual void updateJh() {
            this->Jh_ = this->H_;
        }

        virtual void updateJh(Eigen::MatrixXd &Jht, double dt) {
            Jht = Jh_;
        }

        ////返回模型似然值
        double getLikelihoodValue() const;

        Eigen::VectorXd x() const { return this->x_; };

        Eigen::MatrixXd P() const { return this->P_; };

        Eigen::MatrixXd S() const { return this->S_; };

        Eigen::MatrixXd F() const { return this->F_; };

        void setStateCoveriance(const Eigen::MatrixXd &P) {
            this->P_ = P;
        }

        void setState(const Eigen::VectorXd &x) {
            this->x_ = x;
        }

    public:
        //状态向量
        Eigen::VectorXd x_;

        //过程协方差矩阵
        Eigen::MatrixXd P_;

        //状态转移矩阵
        Eigen::MatrixXd F_;

        //测量矩阵
        Eigen::MatrixXd H_;

        //测量协方差矩阵
        Eigen::MatrixXd R_;

        //状态协方差矩阵
        Eigen::MatrixXd Q_;

        //状态转移矩阵的雅可比形式
        Eigen::MatrixXd Jf_;

        //测量矩阵的雅可比形式
        Eigen::MatrixXd Jh_;

        //控制矩阵
        Eigen::MatrixXd C_;

        //残差的协方差矩阵
        Eigen::MatrixXd S_;

        //测量向量
        Eigen::VectorXd z_;


    public:
        double likelihood_; //似然值
        double dt_ = 0.015; //时间量
        int cp_ = 0; //控制量个数

    public:
        KFParam kf_param_;
        void setRCoeff(double &r, int idx);
        void setQCoeff(double &q, int idx);
    };
}

#endif