#include "imm.hpp"
namespace filter
{
    IMM::IMM():model_num_(0){}
    IMM::~IMM(){}
    IMM::IMM(const IMM& imm)
    {
        this->transfer_prob_ = imm.transfer_prob_;
        this->model_num_ = imm.model_num_;
        this->P_ = imm.P_;
    }
    ///添加运动模型
    void IMM::addModel(const std::shared_ptr<KalmanFilter>& model)
    {
        this->models_.push_back(model);
        ++this->model_num_;
    }

    /////初始化方法用于设置初始状态  状态向量、状态矩阵、模型概率、传递概率
    void IMM::init(const Eigen::VectorXd& x,
            const Eigen::MatrixXd& P,
            const Eigen::MatrixXd& model_prob,
            const Eigen::MatrixXd& transfer_prob)
    {
        assert(this->model_num_ > 0);
        assert(transfer_prob.cols() == this->model_num_ && transfer_prob.rows() == this->model_num_);
        assert(model_prob.size() == this->model_num_);

        this->state_num_ = x.size();
        this->X_.resize(this->state_num_, this->model_num_);
        for(int ii = 0; ii < this->model_num_; ii++)
        {
            this->X_.col(ii) = this->models_[ii]->x();
        }

        this->c_.resize(this->model_num_);
        this->x_ = x;
        this->P_ = P;
        this->model_prob_ = model_prob;
        this->transfer_prob_ = transfer_prob;
    }


    void IMM::init(
            const Eigen::VectorXd& x,
            const Eigen::MatrixXd& model_prob,
            const Eigen::MatrixXd& transfer_prob)
    {
        assert(this->model_num_ > 0);
        assert(transfer_prob.cols() == this->model_num_ && transfer_prob.rows() == this->model_num_);
        assert(model_prob.size() == this->model_num_);

        this->state_num_ = x.size();
        this->X_.resize(this->state_num_, this->model_num_);
        for(int ii = 0; ii < this->model_num_; ii++)
        {
            this->X_.col(ii) = this->models_[ii]->x();
        }

        this->c_.resize(this->model_num_);
        this->x_ = x;
        this->model_prob_ = model_prob;
        this->transfer_prob_ = transfer_prob;
    }

    void IMM::stateInteraction()
    {
        //1 计算交互后各模型概率
        this->c_ = Eigen::VectorXd::Zero(this->model_num_);
        for(int j = 0; j < this->model_num_; j++)
        {
            for(int i = 0; i < this->model_num_; i++)
            {
                this->c_(j) += this->transfer_prob_(i, j) * this->model_prob_(i);
            }
        }

        Eigen::MatrixXd U = Eigen::MatrixXd::Zero(this->model_num_, this->model_num_);
        for(int ii = 0; ii < this->model_num_; ii++)
        {
            this->X_.col(ii) = this->models_[ii]->x();
        }

        Eigen::MatrixXd X = this->X_;
        this->X_.fill(0);
        //step2
        for(int j = 0; j < this->model_num_; j++)
        {
            for(int i = 0; i < this->model_num_; i++)
            {
                U(i, j) = (1 / this->c_(j)) * this->transfer_prob_(i, j) * this->model_prob_(i);
                this->X_.col(j) += X.col(i) * U(i, j);
            }
        }

        //step3
        for(int i = 0; i < this->model_num_; i++)
        {
            Eigen::MatrixXd P = Eigen::MatrixXd::Zero(this->model_num_, this->model_num_);
            for(int j = 0; j < this->model_num_; j++)
            {
                Eigen::VectorXd s = this->X_.col(i) - this->X_.col(j);
                P += U(i, j) * (this->models_[i]->P() + s * s.transpose());
            }
            this->models_[i]->setStateCoveriance(P);
            this->models_[i]->setState(this->X_.col(i));
        }
    }

    void IMM::updateState()
    {

    }

    ////状态更新
    void IMM::updateState(const Eigen::VectorXd& z, const double& dt)
    {
        for(int i = 0; i < this->model_num_; i++)
        {
            this->models_[i]->Predict(dt);
            // if(z != nullptr)
            this->models_[i]->Update(z, 1);
        }
    }

    ///更新模型概率
    void IMM::updateModelProb()
    {
        //模型概率更新
        double c_sum = 0;
        for(int i = 0; i < this->model_num_; i++)
        {
            c_sum += this->models_[i]->getLikelihoodValue() * this->c_(i);
        }

        for(int i = 0; i < this->model_num_; i++)
        {
            this->model_prob_(i) = (1 / c_sum) * this->models_[i]->getLikelihoodValue() * this->c_(i);
        }
    }

    ///各模型状态估计融合
    void IMM::estimateFusion()
    {
        this->x_ = this->X_ * this->model_prob_;

        for(int i = 0; i < this->model_num_; i++)
        {
            //TODO:
            Eigen::MatrixXd v = this->X_.col(i) - this->x_;
            this->P_ += this->model_prob_[i] * (this->models_[i]->P() + v * v.transpose());
        }
    }

    void IMM::updateOnce(const Eigen::VectorXd& z, const double& dt)
    {
        if (z.isZero()) {
            stateInteraction();
            updateState(z, dt);
            estimateFusion();
        } else {
            stateInteraction();
            updateState(z, dt);
            updateModelProb();
            estimateFusion();
        }
    }

}