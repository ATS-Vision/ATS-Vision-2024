#include "particle_filter.hpp"

namespace filter
{

    ParticleFilter::ParticleFilter(){}

    ParticleFilter::ParticleFilter(YAML::Node &config,const std::string param_name)
    {
        initParam(config,param_name);
    }

    ParticleFilter::~ParticleFilter(){}

    /**
     * 生成正态分布矩阵
     * **/
    bool ParticleFilter::randomlizedGaussianColwise(Eigen::MatrixXd &matrix, Eigen::MatrixXd &cov)
    {

        std::random_device rd;
        std::default_random_engine e(rd());
        std::vector<std::normal_distribution<double>> normal_distribution_list;

        //假设各个变量不相关
        for (int i = 0; i < cov.cols(); i++)
        {
            std::normal_distribution<double> n(0,cov(i,i));
            normal_distribution_list.push_back(n);
        }


        for (int col = 0; col < matrix.cols(); col++)
        {
            // cout<<normal_distribution_list[col](e)<<endl;
            for(int row = 0; row < matrix.rows(); row++)
            {
                auto tmp = normal_distribution_list[col](e);
                matrix(row, col) = tmp;
                // matrix(row,col) = 1;
            }
        }

        return true;
    }

    ////初始化
    bool ParticleFilter::initParam(YAML::Node &config,const std::string param_name)
    {
        //初始化向量长度与粒子数
        vector_len = config[param_name]["vector_len"].as<int>();
        num_particle = config[param_name]["num_particle"].as<int>();
        Eigen::MatrixXd process_noise_cov_tmp(vector_len,vector_len);
        Eigen::MatrixXd observe_noise_cov_tmp(vector_len,vector_len);
        //初始化过程噪声矩阵
        auto read_vector = config[param_name]["process_noise"].as<std::vector<float>>();
        initMatrix(process_noise_cov_tmp,read_vector);
        process_noise_cov = process_noise_cov_tmp;
        //初始化观测噪声矩阵
        read_vector = config[param_name]["observe_noise"].as<std::vector<float>>();
        initMatrix(observe_noise_cov_tmp,read_vector);
        observe_noise_cov = observe_noise_cov_tmp;
        //初始化粒子矩阵及粒子权重
        // matrix_particle = 3 * Eigen::MatrixXd::Random(num_particle, vector_len);
        matrix_particle = Eigen::MatrixXd::Zero(num_particle, vector_len);
        randomlizedGaussianColwise(matrix_particle, process_noise_cov);
        matrix_weights = Eigen::MatrixXd::Ones(num_particle, 1) / float(num_particle);
        is_ready = false;

        return true;
    }

    bool ParticleFilter::initParam(ParticleFilter parent)
    {
        vector_len = parent.vector_len;
        num_particle = parent.num_particle;
        process_noise_cov = parent.process_noise_cov;
        observe_noise_cov = parent.observe_noise_cov;
        //初始化粒子矩阵及粒子权重
        matrix_particle = Eigen::MatrixXd::Zero(num_particle, vector_len);
        randomlizedGaussianColwise(matrix_particle, process_noise_cov);
        matrix_particle = 3 * Eigen::MatrixXd::Random(num_particle, vector_len);
        matrix_weights = Eigen::MatrixXd::Ones(num_particle, 1) / float(num_particle);
        is_ready = false;

        return true;
    }

    ///进行预测
    Eigen::VectorXd ParticleFilter::predict()
    {
        Eigen::VectorXd particles_weighted = matrix_particle.transpose() * matrix_weights;
        return particles_weighted;
    }

    bool ParticleFilter::update(Eigen::VectorXd measure)
    {
        Eigen::MatrixXd gaussian = Eigen::MatrixXd::Zero(num_particle, vector_len);
        Eigen::MatrixXd mat_measure = measure.replicate(1,num_particle).transpose();
        auto err = ((measure - (matrix_particle.transpose() * matrix_weights)).norm());
        // cout<<num_particle<<" err "<<err<<endl;

        if (is_ready)
        {
            //序列重要性采样
            matrix_weights = Eigen::MatrixXd::Ones(num_particle, 1);
            //按照高斯分布概率密度函数曲线右半侧计算粒子权重
            for(int i = 0; i < matrix_particle.cols(); i++)
            {
                auto sigma = observe_noise_cov(i,i);
                Eigen::MatrixXd weights_dist = (matrix_particle.col(i) - mat_measure.col(i)).rowwise().squaredNorm();
                Eigen::MatrixXd tmp = ((-(weights_dist / pow(sigma, 2)) / matrix_particle.cols()).array().exp() / (sqrt(CV_2PI) * sigma)).array();
                matrix_weights = matrix_weights.array() * tmp.array();
            }
            matrix_weights /= matrix_weights.sum();
            double n_eff = 1.0 / (matrix_weights.transpose() * matrix_weights).value();
            //TODO:有效粒子数少于一定值时进行重采样,该值需在实际调试过程中修改
            // if (n_eff < 0.5 * num_particle)
            if (err > observe_noise_cov(0,0) || (n_eff < 0.5 * num_particle))
            {
                // cout<<"res"<<num_particle<<endl;
                resample();
            }
        }
        else
        {
            matrix_particle+=mat_measure;
            is_ready = true;
            return false;
        }
        return true;
    }

    bool ParticleFilter::resample()
    {

        //重采样采用低方差采样,复杂度为O(N),较轮盘法的O(NlogN)更小,实现可参考<Probablistic Robotics>
        std::random_device rd;
        std::default_random_engine e(rd());
        std::uniform_real_distribution<> random {0.0, 1. / num_particle};

        int i = 0;
        double c = matrix_weights(0,0);
        auto r = random(e);
        Eigen::MatrixXd matrix_particle_tmp = matrix_particle;

        for (int m = 0; m < num_particle; m++)
        {
            auto u = r + m * (1. / num_particle);
            // 当 u > c 不进行采样
            while (u > c)
            {
                i++;
                c = c + matrix_weights(i, 0);
            }
            matrix_particle_tmp.row(m) = matrix_particle.row(i);
        }
        Eigen::MatrixXd gaussian = Eigen::MatrixXd::Zero(num_particle, vector_len);
        randomlizedGaussianColwise(gaussian, process_noise_cov);
        matrix_particle = matrix_particle_tmp + gaussian;
        matrix_weights = Eigen::MatrixXd::Ones(num_particle, 1) / float(num_particle);
        return true;
    }

}