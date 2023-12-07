#ifndef PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_HPP_

#include <random>
#include <iostream>
#include "../../tool/tool.hpp"

using namespace std;

namespace filter
{
    class ParticleFilter
    {
    public:
        ParticleFilter(YAML::Node &config,const std::string param_name);
        ParticleFilter();
        ~ParticleFilter();

        Eigen::VectorXd predict();
        bool initParam(YAML::Node &config,const std::string param_name);
        bool initParam(ParticleFilter parent);
        bool update(Eigen::VectorXd measure);
        bool randomlizedGaussianColwise(Eigen::MatrixXd &matrix, Eigen::MatrixXd &cov);
        bool is_ready;
    private:
        bool resample();

        int vector_len;
        int num_particle;

        Eigen::MatrixXd process_noise_cov;
        Eigen::MatrixXd observe_noise_cov;
        Eigen::MatrixXd weights;

        Eigen::MatrixXd matrix_estimate;
        Eigen::MatrixXd matrix_particle;
        Eigen::MatrixXd matrix_weights;
    };
}

#endif