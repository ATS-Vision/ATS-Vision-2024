#ifndef FILTER_STRUCT_HPP
#define FILTER_STRUCT_HPP

#include <ceres/ceres.h>
#include "../../tool/tool.hpp"

namespace filter
{
    struct KFParam {
        vector<double> process_noise_params;
        vector<double> measure_noise_params;
    };

    struct IMMParam {
        vector<double> imm_model_trans_prob_params;
        vector<double> imm_model_prob_params;
    };
}

#endif