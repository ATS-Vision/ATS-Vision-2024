#ifndef MODEL_TOOL_HPP_
#define MODEL_TOOL_HPP_

#include "../imm/imm.hpp"
namespace filter
{
    class ModelGenerator
    {
    public:
        ModelGenerator();
        // ModelGenerator(IMMParam imm_param);
        ~ModelGenerator();

        static IMMParam imm_param_;
        static std::shared_ptr<IMM> generateIMMModel(const Eigen::VectorXd& x, const double& dt);
        static std::shared_ptr<CV> generateCVModel(const Eigen::VectorXd& x, const double& dt);
        static std::shared_ptr<CA> generateCAModel(const Eigen::VectorXd& x, const double& dt);
        static std::shared_ptr<CT> generateCTModel(const Eigen::VectorXd& x, const double& w, const double& dt);
    };
}

#endif