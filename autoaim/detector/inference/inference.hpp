#ifndef INFERENCE_HPP_
#define INFERENCE_HPP_

#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Core>
#include "../param_struct/param_struct.hpp"
#include "../../../tool/tool.hpp"
using namespace cv;
using namespace std;

class ArmorDetector
{
public:
    ArmorDetector();
    ~ArmorDetector();
    bool detect(cv::Mat &src, std::vector<ArmorObject>& objects);
    bool initModel(std::string path);
private:
    int dw, dh;
    float rescale_ratio;  ///缩放因子

    ov::Core core;
    std::shared_ptr<ov::Model> model; // 网络
    ov::CompiledModel compiled_model; // 可执行网络
    ov::InferRequest infer_request;   // 推理请求
    ov::Tensor input_tensor;

    std::string input_name;
    std::string output_name;

    Eigen::Matrix<float, 3, 3> transfrom_matrix;
};

#endif