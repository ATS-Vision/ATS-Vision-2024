#include "coordsolver.hpp"

CoordSolver::CoordSolver(){
}

CoordSolver::~CoordSolver() {
}

/////加载数据
bool CoordSolver::loadParam(string coord_path, string param_name) {
    YAML::Node config = YAML::LoadFile(coord_path);

    Eigen::MatrixXd mat_intrinsic(3, 3);
    Eigen::MatrixXd mat_ic(4, 4);
    Eigen::MatrixXd mat_ci(4, 4);
    Eigen::MatrixXd mat_coeff(1, 5);
    Eigen::MatrixXd mat_xyz_offset(1, 3);
    Eigen::MatrixXd mat_t_iw(1, 3);
    Eigen::MatrixXd mat_angle_offset(1, 2);

    //初始化弹道补偿参数
    max_iter = config[param_name]["max_iter"].as<int>();
    stop_error = config[param_name]["stop_error"].as<float>();
    R_K_iter = config[param_name]["R_K_iter"].as<int>();

    //初始化内参矩阵
    auto read_vector = config[param_name]["Intrinsic"].as<vector<float>>();
    initMatrix(mat_intrinsic, read_vector);
    eigen2cv(mat_intrinsic, intrinsic);

    //初始化畸变矩阵
    read_vector = config[param_name]["Coeff"].as<vector<float>>();
    initMatrix(mat_coeff, read_vector);
    eigen2cv(mat_coeff, dis_coeff);

    read_vector = config[param_name]["T_iw"].as<vector<float>>();
    initMatrix(mat_t_iw, read_vector);
    t_iw = mat_t_iw.transpose();

    read_vector = config[param_name]["xyz_offset"].as<vector<float>>();
    initMatrix(mat_xyz_offset, read_vector);
    xyz_offset = mat_xyz_offset.transpose();

    read_vector = config[param_name]["angle_offset"].as<vector<float>>();
    initMatrix(mat_angle_offset, read_vector);
    angle_offset = mat_angle_offset.transpose();

    read_vector = config[param_name]["T_ic"].as<vector<float>>();
    initMatrix(mat_ic, read_vector);
    transform_ic = mat_ic;

    read_vector = config[param_name]["T_ci"].as<vector<float>>();
    initMatrix(mat_ci, read_vector);
    transform_ci = mat_ci;

    return true;
}

///解算
PnPInfo CoordSolver::pnp(const std::vector<Point2f> &points_pic, const Eigen::Matrix3d &rmat_imu, enum TargetType type,
                         int method = SOLVEPNP_IPPE) {

    std::vector<Point3d> points_world;

    //长度为4进入装甲板模式

    //大于长宽比阈值使用大装甲板世界坐标
    if (type == BIG) {
        points_world = {
                {-0.1125, 0.027,  0},
                {-0.1125, -0.027, 0},
                {0.1125,  -0.027, 0},
                {0.1125,  0.027,  0}};
    } else if (type == SMALL) {
        points_world = {
                {-0.066, 0.027,  0},
                {-0.066, -0.027, 0},
                {0.066,  -0.027, 0},
                {0.066,  0.027,  0}};
    }
        //长度为5进入大符模式
    else if (type == BUFF) {
        points_world = {
                {-0.1125, 0.027,  0},
                {-0.1125, -0.027, 0},
                {0,       -0.7,   -0.05},
                {0.1125,  -0.027, 0},
                {0.1125,  0.027,  0}};
    }
    Mat rvec;
    Mat rmat;
    Mat tvec;
    Eigen::Matrix3d rmat_eigen;
    Eigen::Vector3d R_center_world = {0, -0.7, -0.05};
    Eigen::Vector3d tvec_eigen;


    solvePnP(points_world, points_pic, intrinsic, dis_coeff, rvec, tvec, false, method);

    PnPInfo result;
    Rodrigues(rvec, rmat);
    cv2eigen(rmat, rmat_eigen);
    cv2eigen(tvec, tvec_eigen);
    if (type == BIG || type == SMALL) {
        result.armor_cam = tvec_eigen;
        result.armor_world = camToWorld(result.armor_cam, rmat_imu);
        result.euler = rotationMatrixToEulerAngles(rmat_eigen);
    } else {
        result.armor_cam = tvec_eigen;
        result.armor_world = camToWorld(result.armor_cam, rmat_imu);
        result.R_cam = (rmat_eigen * R_center_world) + tvec_eigen;
        result.R_world = camToWorld(result.R_cam, rmat_imu);
        Eigen::Matrix3d rmat_eigen_world = rmat_imu * (transform_ic.block(0, 0, 3, 3) * rmat_eigen);
        result.euler = rotationMatrixToEulerAngles(rmat_eigen_world);
        result.rmat = rmat_eigen_world;
    }
    return result;
}

/**
 * @brief 静态坐标补偿
 *
 * @param xyz 待补偿坐标
 * @return Eigen::Vector3d 补偿后坐标
 */
inline Eigen::Vector3d CoordSolver::staticCoordOffset(Eigen::Vector3d &xyz) {
    return xyz + xyz_offset;
}

/**
 * @brief 静态角度补偿
 *
 * @param angle 待补偿角度
 * @return Eigen::Vector2d 补偿后角度
 */
inline Eigen::Vector2d CoordSolver::staticAngleOffset(Eigen::Vector2d &angle) {
    return angle + angle_offset;
}

/**
 * 这是因为通常情况下，yaw角是使用x和z坐标的反正切值来计算的，而反正切函数 atan2(y, x) 的返回值
 * 是沿着x轴的方向到点(x, y)的射线与x轴正方向之间的角度。在这个函数中，使用了 xyz[0] 和 xyz[2] 分别表示向量在x和z轴上的分量。
 * 具体来说，atan2(xyz[0], xyz[2]) 返回的是x和z分量形成的平面上的方向与x轴正方向之间的角度，即yaw角。
 * */
inline double CoordSolver::calcYaw(Eigen::Vector3d &xyz) {
    return atan2(xyz[0], xyz[2]) * 180 / CV_PI;
}

/**
 * y 轴上的分量与 x-z 平面上的点之间的夹角，即 pitch 角。使用 atan2 函数可以确保计算的角度值在合适的象限内
 * 最终的负号表示将 pitch 角取负值。这是因为在一些坐标系中，正的 pitch 角通常表示向下旋转，而负的 pitch 角表示向上旋转
 * **/
inline double CoordSolver::calcPitch(Eigen::Vector3d &xyz) {
    return -(atan2(xyz[1], sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2])) * 180 / CV_PI);
}
/**
**
* @brief 计算目标Yaw,Pitch角度
* @return Yaw与Pitch
*/
inline Eigen::Vector2d CoordSolver::calcYawPitch(Eigen::Vector3d &xyz) {
    Eigen::Vector2d angle;
    //Yaw(逆时针)
    //Pitch(目标在上方为正)
    angle << calcYaw(xyz), calcPitch(xyz);
    return angle;
}

/**
 * @brief 计算Pitch轴偏移量
 *
 * @param xyz 坐标
 * @return double Pitch偏移量
 */
inline double CoordSolver::dynamicCalcPitchOffset(Eigen::Vector3d &xyz) {
    //TODO:根据陀螺仪安装位置调整距离求解方式
    //降维，坐标系Y轴以垂直向上为正方向
    auto dist_vertical = xyz[2];
    auto vertical_tmp = dist_vertical;
    auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
    auto pitch = atan(dist_vertical / dist_horizonal) * 180 / CV_PI;
    auto pitch_new = pitch;
    auto pitch_offset = 0.0;
    //开始使用龙格库塔法求解弹道补偿
    for (int i = 0; i < max_iter; i++) {
        //TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
        //初始化
        auto x = 0.0;
        auto y = 0.0;
        auto p = tan(pitch_new / 180 * CV_PI);
        auto v = bullet_speed;
        auto u = v / sqrt(1 + pow(p, 2));
        auto delta_x = dist_horizonal / R_K_iter;
        for (int j = 0; j < R_K_iter; j++) {
            auto k1_u = -k * u * sqrt(1 + pow(p, 2));
            auto k1_p = -g / pow(u, 2);
            auto k1_u_sum = u + k1_u * (delta_x / 2);
            auto k1_p_sum = p + k1_p * (delta_x / 2);

            auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
            auto k2_p = -g / pow(k1_u_sum, 2);
            auto k2_u_sum = u + k2_u * (delta_x / 2);
            auto k2_p_sum = p + k2_p * (delta_x / 2);

            auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
            auto k3_p = -g / pow(k2_u_sum, 2);
            auto k3_u_sum = u + k3_u * (delta_x / 2);
            auto k3_p_sum = p + k3_p * (delta_x / 2);

            auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
            auto k4_p = -g / pow(k3_u_sum, 2);

            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

            x += delta_x;
            y += p * delta_x;
        }
        //评估迭代结果,若小于迭代精度需求则停止迭代
        auto error = dist_vertical - y;
        if (abs(error) <= stop_error) {
            break;
        } else {
            vertical_tmp += error;
            // xyz_tmp[1] -= error;
            pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / CV_PI;
        }
    }
    return pitch_new - pitch;
}

/**
 * @brief 相机坐标系至世界坐标系
 * @param point_camera 相机坐标系下坐标
 * @param rmat 由陀螺仪四元数解算出的旋转矩阵
 * @return 世界坐标系下坐标
 * **/
Eigen::Vector3d CoordSolver::camToWorld(const Eigen::Vector3d &point_camera, const Eigen::Matrix3d &rmat) {
    //升高维度
    Eigen::Vector4d point_camera_tmp;
    Eigen::Vector4d point_imu_tmp;
    Eigen::Vector3d point_imu;
    Eigen::Vector3d point_world;

    point_camera_tmp << point_camera[0], point_camera[1], point_camera[2], 1;
    point_imu_tmp = transform_ic * point_camera_tmp;
    point_imu << point_imu_tmp[0], point_imu_tmp[1], point_imu_tmp[2];
    point_imu -= t_iw;
    return rmat * point_imu;
}

/**
 * @brief 世界坐标系至相机坐标系
 * @param point_world 世界坐标系下坐标
 * @param rmat 由陀螺仪四元数解算出的旋转矩阵
 * @return 相机坐标系下坐标
 * **/
Eigen::Vector3d CoordSolver::worldToCam(const Eigen::Vector3d &point_world, const Eigen::Matrix3d &rmat) {

    Eigen::Vector4d point_camera_tmp;
    Eigen::Vector4d point_imu_tmp;
    Eigen::Vector3d point_imu;
    Eigen::Vector3d point_camera;

    point_imu = rmat.transpose() * point_world;
    point_imu += t_iw;
    point_imu_tmp << point_imu[0], point_imu[1], point_imu[2], 1;
    point_camera_tmp = transform_ci * point_imu_tmp;
    point_camera << point_camera_tmp[0], point_camera_tmp[1], point_camera_tmp[2];

    return point_camera;
}


bool CoordSolver::setBulletSpeed(double speed) {
    bullet_speed = speed;
    return true;
}
