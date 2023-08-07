// ***Description***:
// Many thanks to the author of the Frenet algorithm here, this paper may be
// very helpful to you, "Optimal Trajectory Generation for Dynamic Street
// Scenarios in a Frenet Frame"
// https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame
// Thanks to open source codes, python robotics, this website can help you
// quickly verify some algorithms, which is very useful for beginners.
// https://github.com/AtsushiSakai/PythonRobotics

#include "path_planning_lattice/frenet_optimal_trajectory.h"

#include <algorithm>
#include <cmath>

// #include "ros/ros.h"

// namespace shenlan {
#define MAX_SPEED 50.0 / 3.6       // maximum speed [m/s]
#define MAX_ACCEL 2.0              // maximum acceleration [m/ss]
#define MAX_CURVATURE 1.0          // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 7.0         // maximum road width [m]
#define D_ROAD_W 1.0               // road width sampling length [m]
#define DT 0.2                     // time tick [s]
#define MAXT 5.0                   // max prediction time [s]
#define MINT 4.0                   // min prediction time [s]
#define TARGET_SPEED 30.0 / 3.6    // target speed [m/s]
#define D_T_S 5.0 / 3.6            // target speed sampling length [m/s]
#define N_S_SAMPLE 1               // sampling number of target speed
#define ROBOT_RADIUS 1.5           // robot radius [m]

#define KJ 0.1
#define KT 0.1
#define KD 1.0
#define KLAT 1.0
#define KLON 1.0

FrenetOptimalTrajectory::FrenetOptimalTrajectory() {}
FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {}

float FrenetOptimalTrajectory::sum_of_power(std::vector<float> value_list) {
    float sum = 0;
    for (float item : value_list) {
        sum += item * item;
    }
    return sum;
};

// 01 获取采样轨迹
Vec_Path FrenetOptimalTrajectory::calc_frenet_paths(float c_speed, float c_d, float c_d_d, float c_d_dd, float s0) {
    std::vector<FrenetPath> fp_list;
    // 根据 当前车辆的速度，当前车辆在frenet坐标系中的s坐标，l坐标，l坐标的一阶导数和二阶导数来采样生成备选轨迹
    // 先遍历 d 方向，再遍历 t 方向，这样可以生成 d-t 曲线，每个d-t曲线下，再遍历备选速度，生成 s-t 曲线，这种方法其实只适用终点 s 自由的方式
    // 完成轨迹采样

    return fp_list;
};

// 02
// 根据参考轨迹与采样的轨迹数组，计算frenet中的其他曲线参数，如航向角，曲率，ds等参数
void FrenetOptimalTrajectory::calc_global_paths(Vec_Path& path_list, Spline2D csp) {
    // 计算采样轨迹的其他参数
   
};

bool FrenetOptimalTrajectory::check_collision(FrenetPath path, const Vec_Poi ob) {
    for (auto point : ob) {
        for (unsigned int i = 0; i < path.x.size(); i++) {
            float dist = std::pow((path.x[i] - point[0]), 2) + std::pow((path.y[i] - point[1]), 2);
            if (dist <= ROBOT_RADIUS * ROBOT_RADIUS) {
                return false;
            }
        }
    }
    return true;
};
// 03
// 检查路径，通过限制做大速度，最大加速度，最大曲率与避障，选取可使用的轨迹数组
Vec_Path FrenetOptimalTrajectory::check_paths(Vec_Path path_list, const Vec_Poi ob) {
    Vec_Path output_fp_list;
    //补全代码

    return output_fp_list;
};

// to-do step 1 finish frenet_optimal_planning
FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(Spline2D csp, float s0, float c_speed, float c_d, float c_d_d, float c_d_dd, Vec_Poi ob) {
    // 01 获取采样轨迹数组
    Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    // 02
    // 根据参考轨迹与采样的轨迹数组，计算frenet中的其他曲线参数，如航向角，曲率，ds等参数
    calc_global_paths(fp_list, csp);

    // 03
    // 检查路径，通过限制最大速度，最大加速度，最大曲率与避障，选取可使用的轨迹数组
    Vec_Path save_paths = check_paths(fp_list, ob);

    float min_cost = std::numeric_limits<float>::max();
    FrenetPath final_path;
    for (auto path : save_paths) {
        if (min_cost >= path.cf) {
            min_cost = path.cf;
            final_path = path;
        }
    }
    return final_path;
};

FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(Spline2D csp, const FrenetInitialConditions& frenet_init_conditions, Vec_Poi ob) {
    float c_speed = frenet_init_conditions.c_speed;
    float c_d = frenet_init_conditions.c_d;
    float c_d_d = frenet_init_conditions.c_d_d;
    float c_d_dd = frenet_init_conditions.c_d_dd;
    float s0 = frenet_init_conditions.s0;

    Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    calc_global_paths(fp_list, csp);
    Vec_Path save_paths = check_paths(fp_list, ob);

    float min_cost = std::numeric_limits<float>::max();
    FrenetPath final_path;
    for (auto path : save_paths) {
        if (min_cost >= path.cf) {
            min_cost = path.cf;
            final_path = path;
        }
    }
    return final_path;
}

// }    // namespace shenlan
