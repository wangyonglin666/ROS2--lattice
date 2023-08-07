#ifndef LATTICE_PLANNER_H_
#define LATTICE_PLANNER_H_

// #include "path_planning_lattice/cpprobotics_types.h"

#include <stdint.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <lgsvl_msgs/msg/vehicle_control_data.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

#include "path_planning_lattice/frenet_optimal_trajectory.h"
#include "path_planning_lattice/frenet_path.h"
#include "path_planning_lattice/lqr_controller.h"
#include "path_planning_lattice/pid_controller.h"
#include "path_planning_lattice/reference_line.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace shenlan::control;
// using namespace shenlan;
// using namespace ros_viz_tools;
using namespace std;
// using namespace zjlmap;

template <typename U, typename V>
double DistanceXY(const U &u, const V &v) {
    return std::hypot(u.x - v.x, u.y - v.y);
}

class PathPlanningNode : public rclcpp::Node {
  public:
    PathPlanningNode();
    ~PathPlanningNode();
    bool init();
    auto createQuaternionMsgFromYaw(double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }

  private:
    // 声明定时器：车辆控制器信号定时调用 全局路径广播定时 重规划算法定时调用
    rclcpp::TimerBase::SharedPtr vehicle_control_iteration_timer;
    void VehicleControllerIterationCallback();

    rclcpp::TimerBase::SharedPtr global_path_publish_timer;
    void GlobalPathPublishCallback();

    rclcpp::TimerBase::SharedPtr lattice_planner_timer;
    void LatticePlannerCallback();

    // 声明广播器：控制信号广播 历史路径广播 全局路径广播 重规划路径广播
    rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr vehicle_control_publisher;
    lgsvl_msgs::msg::VehicleControlData control_cmd;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr history_path_visualization_publisher;
    nav_msgs::msg::Path history_path;
    geometry_msgs::msg::PoseStamped history_path_points;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
    nav_msgs::msg::Path global_path;
    geometry_msgs::msg::PoseStamped this_pose_stamped;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr replan_path_publisher_;

    // 订阅器以及订阅回调函数和广播消息类型
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_data_subscriber;
    void OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr lacalization_data_imu_subscriber;
    void IMUCallback(sensor_msgs::msg::Imu::SharedPtr msg);

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_gps_vehicle;

    //加载路网地图，并设置轨迹的速度信息
    bool loadRoadmap(const std::string &roadmap_path, const double target_speed);
    //将路网转换成可视化的marker数据
    // void addRoadmapMarker(const std::vector<TrajectoryPoint> &path, const std::string &frame_id);

    // void addLocalTrajectoryMarker(const std::vector<TrajectoryPoint> &path, const std::string &frame_id);
    //计算两点之间的距离
    double pointDistance(const TrajectoryPoint &point, const double x, const double y) {
        double dx = point.x - x;
        double dy = point.y - y;
        return sqrt(dx * dx + dy * dy);
    }
    double pointDistance(const Poi_f &point, const double x, const double y) {
        double dx = point[0] - x;
        double dy = point[1] - y;
        return sqrt(dx * dx + dy * dy);
    }
    double pointDistance(const double x1, const double y1, const double x, const double y) {
        double dx = x1 - x;
        double dy = y1 - y;
        return sqrt(dx * dx + dy * dy);
    }

    TrajectoryData GetTrajectoryFromFrenetPath(const FrenetPath &path);

    // void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> &local_plan) const;

    Spline2D CreateRefrenceSpiline();

    void UpdateStaticObstacle();

    void GetWayPoints();
    void GenerateGlobalPath();

    int GetNearestReferenceIndex(const VehicleState &ego_state);    // 根据车辆的当前位置，获取与参考路劲最近点的id

    double GetNearestReferenceLength(const VehicleState &ego_state);
    double GetNearestReferenceLatDist(const VehicleState &ego_state);
    bool LeftOfLine(const VehicleState &p, const geometry_msgs::msg::PoseStamped &p1, const geometry_msgs::msg::PoseStamped &p2);

    // void LoadReferenceLine();

  private:
    VehicleState vehicleState_;

    double targetSpeed_ = 5;
    std::unique_ptr<shenlan::control::PIDController> pid_controller_longitudinal;
    std::unique_ptr<shenlan::control::LqrController> lqr_controller_lateral;
    double controlFrequency_ = 100;    //控制频率
    double plannerFrequency_ = 10;     //规划频率
    TrajectoryData planningNodePublishedTrajectory_;
    TrajectoryData planningPublishedTrajectoryDebug_;    //规划下发的轨迹
    TrajectoryData last_trajectory_;                     //规划下发的轨迹
    TrajectoryData planningPublishedTrajectory_;         //跟踪的轨迹
    TrajectoryPoint goalPoint_;                          //终点
    double goalTolerance_ = 0.5;                         //到终点的容忍距离
    bool isReachGoal_ = false;
    bool firstRecord_ = true;
    bool plannerFlag_ = false;

    std::vector<Poi_f> obstcle_list_;

    FrenetInitialConditions frenet_init_conditions_;

    Vec_f wx_, wy_;

    Spline2D *csp_obj_;

    float c_speed_ = 10.0 / 3.6;
    float c_d_ = 2.0;
    float c_d_d_ = 0.0;
    float c_d_dd_ = 0.0;
    float s0_ = 0.0;

    nav_msgs::msg::Path global_plan_;
    // nav_msgs::msg::Path global_plan;
    std::string frame_id_;

    std::string map_path_;

    double end_x_, end_y_, end_s_;
    bool near_goal_ = false;
    bool use_reference_line_ = false;
};
#endif /* __LQR_CONTROLLER_NODE_H__ */
