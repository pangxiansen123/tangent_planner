#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>

#ifndef TANGENT_PLANNER_H
#define TANGENT_PLANNER_H

#define square(x) pow(x, 2)

#define TIME_TO_REACH_MAX_VEL 2.0 // sec

namespace tangent_planner
{
    class TangentPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        TangentPlanner();
        ~TangentPlanner();
        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
        bool isGoalReached();

    private:
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        // 对值进行重新设置，在到达目标点的时候设置
        void resetVariables();
        // 检查odom是否能转换到global_frame
        bool areFramesSetCorrectly();
        // 计算本帧数据的离散点
        void findDiscontinuities();
        void visualizeDiscontinuities();
        // 找到合理的突变点，并且更新goal_angle_
        void selectDiscontinuityPoint();

        void updateFootprint();
        void updateParameters(ros::NodeHandle &n, const std::string &planner_name);
        // 更新机器人的实时位置
        void updatePose();
        // 更新距离目标的位置和目标旋转角与机器人的旋转角的夹角和当前位置和目标连线与机器人旋转角的夹角
        void updateGoal();
        // 对进行将要进行的状态进行更新
        void updateState();
        // 这些都是对kp_和angular_velocity_进行更新
        void updateController();
        // 更新速度，对速度进行设置
        void updateVelocity();
        // 初始化雷达的回调函数和发布显示话题
        void initializeSubPub(ros::NodeHandle &n);
        // 初始化显示信息
        void initializeVisualization();

        bool foundObstacle();
        void followBoundaries();

        // 是否是新的目标
        bool isNewGoal();
        // 是否接近目标
        bool isCloseToGoal();
        // 是否到达目标
        bool isAtGoal();
        bool isAtGoalOrientation();
        // 是否当前位置和目标连线与机器人旋转角的夹角在允许的范围之内
        bool isInGoalDirection();
        // 这个不知道作用是啥，发现局部最小值
        bool foundLocalMinima();

        void normalizeAngle(float &angle);
        // 根据角度，得到这束激光是第几束
        void angleToLaserIndex(float angle, unsigned int &index);
        // 将geometry_msgs::PoseStamped转化为geometry_msgs::Pose2D
        void poseStampedToPose2D(const geometry_msgs::PoseStamped &from, geometry_msgs::Pose2D &to);

        // 发布判断出来的离散点
        ros::Publisher marker_pub_; 
        ros::Subscriber laser_sub_;
        // 存储激光雷达数据
        sensor_msgs::LaserScan::ConstPtr laser_msg_;
        // 存储速度
        geometry_msgs::Twist *cmd_ptr_;
        // 在这个存储的是机器人当前的位置
        geometry_msgs::Pose2D current_pose_;
        // 这个是全局路径，可以进行更改，成为局部路径地图中的路径
        std::vector<geometry_msgs::Pose2D> global_plan_;

        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;
        tf2_ros::Buffer *tf_;
        // base_scan到odom的转换关系
        geometry_msgs::TransformStamped laser_to_base_transform_;

        std::size_t plan_size_;
        unsigned int plan_index_;

        bool initialized_;
        bool is_new_goal_;
        bool goal_reached_;
        bool goal_unreachable_;
        bool tangent_selected_;

        float robot_front_, robot_back_, robot_width_min_, robot_width_max_, robot_width_;
        float max_trans_vel_, max_rot_vel_;

        float controller_frequency_;
        float kp_, kp_i_;

        float obstacle_range_;
        float xy_goal_tolerance_, yaw_goal_tolerance_;

        // 当前位置和目标连线与机器人旋转角的夹角
        float goal_angle_;
        // 距离目标的距离
        float goal_distance_;
        // 目标旋转角与机器人的旋转角的夹角
        float goal_orientation_;
        // 需要赋值给cmd_vel的旋转速度
        float angular_velocity_;
        // prev_heuristic_distance_表示上一时刻的h氏距离
        float heuristic_distance_, prev_heuristic_distance_;
        // 算法中的判断两种模式的依据
        // dfollowed是紧随边界上的点与目标之间的最短距离(从这个特定障碍的边界开始以来的所有感知点中)。
        // dreach为机器人范围内点之间的最短距离(此范围包括紧随障碍物上的点和自由空间内传感器范围内的点)
        float dfollowed_, dreached_;
        // 表示算法中的突变点的距离值
        std::vector<float> discontinuities_ranges_;
        // 表示算法中的突变点的角度值
        std::vector<float> discontinuities_angles_;
        // 这个不太懂什么意思
        std::vector<float> discontinuities_corrections_;

        visualization_msgs::Marker discontinuity_points_;

        enum BugState
        {
            // 两种模式，向目标前进
            MotionToGoal, 
            // 沿着障碍物运动
            FollowBoundary,

            // 下面是别的状态
            TurnToGoal, //开始旋转，旋转到允许角度
            CloseToGoal, // 表示接近目标了
            OrientToGoal, //到达目标点，需要旋转到目标方向
            GoDiscontinuity,
            StopAtGoal //到达目标
        } state_;

    };
}; // namespace tangent_planner

#endif
