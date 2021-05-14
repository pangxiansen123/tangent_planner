#include "tangent_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(tangent_planner::TangentPlanner, nav_core::BaseLocalPlanner)

namespace tangent_planner
{
    TangentPlanner::TangentPlanner() : initialized_(false), goal_reached_(false), tf_(nullptr), costmap_ros_(nullptr),
                                       robot_front_(0), robot_back_(0), robot_width_min_(0), robot_width_max_(0),
                                       plan_size_(0), plan_index_(0), kp_(0), kp_i_(0), is_new_goal_(true),
                                       dreached_(std::numeric_limits<float>::max()), dfollowed_(std::numeric_limits<float>::max()),
                                       heuristic_distance_(0), prev_heuristic_distance_(0)
    {
    }

    TangentPlanner::~TangentPlanner()
    {
        discontinuity_points_.points.clear();
        marker_pub_.publish(discontinuity_points_);
    }

    void TangentPlanner::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        laser_msg_ = msg;
        // 这个就是发现离散点
        findDiscontinuities();
        // 可视化离散点，显示在rviz上
        visualizeDiscontinuities();
    }
    // 初始化雷达的回调函数和发布显示话题
    void TangentPlanner::initializeSubPub(ros::NodeHandle &n)
    {
        laser_sub_ = n.subscribe("/scan", 1, &TangentPlanner::laserCallback, this);
        marker_pub_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }
    // 初始化显示信息
    void TangentPlanner::initializeVisualization()
    {
        // 这个是需要更换的
        discontinuity_points_.header.frame_id = "/base_scan";
        discontinuity_points_.ns = "discontinuity_points";
        discontinuity_points_.action = visualization_msgs::Marker::ADD;
        discontinuity_points_.type = visualization_msgs::Marker::POINTS;
        discontinuity_points_.scale.x = discontinuity_points_.scale.y = 0.05;
        discontinuity_points_.color.b = 1.0;
        discontinuity_points_.color.a = 1.0;
        discontinuity_points_.lifetime = ros::Duration();
    }
    // 显示
    void TangentPlanner::visualizeDiscontinuities()
    {
        discontinuity_points_.points.clear();
        geometry_msgs::Point p;

        for (size_t i = 0; i < discontinuities_ranges_.size(); i++)
        {
            p.x = discontinuities_ranges_.at(i) * cos(discontinuities_angles_.at(i));
            p.y = discontinuities_ranges_.at(i) * sin(discontinuities_angles_.at(i));
            p.z = 0;
            discontinuity_points_.points.push_back(p);
        }

        marker_pub_.publish(discontinuity_points_);
    }

    void TangentPlanner::updateFootprint()
    {
        geometry_msgs::Polygon footprint = costmap_ros_->getRobotFootprintPolygon();

        for (auto p : footprint.points)
        {
            // 机器人x方向的最大值和最小值
            robot_front_ = robot_front_ < p.x ? p.x : robot_front_;
            robot_back_ = robot_back_ > p.x ? p.x : robot_back_;
            // 宽度方向的最大值和最小值
            robot_width_min_ = robot_width_min_ > p.y ? p.y : robot_width_min_;
            robot_width_max_ = robot_width_max_ < p.y ? p.y : robot_width_max_;
        }
        // 机器人的宽度
        robot_width_ = abs(robot_width_min_) + abs(robot_width_max_);
    }

    void TangentPlanner::updateParameters(ros::NodeHandle &n, const std::string &planner_name)
    {
        const std::string local_costmap_name = costmap_ros_->getName();

        try
        {
            n.setParam("recovery_behavior_enabled", false);
            n.setParam("clearing_rotation_allowed", false);
            // 这个参数是在move_base下面进行设置的，所以不用加planner_name
            n.getParam("controller_frequency", controller_frequency_);
            n.getParam(planner_name + "/max_trans_vel", max_trans_vel_);
            n.getParam(planner_name + "/max_rot_vel", max_rot_vel_);
            n.getParam(planner_name + "/xy_goal_tolerance", xy_goal_tolerance_);
            n.getParam(planner_name + "/yaw_goal_tolerance", yaw_goal_tolerance_);
        }
        catch (const ros::InvalidNameException &e)
        {
            ROS_ERROR("Invalid name: %s", e.what());
        }
        // kp_i_更新这个值
        kp_i_ = 1 / (TIME_TO_REACH_MAX_VEL * controller_frequency_);
    }
    // 检查odom是否能转换到global_frame
    bool TangentPlanner::areFramesSetCorrectly()
    {
        const char *global_frame = costmap_ros_->getGlobalFrameID().c_str();

        bool is_correct = false;
        try
        {
            is_correct = tf_->canTransform("odom", global_frame, ros::Time(0), NULL);
            // base_scan这个是scan的tf树，就是base_scan到odom的转换关系
            laser_to_base_transform_ = tf_->lookupTransform("odom", "base_scan", ros::Time(0));
        }
        catch (...)
        {
            ROS_ERROR("Error occured while looking up transformation");
        }
        return is_correct;
    }

    void TangentPlanner::initialize(std::string planner_name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (initialized_)
        {
            ROS_INFO("Already initialized!");
        }
        else
        {
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle priv_nh("~");
            // 初始化雷达的回调函数和发布显示话题
            initializeSubPub(priv_nh);
            // 初始化显示信息
            initializeVisualization();

            updateFootprint();
            updateParameters(priv_nh, planner_name);
            // 判断是否初始化
            if (areFramesSetCorrectly())
            {
                ROS_INFO("The planner is initialized properly!");
                initialized_ = true;
            }
            else
            {
                ROS_ERROR("The tf tree is not set properly between the global frame and the local frame");
                initialized_ = false;
            }
        }
    }

    bool TangentPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            ROS_WARN("Planner is not initialized");
            return false;
        }
        // 表示收到了新的路径
        else if (plan_size_ != plan.size())
        {
            global_plan_.clear();
            plan_size_ = plan.size();
            goal_reached_ = false;
            geometry_msgs::Pose2D pose2d;
            geometry_msgs::PoseStamped poseStamped;
            // 这个是要进行更改的,进行一个循环
            // 这里可以进行更改，实时的更新
            for (auto p : plan)
            {
                try
                {
                    tf_->transform<geometry_msgs::PoseStamped>(p, poseStamped, "odom", ros::Duration(0));
                    poseStampedToPose2D(poseStamped, pose2d);
                    global_plan_.push_back(pose2d);
                }
                catch (const tf2::TransformException &e)
                {
                    ROS_ERROR("%s", e.what());
                }
            }
        }
        return true;
    }
    // 执行各个函数，得到想要的速度
    bool TangentPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_WARN("Planner is not initialized");
            return false;
        }
        // 赋值指针，对指针进行操作
        cmd_ptr_ = &cmd_vel;

        updatePose();
        updateGoal();
        updateState();
        updateController();
        updateVelocity();
        return true;
    }
    // 更新机器人的实时位置
    void TangentPlanner::updatePose()
    {
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseStamped transformed_pose;
        // getRobotPose的值可能有问题，需要改正
        // 问题原因就是costmap2d包进行了更新，函数变了
        costmap_ros_->getRobotPose(pose);

        try
        {
            tf_->transform<geometry_msgs::PoseStamped>(pose, transformed_pose, "odom", ros::Duration(0));
            poseStampedToPose2D(transformed_pose, current_pose_);
        }
        catch (const tf2::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
        }
    }
    // 更新距离目标的位置和目标旋转角与机器人的旋转角的夹角和当前位置和目标连线与机器人旋转角的夹角
    void TangentPlanner::updateGoal()
    {
        float dx = global_plan_.at(plan_size_ - 1).x - current_pose_.x;
        float dy = global_plan_.at(plan_size_ - 1).y - current_pose_.y;
        // 计算距离目标的位置
        goal_distance_ = hypot(dx, dy) - robot_front_;
        // 当前位置和目标连线与机器人旋转角的夹角
        goal_angle_ = atan2(dy, dx) - current_pose_.theta;
        // 将角度控制在[-pi,pi]之间
        normalizeAngle(goal_angle_);
        // 目标旋转角与机器人的旋转角的夹角 
        goal_orientation_ = global_plan_.at(plan_size_ - 1).theta - current_pose_.theta;
        normalizeAngle(goal_orientation_);
        // plan_index_++;
    }
    // 对进行将要进行的状态进行更新
    void TangentPlanner::updateState()
    {
        // 绕边界运行
        if (state_ != BugState::FollowBoundary)
        {
            // 是否转到允许方向 是否到达目标 是否收到新的目标
            if (!isInGoalDirection() && !isAtGoal() && isNewGoal())
            {
                // 旋转到目标
                state_ = BugState::TurnToGoal;
            }
            else if (isInGoalDirection() && isNewGoal())
            {
                // 转到允许方向并且是新目标
                is_new_goal_ = false;
            }
            else if (isAtGoal() && !isAtGoalOrientation())
            {
                //到达目标点，需要旋转到目标方向
                state_ = BugState::OrientToGoal;
            }
            else if ((isInGoalDirection() || !foundObstacle()) && isCloseToGoal())
            {
                // 表示接近目标了
                state_ = BugState::CloseToGoal;
            }
            else if (isAtGoal() && isAtGoalOrientation())
            {
                // 到达目标
                state_ = BugState::StopAtGoal;
            }
            else
            {
                state_ = BugState::MotionToGoal;
                // 如果发现障碍物的话，就执行算法
                if (foundObstacle() && foundLocalMinima())
                {
                    state_ = BugState::FollowBoundary;
                }
            }
        }
    }
    // 这些都是对kp_和angular_velocity_进行更新
    void TangentPlanner::updateController()
    {
        // 表示到达目标
        if (state_ == BugState::StopAtGoal)
        {
            kp_ = 0;
            angular_velocity_ = 0;
            goal_reached_ = true;
            resetVariables();
        }
        // 还没到达目标，并且当前位置和目标连线与机器人旋转角的夹角大于一定范围
        // 所以需要转向夹角
        else if (state_ == BugState::TurnToGoal)
        {
            kp_ = 0;
            // goal_angle_当前位置和目标连线与机器人旋转角的夹角
            normalizeAngle(goal_angle_);
            goal_angle_ = abs(goal_angle_) < yaw_goal_tolerance_ ? 0 : goal_angle_;
            // 需要转向到目标，所以设置旋转速度
            angular_velocity_ = 0.5 * goal_angle_;
        }
        else if (state_ == BugState::OrientToGoal)
        {
            // 旋转到目标方向
            kp_ = 0;
            angular_velocity_ = 0.5 * goal_orientation_;
        }
        // 沿着目标运动，沿着目标点移动的同时检测是否遇到障碍物
        else if (state_ == BugState::MotionToGoal)
        {
            // 发现周围是否有障碍物，只要是0.5m内，都算障碍物
            if (foundObstacle())
            {
                // 找到合理的突变点，并且更新goal_angle_
                selectDiscontinuityPoint();
            }
            kp_ = kp_ + kp_i_;
            kp_ = kp_ < 1.0 ? kp_ : 1.0;
            angular_velocity_ = goal_angle_;
        }
        // 沿着障碍物边界运动
        else if (state_ == BugState::FollowBoundary)
        {
            followBoundaries();
            kp_ = kp_ + kp_i_;
            kp_ = kp_ < 1.0 ? kp_ : 1.0;
            angular_velocity_ = goal_angle_;
        }
        // 靠近目标了
        else if (state_ == BugState::CloseToGoal)
        {
            kp_ = kp_ - kp_i_;
            kp_ = kp_ < 0.25 ? 0.25 : kp_;
            angular_velocity_ = goal_angle_;
        }
    }
    // 根据updateController函数得到的kp_和angular_velocity_，对线速度和角速度进行更新
    void TangentPlanner::updateVelocity()
    {
        cmd_ptr_->angular.z = angular_velocity_;
        cmd_ptr_->angular.z = (cmd_ptr_->angular.z > max_rot_vel_) ? max_rot_vel_ : cmd_ptr_->angular.z;
        cmd_ptr_->linear.x = (max_trans_vel_ - (max_trans_vel_ / max_rot_vel_) * abs(cmd_ptr_->angular.z)) * kp_;
    }
    // 计算本帧数据的离散点
    void TangentPlanner::findDiscontinuities()
    {
        discontinuities_angles_.clear();
        discontinuities_ranges_.clear();
        discontinuities_corrections_.clear();

        float range = 0, next_range = 0;
        float range_diff = 0;
        unsigned int index;
        // 遍历所有的激光数据
        for (float angle = laser_msg_->angle_min; angle < laser_msg_->angle_max - laser_msg_->angle_increment; angle += laser_msg_->angle_increment)
        {
            // 根据角度，得到这束激光是第几束
            angleToLaserIndex(angle, index);
            // 距离机器人的距离，这里将机器人的尺寸考虑进去
            range = laser_msg_->ranges.at(index) - robot_front_;
            // 下一个值达到最大值了没
            index = (++index == laser_msg_->ranges.size()) ? 0 : index;
            next_range = laser_msg_->ranges.at(index) - robot_front_;

            if (std::isnan(range) || range < robot_width_max_ ||
                std::isnan(next_range) || next_range < robot_width_max_)
            {
                continue;
            }
            // 计算两束激光之间的差值
            range_diff = range - next_range;
            // 如果是真值，说明下一个束是遇到障碍物了
            if (range_diff > robot_width_)
            {
                discontinuities_ranges_.push_back(next_range);
                discontinuities_angles_.push_back(angle);
                discontinuities_corrections_.push_back(2 * asin(robot_width_min_ / next_range));
            }
            // 如果是负值，说明本束是遇到障碍物了
            else if (range_diff < -robot_width_)
            {
                discontinuities_ranges_.push_back(range);
                discontinuities_angles_.push_back(angle + laser_msg_->angle_increment);
                discontinuities_corrections_.push_back(2 * asin(robot_width_max_ / range));
            }
        }
    }
    // 找到合理的突变点，并且更新goal_angle_
    void TangentPlanner::selectDiscontinuityPoint()
    {
        prev_heuristic_distance_ = heuristic_distance_;

        tangent_selected_ = false;
        float correction = 0.0;
        float angle = 0.0;
        float obstacle_to_goal_distance = 0.0;
        heuristic_distance_ = std::numeric_limits<float>::max();
        // discontinuities_angles_表示突变点
        for (size_t i = 0; i < discontinuities_angles_.size(); i++)
        {
            // square表示平方
            obstacle_to_goal_distance = sqrt(square(discontinuities_ranges_.at(i)) + square(goal_distance_) - 2 * discontinuities_ranges_.at(i) * goal_distance_ * cos(goal_angle_ - discontinuities_angles_.at(i)));
            // 就是得分值，具体的还没有取研究
            // 找到最小值，并且存储
            if (heuristic_distance_ > obstacle_to_goal_distance + discontinuities_ranges_.at(i))
            {
                heuristic_distance_ = obstacle_to_goal_distance + discontinuities_ranges_.at(i);
                angle = discontinuities_angles_.at(i);
                correction = discontinuities_corrections_.at(i);
                tangent_selected_ = true;
            }
        }

        if (tangent_selected_)
        {
            goal_angle_ = angle + correction;
            normalizeAngle(goal_angle_);
            goal_angle_ = abs(goal_angle_) < yaw_goal_tolerance_ ? 0 : goal_angle_;
        }
    }
    // 没太看懂，就是根据障碍物更新goal_angle_
    void TangentPlanner::followBoundaries()
    {
        unsigned int index;
        float angle = 0;
        float right_angle, left_angle;

        float desired_distance = 0.5;
        double min_distance;
        // 判断依据没太懂
        // dfollowed是紧随边界上的点与目标之间的最短距离(从这个特定障碍的边界开始以来的所有感知点中)。
        // dreach为机器人范围内点之间的最短距离(此范围包括紧随障碍物上的点和自由空间内传感器范围内的点)
        dreached_ = goal_distance_;
        dfollowed_ = (dfollowed_ > dreached_) ? dreached_ : dfollowed_;
        // 当dfollowed_大于dreached_时，执行MotionToGoal
        dreached_ <= dfollowed_ ? state_ = BugState::MotionToGoal : 0;
        // 这个函数的作用就是找每束激光与desired_distance差值的最小距离
        std::vector<float>::const_iterator result = std::min_element(laser_msg_->ranges.begin(), laser_msg_->ranges.end(), [&](float x, float y) {
            return x - desired_distance < y - desired_distance;
        });
        // 返回ranges.begin()与result之间有多少束激光
        index = std::distance(laser_msg_->ranges.begin(), result);
        min_distance = laser_msg_->ranges.at(index);
        angle = index * laser_msg_->angle_increment;

        left_angle = angle + M_PI_2;
        right_angle = angle - M_PI_2;
        // 检查是否为负号，负号的话就是正
        bool direction_sign = std::signbit(min_distance - desired_distance);
        float correction = 0.15 * min_distance / desired_distance;

        if (abs(min_distance - desired_distance) > 0.05)
        {
            left_angle += direction_sign ? correction : correction * (-1);
            right_angle += direction_sign ? correction * (-1) : correction;
        }

        goal_angle_ = right_angle;

        normalizeAngle(goal_angle_);
        goal_angle_ = abs(goal_angle_) < yaw_goal_tolerance_ ? 0 : goal_angle_;
    }
    // 发现周围是否有障碍物，只要是0.5m内，都算障碍物
    bool TangentPlanner::foundObstacle()
    {
        unsigned int index;
        // 根据角度，得到这束激光是第几束
        angleToLaserIndex(goal_angle_, index);
        float range = laser_msg_->ranges.at(index);
        float obstacle_range = 0.5;

        // goal_angle_表示当前位置和目标连线与机器人旋转角的夹角
        float start_angle = goal_angle_ + 2 * atan2(robot_width_min_, obstacle_range);
        float end_angle = goal_angle_ + 2 * atan2(robot_width_max_, obstacle_range);
        // 这个角度范围内的激光束
        for (float angle = start_angle; angle <= end_angle; angle += laser_msg_->angle_increment)
        {
            angleToLaserIndex(angle, index);
            range = laser_msg_->ranges.at(index);

            if (std::isnan(range) || std::isinf(range) || range < 0.001)
            {
                continue;
            }
            else if ((range < obstacle_range && range <= goal_distance_))
            {
                return true;
            }
        }

        return false;
    }
    // 对值进行重新设置，在到达目标点的时候设置
    void TangentPlanner::resetVariables()
    {
        is_new_goal_ = true;
        tangent_selected_ = false;
        heuristic_distance_ = 0;
        prev_heuristic_distance_ = 0;
        dfollowed_ = std::numeric_limits<float>::max();
        dreached_ = std::numeric_limits<float>::max();
    }

    bool TangentPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_WARN("Planner is not initialized");
        }
        else if (goal_reached_)
        {
            ROS_INFO("Goal Reached");
            return true;
        }
        return false;
    }
    // 这个不知道作用是啥，发现局部最小值
    bool TangentPlanner::foundLocalMinima()
    {
        return prev_heuristic_distance_ - heuristic_distance_ > 2 * xy_goal_tolerance_;
    }
    // 是否接近目标
    bool TangentPlanner::isCloseToGoal()
    {
        // 接近目标
        return goal_distance_ < kp_ * max_trans_vel_;
    }
    // 是否到达目标
    bool TangentPlanner::isAtGoal()
    {
        return goal_distance_ < xy_goal_tolerance_;
    }

    bool TangentPlanner::isAtGoalOrientation()
    {
        return abs(goal_orientation_) < yaw_goal_tolerance_;
    }
    // 是否当前位置和目标连线与机器人旋转角的夹角在允许的范围之内
    bool TangentPlanner::isInGoalDirection()
    {
        // 当前位置和目标连线与机器人旋转角的夹角
        return abs(goal_angle_) < yaw_goal_tolerance_;
    }
    // 是否是新的目标
    bool TangentPlanner::isNewGoal()
    {
        return is_new_goal_;
    }
    // 将角度控制在[-pi,pi]之间
    void TangentPlanner::normalizeAngle(float &angle)
    {
        angle = (angle > M_PI) ? angle - 2 * M_PI : angle;
        angle = (angle < -M_PI) ? angle + 2 * M_PI : angle;
    }
    // 根据角度，得到这束激光是第几束
    void TangentPlanner::angleToLaserIndex(float angle, unsigned int &index)
    {
        angle = (angle < laser_msg_->angle_min) ? angle + 2 * M_PI : angle;
        float angle_increment_deg = 180.0 / M_PI * laser_msg_->angle_increment;

        if (angle >= laser_msg_->angle_min && angle <= laser_msg_->angle_max)
        {
            angle = (angle - laser_msg_->angle_min) * (2 * M_PI) / (laser_msg_->angle_max - laser_msg_->angle_min);
            float angle_deg = 180.0 / M_PI * angle;
            float angle_positive = (angle_deg >= 0) ? angle_deg / angle_increment_deg : angle_deg / angle_increment_deg + laser_msg_->ranges.size();
            index = std::lround(angle_positive);
            index = (index == laser_msg_->ranges.size()) ? 0 : index;
        }
    }
    // 将geometry_msgs::PoseStamped转化为geometry_msgs::Pose2D
    void TangentPlanner::poseStampedToPose2D(const geometry_msgs::PoseStamped &from, geometry_msgs::Pose2D &to)
        {
            to.x = from.pose.position.x;
            to.y = from.pose.position.y;
            to.theta = tf2::getYaw<geometry_msgs::Quaternion>(from.pose.orientation);
        }

    