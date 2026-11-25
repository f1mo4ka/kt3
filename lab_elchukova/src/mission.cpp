#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <iomanip>

struct Point3D {
    std::string name;
    double x, y, z;
};

class MissionNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher goal_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber odom_fallback_sub_;
    ros::Timer log_timer_;

    std::vector<Point3D> waypoints_;
    size_t current_idx_ = 0;

    geometry_msgs::Point current_pos_;
    geometry_msgs::Point prev_pos_;
    bool has_odom_ = false;
    bool mission_started_ = false;

    // Метрики
    double total_distance_ = 0.0;
    ros::Time start_time_;
    ros::Time segment_start_time_;
    std::vector<double> segment_times_;

    // Логика переключения (радиус < 0.5 м, удержание >= 1.0 с)
    ros::Time in_zone_start_time_;
    bool holding_in_zone_ = false;

    std::ofstream csv_file_;

public:
    MissionNode() : pnh_("~") {
        // Чтение параметров A, B, C, D
        std::vector<double> pa, pb, pc, pd;
        pnh_.param<std::vector<double>>("point_a", pa, {-18.0, 0.0, 1.0});
        pnh_.param<std::vector<double>>("point_b", pb, {0.0, 4.0, 1.0});
        pnh_.param<std::vector<double>>("point_c", pc, {18.0, 0.0, 1.0});
        pnh_.param<std::vector<double>>("point_d", pd, {0.0, -4.0, 1.0});

        std::string csv_path;
        pnh_.param<std::string>("csv_path", csv_path, "/home/john/mission_log.csv");

        // Маршрут: B -> C -> D -> A (A - стартовая точка)
        waypoints_ = {
            {"B", pb[0], pb[1], pb[2]},
            {"C", pc[0], pc[1], pc[2]},
            {"D", pd[0], pd[1], pd[2]},
            {"A", pa[0], pa[1], pa[2]}
        };

        // Подготовка CSV файла
        csv_file_.open(csv_path);
        if (csv_file_.is_open()) {
            csv_file_ << "time_from_start,x,y,z,target_idx,target_name,dist_to_goal\n";
            ROS_INFO("[Mission] CSV log created at %s", csv_path.c_str());
        } else {
            ROS_WARN("[Mission] Failed to open CSV file!");
        }

        // Публикатор цели
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

        // Подписчик на одометрию (поддерживает оба варианта топика)
        odom_sub_ = nh_.subscribe("/drone_0_visual_slam/odom", 10, &MissionNode::odomCallback, this);
        odom_fallback_sub_ = nh_.subscribe("/visual_slam/odom", 10, &MissionNode::odomCallback, this);

        // Таймер для записи в CSV каждые 100 мс (10 Гц)
        log_timer_ = nh_.createTimer(ros::Duration(0.1), &MissionNode::logTimerCallback, this);

        ROS_INFO("[Mission] Mission node ready. Waiting for drone odometry...");
    }

    ~MissionNode() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

    double getDistance(const geometry_msgs::Point& p, const Point3D& target) {
        return std::sqrt(std::pow(p.x - target.x, 2) +
                         std::pow(p.y - target.y, 2) +
                         std::pow(p.z - target.z, 2));
    }

    void publishGoal(const Point3D& pt) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "world";
        goal.pose.position.x = pt.x;
        goal.pose.position.y = pt.y;
        goal.pose.position.z = pt.z;
        goal.pose.orientation.w = 1.0;
        goal_pub_.publish(goal);

        ROS_INFO("[Mission] >>> Goal sent: [%s] at (%.2f, %.2f, %.2f)",
                 pt.name.c_str(), pt.x, pt.y, pt.z);
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        current_pos_ = msg->pose.pose.position;

        // Первый приём одометрии — старт миссии
        if (!has_odom_) {
            has_odom_ = true;
            prev_pos_ = current_pos_;
            start_time_ = ros::Time::now();
            segment_start_time_ = start_time_;
            mission_started_ = true;

            ROS_INFO("[Mission] Odometry received! Starting mission...");
            ros::Duration(0.5).sleep(); // Небольшая пауза для инициализации
            publishGoal(waypoints_[0]);
            return;
        }

        // Подсчёт пройденного пути
        double ds = std::sqrt(std::pow(current_pos_.x - prev_pos_.x, 2) +
                              std::pow(current_pos_.y - prev_pos_.y, 2) +
                              std::pow(current_pos_.z - prev_pos_.z, 2));
        total_distance_ += ds;
        prev_pos_ = current_pos_;

        if (!mission_started_ || current_idx_ >= waypoints_.size()) return;

        double dist = getDistance(current_pos_, waypoints_[current_idx_]);

        // Условие перехода: расстояние < 0.5 м с удержанием >= 1.0 сек
        if (dist < 0.5) {
            if (!holding_in_zone_) {
                holding_in_zone_ = true;
                in_zone_start_time_ = ros::Time::now();
            } else if ((ros::Time::now() - in_zone_start_time_).toSec() >= 1.0) {
                double seg_time = (ros::Time::now() - segment_start_time_).toSec();
                segment_times_.push_back(seg_time);

                ROS_INFO("[Mission] Reached point %s in %.2f seconds.",
                         waypoints_[current_idx_].name.c_str(), seg_time);

                current_idx_++;
                holding_in_zone_ = false;

                if (current_idx_ < waypoints_.size()) {
                    segment_start_time_ = ros::Time::now();
                    publishGoal(waypoints_[current_idx_]);
                } else {
                    finishMission();
                }
            }
        } else {
            holding_in_zone_ = false;
        }
    }

    void logTimerCallback(const ros::TimerEvent&) {
        if (!mission_started_ || current_idx_ >= waypoints_.size() || !csv_file_.is_open()) return;

        double t = (ros::Time::now() - start_time_).toSec();
        double dist = getDistance(current_pos_, waypoints_[current_idx_]);

        csv_file_ << std::fixed << std::setprecision(3)
                  << t << ","
                  << current_pos_.x << ","
                  << current_pos_.y << ","
                  << current_pos_.z << ","
                  << current_idx_ + 1 << ","
                  << waypoints_[current_idx_].name << ","
                  << dist << "\n";
    }

    void finishMission() {
        double total_time = (ros::Time::now() - start_time_).toSec();

        ROS_INFO("=================================================");
        ROS_INFO("            MISSION REPORT (A->B->C->D->A)       ");
        ROS_INFO("=================================================");
        ROS_INFO("Total Flight Time:    %.2f s", total_time);
        ROS_INFO("Total Traversed Path: %.2f m", total_distance_);
        ROS_INFO("-------------------------------------------------");
        std::vector<std::string> seg_names = {"A -> B", "B -> C", "C -> D", "D -> A"};
        for (size_t i = 0; i < segment_times_.size(); ++i) {
            ROS_INFO("Segment %s duration: %.2f s", seg_names[i].c_str(), segment_times_[i]);
        }
        ROS_INFO("=================================================");

        ros::shutdown();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission");
    MissionNode node;
    ros::spin();
    return 0;
}
