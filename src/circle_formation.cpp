#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <cmath>
#include <algorithm>
using std::placeholders::_1;
#define PI 3.1415926535897932

const double tolerance = 0.1;     // angle tolerance
double center[2] = {0.0, 0.0};
const double radius = 4.0;

class Robot
{
public:
    double x;
    double y;
    double yaw;  // [-pi, pi]
    int id;     // 0 1 2 ...
    std::string name;   // "robot_0", "robot_1" ...
    double phi;

    Robot(int id, const std::string name)
    {
        this->x = 0.0;
        this->y = 0.0;
        this->yaw = 0.0;
        this->id = id;
        this->name = name;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        this->x = odom->pose.pose.position.x;
        this->y = odom->pose.pose.position.y;
        double qw = odom->pose.pose.orientation.w;
        double qx = odom->pose.pose.orientation.x;
        double qy = odom->pose.pose.orientation.y;
        double qz = odom->pose.pose.orientation.z;
        this->yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }
};

bool cmp(std::shared_ptr<Robot> probot1, std::shared_ptr<Robot> probot2)
{
    if(probot1->phi - probot2->phi < -tolerance) return true;
    if(probot1->phi - probot2->phi > tolerance) return false;
    double r1 = fabs(probot1->x - center[0]) + fabs(probot1->y - center[1]);
    double r2 = fabs(probot2->x - center[0]) + fabs(probot2->y - center[1]);
    if(r1 < r2) return true;
    return false;
}

class SteeringNode: public rclcpp::Node
{
private:
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> vel_pubs;  // vector of cmd_vel publishers
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs;    // vector of odometry subscribers
    rclcpp::CallbackGroup::SharedPtr odom_call_back_group;  // callback group of odometry subscriptions

public:
    int num_of_robots;
    std::vector<std::shared_ptr<Robot>> robots;
    SteeringNode(const std::string name): Node(name)
    {
        this->declare_parameter<int>("num_of_robots", 5);
        this->declare_parameter<double>("center_x", 0.0);
        this->declare_parameter<double>("center_y", 0.0);
        this->get_parameter("num_of_robots", this->num_of_robots);
        this->get_parameter("center_x", center[0]);
        this->get_parameter("center_y", center[1]);
        RCLCPP_INFO(this->get_logger(), "rotating center of diff drive robots: x=%g, y=%g", center[0], center[1]);
        this->robots.resize(this->num_of_robots);
        this->vel_pubs.resize(this->num_of_robots);
        this->odom_subs.resize(this->num_of_robots);
        this->odom_call_back_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions options;
        options.callback_group = this->odom_call_back_group;

        for(int i = 0; i < this->num_of_robots; i++)
        {
            this->robots[i] = std::make_shared<Robot>(i, std::string("ddrobot_") + std::to_string(i));
            auto probot = this->robots[i];
            this->vel_pubs[i] = this->create_publisher<geometry_msgs::msg::Twist>(probot->name+std::string("/cmd_vel"), 10);
            this->odom_subs[i] = this->create_subscription<nav_msgs::msg::Odometry>(
                std::string("/") + probot->name + std::string("/odom"),
                10,
                std::bind(&Robot::odom_callback, probot, _1),
                options
            );
        }
        RCLCPP_INFO(this->get_logger(), "Creating steering node %s successfully", name.c_str());
    }

    void pub_vel(std::shared_ptr<Robot> probot, double v, double w)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = v;
        msg.linear.y = msg.linear.z = 0.0;
        msg.angular.z = w;
        msg.angular.x = msg.angular.y = 0.0;
        this->vel_pubs[probot->id]->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Published v=%g w=%g to %s at x=%g, y=%g", v, w, probot->name.c_str(), probot->x, probot->y);
    }

    void counterclockwise_sort()
    {
        for(auto probot:this->robots)
        {
            probot->phi = atan2(probot->y - center[1], probot->x - center[0]);
        }
        std::sort(this->robots.begin(), this->robots.end(), cmp);
        RCLCPP_INFO(this->get_logger(), "sort result of diff drive robots:");
        for(auto probot:this->robots)
        {
            RCLCPP_INFO(this->get_logger(), "robot %s at x=%g, y=%g, phi=%g", probot->name.c_str(), probot->x, probot->y, probot->phi);
        }
    }
};

int main(int argc, char *const *argv)
{
    rclcpp::init(argc, argv);
    auto pnode = std::make_shared<SteeringNode>("ddrobot_steering_node");

    sleep(5);

    // start formation
    RCLCPP_INFO(pnode->get_logger(), "Start circle formation");
    rclcpp::Rate rate(30);
    int num = pnode->num_of_robots;
    double (*p_bar)[2] = new double[num][2];
    double *phi = new double[num];
    double *d = new double[num];
    for(int i = 0; i < num; i++) d[i] = 2*PI/num;
    double *loss_r = new double[num]{0.0};
    rclcpp::spin_some(pnode);
    const double k = 4.0, kp = 2.0, ki = 10.0, C1 = 2.0, C2 = 1.5, C3 = 0.7, r0 = 0.5;
    pnode->counterclockwise_sort();
    int frame = 0;
    while(rclcpp::ok())
    {
        frame++;
        rclcpp::spin_some(pnode);
        for(int i = 0; i < num; i++)
        {
            p_bar[i][0] = pnode->robots[i]->x - center[0];
            p_bar[i][1] = pnode->robots[i]->y - center[1];
            phi[i] = atan2(p_bar[i][1], p_bar[i][0]);
        }
        for(int i = 0; i < num; i++)
        {
            int i_plus = (i + 1) % num;
            int i_sub = (i - 1 + num) % num;
            double beta = phi[i_plus] - phi[i];
            if(beta < -tolerance) beta += 2*PI;
            double beta_sub = phi[i] - phi[i_sub];
            if(beta_sub < -tolerance) beta_sub += 2*PI;
            double alpha = pnode->robots[i]->yaw - phi[i];

            double v = C1 + C2/(2*PI) * (d[i_sub]*beta-d[i]*beta_sub)/(d[i]+d[i_sub]);

            // collision avoidance
            double r = sqrt(p_bar[i][0]*p_bar[i][0] + p_bar[i][1]*p_bar[i][1]);
            double phi_dot = v * sin(alpha) / r;
            if(phi_dot > 0)
            {
                if(phi_dot > C3 * beta) v *= C3 * beta / phi_dot;
            }
            else if(phi_dot < 0)
            {
                if(phi_dot < -C3 * beta_sub) v *= C3 * beta_sub / phi_dot;
            }

            double w = (k * cos(alpha) + v) / radius;

            // radius loss decrease
            if(fabs(r - radius) < r0)
            {
                loss_r[i] += r - radius;
                w += (kp * (r - radius) + ki * loss_r[i] / frame) * v / radius;
            }
            pnode->pub_vel(pnode->robots[i], v, w);
            RCLCPP_INFO(pnode->get_logger(), "ddrobot_%d's distance to center: %g", i, r);
        }
        rate.sleep();
    }

    delete(p_bar);
    delete(phi);
    delete(d);
    rclcpp::shutdown();
    return 0;
}