#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <math.h>
using std::placeholders::_1;
#define PI 3.1415926535897932384626

double tolerance = 0.1;     // angle tolerance
double center[2] = {0.0, 0.0};
double radius = 5.0;

class Robot
{
public:
    double x;
    double y;
    double facing;  // [-pi, pi]
    int id;     // 0 1 2 ...
    std::string name;   // "robot_0", "robot_1" ...
    double theta;

    Robot(int id, const std::string name)
    {
        this->x = 0.0;
        this->y = 0.0;
        this->facing = 0.0;
        this->id = id;
        this->name = name;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        this->x = odom->pose.pose.position.x;
        this->y = odom->pose.pose.position.y;
        double w = odom->pose.pose.orientation.w;
        double z = odom->pose.pose.orientation.z;
        this->facing = 2 * atan2(z, w);
    }
};

bool cmp(std::shared_ptr<Robot> probot1, std::shared_ptr<Robot> probot2)
{
    if(probot1->theta - probot2->theta < -tolerance) return true;
    if(probot1->theta - probot2->theta > tolerance) return false;
    double r1 = fabs(probot1->x - center[0]) + fabs(probot1->y - center[1]);
    double r2 = fabs(probot2->x - center[0]) + fabs(probot2->y - center[1]);
    if(r1 < r2) return true;
    return false;
}

class SteeringNode: public rclcpp::Node
{
private:
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> vel_pubs;  // vector of cmd_vel publishers
    rclcpp::CallbackGroup::SharedPtr odom_call_back_group;  // callback group of odometry subscriptions

public:
    int num_of_robots;
    std::vector<std::shared_ptr<Robot>> robots;
    SteeringNode(const std::string name): Node(name)
    {
        this->declare_parameter<int>("num_of_robots", 5);
        this->get_parameter("num_of_robots", this->num_of_robots);
        this->robots.resize(this->num_of_robots);
        this->vel_pubs.resize(this->num_of_robots);
        this->odom_call_back_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        for(int i = 0; i < this->num_of_robots; i++)
        {
            this->robots[i] = std::make_shared<Robot>(i, std::string("robot_") + std::to_string(i));
            auto probot = this->robots[i];
            this->vel_pubs[i] = this->create_publisher<geometry_msgs::msg::Twist>(probot->name+std::string("/cmd_vel"), 10);
            rclcpp::SubscriptionOptions options;
            options.callback_group = this->odom_call_back_group;
            this->create_subscription<nav_msgs::msg::Odometry>(
                probot->name + std::string("/odom"),
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
    }

    void counterclockwise_sort()
    {
        for(auto probot:this->robots)
        {
            probot->theta = atan2(probot->y - center[1], probot->x - center[0]);
        }
        std::sort(this->robots.begin(), this->robots.end(), cmp);
    }
};

void circle_formation_control(double* p_bar, double* p_hat, double* p_hat_minus, double d, double d_minus, double facing, double vel[2])
{
    const double k = PI-7/4, C1 = 1.0, C2 = 1.0;
    double theta = atan2(p_bar[1], p_bar[0]);
    double p_bar_plus[2], p_bar_minus[2];
    p_bar_plus[0] = p_bar[0] + p_hat[0];
    p_bar_plus[1] = p_bar[1] + p_hat[1];
    p_bar_minus[0] = p_bar[0] - p_hat_minus[0];
    p_bar_minus[1] = p_bar[1] - p_hat_minus[1];
    double alpha = atan2(p_bar[0]*p_bar_plus[1]-p_bar[1]*p_bar_plus[0], p_bar[0]*p_bar_plus[0]+p_bar[1]*p_bar_plus[1]);
    if(alpha < -tolerance) alpha += 2*PI;
    double alpha_minus = atan2(p_bar[1]*p_bar_minus[0]-p_bar[0]*p_bar_minus[1], p_bar[0]*p_bar_minus[0]+p_bar[1]*p_bar_minus[1]);
    if(alpha_minus < -tolerance) alpha_minus += 2*PI;
    double l = radius * radius - p_bar[0] * p_bar[0] - p_bar[1] * p_bar[1];
    vel[0] = C1 + C2 * (d_minus * alpha - d * alpha_minus) / (2 * PI * (d + d_minus));
    vel[1] = (k * cos(theta-facing) + vel[0]) / radius;
}

void executor_thread(rclcpp::executors::MultiThreadedExecutor &executor)
{
    executor.spin();
}

int main(int argc, char *const *argv)
{
    rclcpp::init(argc, argv);
    auto pnode = std::make_shared<SteeringNode>("steering_node");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pnode);
    std::thread exec_thread(executor_thread, std::ref(executor));
    RCLCPP_INFO(pnode->get_logger(), "Receiving odom msg");

    sleep(3);

    // start formation
    RCLCPP_INFO(pnode->get_logger(), "Start circle formation");
    rclcpp::Rate rate(30);
    int num = pnode->num_of_robots;
    double (*p_bar)[2] = new double[num][2];
    double (*p_hat)[2] = new double[num][2];
    double *d = new double[num]{2.*PI/num};
    double vel[2];
    pnode->counterclockwise_sort();
    while(rclcpp::ok())
    {
        for(int i = 0; i < num; i++)
        {
            p_bar[i][0] = pnode->robots[i]->x - center[0];
            p_bar[i][1] = pnode->robots[i]->y - center[1];
        }
        for(int i = 0; i < num-1; i++)
        {
            p_hat[i][0] = p_bar[i+1][0] - p_bar[i][0];
            p_hat[i][1] = p_bar[i+1][1] - p_bar[i][1];
        }
        p_hat[num-1][0] = p_bar[0][0] - p_bar[num-1][0];
        p_hat[num-1][1] = p_bar[0][1] - p_bar[num-1][1];

        circle_formation_control(p_bar[0], p_hat[0], p_hat[num-1], d[0], d[num-1], pnode->robots[0]->facing, vel);
        pnode->pub_vel(pnode->robots[0], vel[0], vel[1]);
        for(int i = 1; i < num; i++)
        {
            circle_formation_control(p_bar[i], p_hat[i], p_hat[i-1], d[i], d[i-1], pnode->robots[i]->facing, vel);
            pnode->pub_vel(pnode->robots[i], vel[0], vel[1]);
        }
        rate.sleep();
    }

    delete(p_bar);
    delete(p_hat);
    delete(d);
    rclcpp::shutdown();
    return 0;
}