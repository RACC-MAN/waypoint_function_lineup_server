#include "waypoint_function_lineup_server/lineup_server.hpp"

using namespace std::chrono_literals;

waypoint_function::LineupServer::LineupServer(const rclcpp::NodeOptions &options) : FunctionServerNode("lineup_server_node", options) 
{
    ServerApply(SERVER_NAME, COMMAND_HEADER, EXECUTE_STATE);

    tarPose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("next_waypoint_msg", 10,
        std::bind(&LineupServer::targetPoseCallback, this, std::placeholders::_1));
    curPose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10,
        std::bind(&LineupServer::currentPoseCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
        std::bind(&LineupServer::scanCallback, this, std::placeholders::_1));

    pub_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
}

void waypoint_function::LineupServer::Update(const std_msgs::msg::Empty::SharedPtr)
{
    lineupAvairable_ = false;
    moveExecute_ = false;
}

void waypoint_function::LineupServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> , std::shared_ptr<waypoint_function_msgs::srv::Command::Response> )
{
    RCLCPP_INFO(get_logger(), "lineup Server Called.");
    lineupAvairable_ = true;
}

void waypoint_function::LineupServer::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    tarPose = msg->pose;
}

void waypoint_function::LineupServer::currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    curPose = msg->pose.pose;
}

void waypoint_function::LineupServer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if(!lineupAvairable_)return;

    float delta_angle = msg->angle_increment;
    int point_counter = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float distance = msg->ranges[i];
      float angle = delta_angle * i + msg->angle_min;
    }

    if(point_counter < scan_tolerance_)
    {
        frame_counter_++;
        if(frame_counter_ > frame_tolerance_) moveExecute_ = true;
    }
    else frame_counter_ = 0;
}

void waypoint_function::LineupServer::send_move_direction()
{
    if(!lineupAvairable_)return;

    geometry_msgs::msg::Twist vel_msg{}; 

    if(!moveExecute_)
    {
        pub_vel_->publish(vel_msg);
        return;
    }
    
    float dx = tarPose.position.x - curPose.position.x;
    float dy = tarPose.position.y - curPose.position.y;
    float theta = atan2(dy, dx);
    dist_sq = dx*dx + dy*dy;

    if(dist_sq < dist_tolerance_*dist_tolerance_)
    {
        pub_vel_->publish(vel_msg);
        lineupAvairable_ = false;
        moveExecute_ = false;
        finishLineup()
        return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(curPose.rotation, q);
    float roll, pitch, yaw;
    tf2::getEulerYPR(q, yaw, pitch, roll);
    vel_msg.angular.z = 0.5 * (theta - yaw);
    vel_msg.linear.x = 4.0;
    pub_vel_->publish(vel_msg);
}

void waypoint_function::LineupServer::finishLineup()
{
    RCLCPP_INFO(this->get_logger(), "Finish lineup sever");
    std::string result_msg = "line_server:complete";
    SendResponse(result_msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::LineupServer)