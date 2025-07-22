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

    nav_handle_ = create_publisher<std_msgs::msg::String>("nav2_cancel",10);
}

void waypoint_function::LineupServer::Update(const std_msgs::msg::Empty::SharedPtr)
{
    lineupAvairable_ = false;
}

void waypoint_function::LineupServer::FunctionCallback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "lineup Server Called.");
    lineupAvairable_ = true;
    response->message = "lineup_server:activated";
}

void waypoint_function::LineupServer::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    tarPoint = msg->pose.position;
}

void waypoint_function::LineupServer::currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    curPoint = msg->pose.pose.position;
}

void waypoint_function::LineupServer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_ptr_)
{
    if(!lineupAvairable_)return;

    float distance = calc_distance(tarPoint, curPoint);
    if(distance > dist_tolerance_) return;

    int scan_count = 0;
    for(auto value : scan_ptr_->ranges) if(value < dist_tolerance_) scan_count++;
}

float waypoint_function::LineupServer::calc_distance(geometry_msgs::msg::Point pos1, geometry_msgs::msg::Point pos2)
{
    float dx = pos1.x - pos2.x;
    float dy = pos1.y - pos2.y;
    return std::sqrt(dx*dx + dy*dy);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_function::LineupServer)