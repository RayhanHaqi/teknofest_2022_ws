#include "ros/ros.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/WaypointReached.h"

int lap_count = 0;
bool is_lap_2 = false;
bool ready_dropping = false;

void wpReachedCallback(const mavros_msgs::WaypointReached::ConstPtr& msg) {
    ROS_INFO("Current wp reached: %d", msg->wp_seq);
    if (msg->wp_seq == 6 && lap_count == 0) {
        ROS_INFO("Entering lap 2");
        lap_count += 1;
        is_lap_2 = true;
    } else if (msg->wp_seq == 5 && lap_count > 0) {
        ROS_INFO("Change to fbwb");
        lap_count += 1;
        ready_dropping = true;
    } 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_controller");
    ros::NodeHandle n;

    // Clear existing waypoint
    mavros_msgs::WaypointClear waypoint_clear_srv = mavros_msgs::WaypointClear();

    ros::ServiceClient waypoint_clear_client = n.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    
    if (waypoint_clear_client.call(waypoint_clear_srv)) {
        ROS_INFO("Waypoint clear: %d", waypoint_clear_srv.response.success);
    } else {
        ROS_INFO("Waypoint clear Error");
        return 1;
    }

    // Send waypoint
    ros::ServiceClient wp_push_client = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    
    mavros_msgs::Waypoint wp_0 = mavros_msgs::Waypoint();
    mavros_msgs::Waypoint wp_1 = mavros_msgs::Waypoint();
    mavros_msgs::Waypoint wp_2 = mavros_msgs::Waypoint();
    mavros_msgs::Waypoint wp_3 = mavros_msgs::Waypoint();
    mavros_msgs::Waypoint wp_4 = mavros_msgs::Waypoint();
    mavros_msgs::Waypoint wp_5 = mavros_msgs::Waypoint();
    mavros_msgs::Waypoint wp_6 = mavros_msgs::Waypoint();
    mavros_msgs::Waypoint wp_7 = mavros_msgs::Waypoint();
    mavros_msgs::Waypoint wp_8 = mavros_msgs::Waypoint();

    // WP circle
    wp_0.autocontinue = true;
    wp_0.command = 16;
    wp_0.frame = 0;
    wp_0.is_current = false;
    wp_0.param1 = 0;
    wp_0.param2 = 0;
    wp_0.param3 = 0;
    wp_0.param4 = 0;
    wp_0.x_lat = -35.3632622;
    wp_0.y_long = 149.1652376;
    wp_0.z_alt = 584.0900268554688;

    wp_1.autocontinue = true;
    wp_1.command = 22;
    wp_1.frame = 3;
    wp_1.is_current = false;
    wp_1.param1 = 30.0;
    wp_1.param2 = 0;
    wp_1.param3 = 0;
    wp_1.param4 = 0;
    wp_1.x_lat = -35.3629241;
    wp_1.y_long = 149.1651723;
    wp_1.z_alt = 10.0;

    wp_2.autocontinue = true;
    wp_2.command = 16;
    wp_2.frame = 3;
    wp_2.is_current = false;
    wp_2.param1 = 0;
    wp_2.param2 = 0;
    wp_2.param3 = 0;
    wp_2.param4 = 0;
    wp_2.x_lat = -35.362397;
    wp_2.y_long = 149.1650274;
    wp_2.z_alt = 10.0;

    wp_3.autocontinue = true;
    wp_3.command = 16;
    wp_3.frame = 3;
    wp_3.is_current = false;
    wp_3.param1 = 0;
    wp_3.param2 = 0;
    wp_3.param3 = 0;
    wp_3.param4 = 0;
    wp_3.x_lat = -35.3624056;
    wp_3.y_long = 149.1646251;
    wp_3.z_alt = 10.0;

    wp_4.autocontinue = true;
    wp_4.command = 16;
    wp_4.frame = 3;
    wp_4.is_current = false;
    wp_4.param1 = 0;
    wp_4.param2 = 0;
    wp_4.param3 = 0;
    wp_4.param4 = 0;
    wp_4.x_lat = -35.3624385;
    wp_4.y_long = 149.1644052;
    wp_4.z_alt = 10.0;

    wp_5.autocontinue = true;
    wp_5.command = 16;
    wp_5.frame = 3;
    wp_5.is_current = false;
    wp_5.param1 = 0;
    wp_5.param2 = 0;
    wp_5.param3 = 0;
    wp_5.param4 = 0;
    wp_5.x_lat = -35.3631866;
    wp_5.y_long = 149.1642979;
    wp_5.z_alt = 10.0;

    wp_6.autocontinue = true;
    wp_6.command = 16;
    wp_6.frame = 3;
    wp_6.is_current = false;
    wp_6.param1 = 0;
    wp_6.param2 = 0;
    wp_6.param3 = 0;
    wp_6.param4 = 0;
    wp_6.x_lat = -35.3631472;
    wp_6.y_long = 149.1647726;
    wp_6.z_alt = 10.0;

    wp_7.autocontinue = true;
    wp_7.command = 177;
    wp_7.frame = 0;
    wp_7.is_current = false;
    wp_7.param1 = 2.0;
    wp_7.param2 = -1.0;
    wp_7.param3 = 0;
    wp_7.param4 = 0;
    wp_7.x_lat = 0;
    wp_7.y_long = 0;
    wp_7.z_alt = 0;

    std::vector<mavros_msgs::Waypoint> lap_1_mission = std::vector<mavros_msgs::Waypoint>();
    lap_1_mission.push_back(wp_0);
    lap_1_mission.push_back(wp_1);
    lap_1_mission.push_back(wp_2);
    lap_1_mission.push_back(wp_3);
    lap_1_mission.push_back(wp_4);
    lap_1_mission.push_back(wp_5);
    lap_1_mission.push_back(wp_6);
    lap_1_mission.push_back(wp_7);

    mavros_msgs::WaypointPush waypoint_push_srv = mavros_msgs::WaypointPush();
    waypoint_push_srv.request.waypoints = lap_1_mission;
    waypoint_push_srv.request.start_index = 0;

    if (wp_push_client.call(waypoint_push_srv)) {
        ROS_INFO("Success: %d", waypoint_push_srv.response.success);
        ROS_INFO("Wp Transferred: %d", waypoint_push_srv.response.wp_transfered);
    } else {
        ROS_INFO("Success: %d", waypoint_push_srv.response.success);
        return 1;
    }


    // Arm throttle
    mavros_msgs::CommandBool arm_request_srv = mavros_msgs::CommandBool();
    arm_request_srv.request.value = true;

    ros::ServiceClient arm_request_client = n.serviceClient<mavros_msgs::CommandBoolRequest>("mavros/cmd/arming");
    
    if (arm_request_client.call(arm_request_srv)) {
        ROS_INFO("Arm: %d", arm_request_srv.response.result);
    } else {
        ROS_INFO("Arm Error: %d", arm_request_srv.response.result);
        return 1;
    }

    // Set mode auto
    mavros_msgs::SetMode set_mode_auto_srv = mavros_msgs::SetMode();
    set_mode_auto_srv.request.custom_mode = "AUTO";

    ros::ServiceClient set_mode_auto_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    if (set_mode_auto_client.call(set_mode_auto_srv)) {
        ROS_INFO("Set mode: %d", set_mode_auto_srv.response.mode_sent);
    } else {
        ROS_INFO("Set mode: %d", set_mode_auto_srv.response.mode_sent);
        return 1;
    }

    // Wait for the first lap
    ros::Subscriber wp_status_sub = n.subscribe<mavros_msgs::WaypointReached>("mavros/mission/reached", 1000, wpReachedCallback);
    
    ros::Rate loop_rate(10);

    while (!is_lap_2) {
        ROS_INFO("Looping");
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    // WP circle
    wp_0.autocontinue = true;
    wp_0.command = 16;
    wp_0.frame = 0;
    wp_0.is_current = false;
    wp_0.param1 = 0;
    wp_0.param2 = 0;
    wp_0.param3 = 0;
    wp_0.param4 = 0;
    wp_0.x_lat = -35.3632622;
    wp_0.y_long = 149.1652376;
    wp_0.z_alt = 584.0900268554688;

    wp_1.autocontinue = true;
    wp_1.command = 22;
    wp_1.frame = 3;
    wp_1.is_current = false;
    wp_1.param1 = 30.0;
    wp_1.param2 = 0;
    wp_1.param3 = 0;
    wp_1.param4 = 0;
    wp_1.x_lat = -35.3629241;
    wp_1.y_long = 149.1651723;
    wp_1.z_alt = 10.0;

    wp_2.autocontinue = true;
    wp_2.command = 16;
    wp_2.frame = 3;
    wp_2.is_current = false;
    wp_2.param1 = 0;
    wp_2.param2 = 0;
    wp_2.param3 = 0;
    wp_2.param4 = 0;
    wp_2.x_lat = -35.362397;
    wp_2.y_long = 149.1650274;
    wp_2.z_alt = 10.0;

    wp_3.autocontinue = true;
    wp_3.command = 16;
    wp_3.frame = 3;
    wp_3.is_current = false;
    wp_3.param1 = 0;
    wp_3.param2 = 0;
    wp_3.param3 = 0;
    wp_3.param4 = 0;
    wp_3.x_lat = -35.3624056;
    wp_3.y_long = 149.1646251;
    wp_3.z_alt = 10.0;

    wp_4.autocontinue = true;
    wp_4.command = 16;
    wp_4.frame = 3;
    wp_4.is_current = false;
    wp_4.param1 = 0;
    wp_4.param2 = 0;
    wp_4.param3 = 0;
    wp_4.param4 = 0;
    wp_4.x_lat = -35.3624385;
    wp_4.y_long = 149.1644052;
    wp_4.z_alt = 10.0;

    wp_5.autocontinue = true;
    wp_5.command = 16;
    wp_5.frame = 3;
    wp_5.is_current = false;
    wp_5.param1 = 0;
    wp_5.param2 = 0;
    wp_5.param3 = 0;
    wp_5.param4 = 0;
    wp_5.x_lat = -35.3626398;
    wp_5.y_long = 149.1643193;
    wp_5.z_alt = 10.0;

    wp_6.autocontinue = true;
    wp_6.command = 16;
    wp_6.frame = 3;
    wp_6.is_current = false;
    wp_6.param1 = 0;
    wp_6.param2 = 0;
    wp_6.param3 = 0;
    wp_6.param4 = 0;
    wp_6.x_lat = -35.3631866;
    wp_6.y_long = 149.1642979;
    wp_6.z_alt = 10.0;

    wp_7.autocontinue = true;
    wp_7.command = 16;
    wp_7.frame = 3;
    wp_7.is_current = false;
    wp_7.param1 = 0;
    wp_7.param2 = 0;
    wp_7.param3 = 0;
    wp_7.param4 = 0;
    wp_7.x_lat = -35.3631472;
    wp_7.y_long = 149.1647726;
    wp_7.z_alt = 10.0;

    wp_8.autocontinue = true;
    wp_8.command = 177;
    wp_8.frame = 0;
    wp_8.is_current = false;
    wp_8.param1 = 2.0;
    wp_8.param2 = -1.0;
    wp_8.param3 = 0;
    wp_8.param4 = 0;
    wp_8.x_lat = 0;
    wp_8.y_long = 0;
    wp_8.z_alt = 0;

    // WP dropping
    // wp_0.autocontinue = true;
    // wp_0.command = 16;
    // wp_0.frame = 0;
    // wp_0.is_current = false;
    // wp_0.param1 = 0;
    // wp_0.param2 = 0;
    // wp_0.param3 = 0;
    // wp_0.param4 = 0;
    // wp_0.x_lat = -35.3632622;
    // wp_0.y_long = 149.1652376;
    // wp_0.z_alt = 584.0900268554688;

    // wp_1.autocontinue = true;
    // wp_1.command = 22;
    // wp_1.frame = 3;
    // wp_1.is_current = false;
    // wp_1.param1 = 30.0;
    // wp_1.param2 = 0;
    // wp_1.param3 = 0;
    // wp_1.param4 = 0;
    // wp_1.x_lat = -35.3629241;
    // wp_1.y_long = 149.1651723;
    // wp_1.z_alt = 10.0;

    // wp_2.autocontinue = true;
    // wp_2.command = 16;
    // wp_2.frame = 3;
    // wp_2.is_current = false;
    // wp_2.param1 = 0;
    // wp_2.param2 = 0;
    // wp_2.param3 = 0;
    // wp_2.param4 = 0;
    // wp_2.x_lat = -35.362585;
    // wp_2.y_long = 149.1651106;
    // wp_2.z_alt = 10.0;

    // wp_3.autocontinue = true;
    // wp_3.command = 16;
    // wp_3.frame = 3;
    // wp_3.is_current = false;
    // wp_3.param1 = 0;
    // wp_3.param2 = 0;
    // wp_3.param3 = 0;
    // wp_3.param4 = 0;
    // wp_3.x_lat = -35.362631;
    // wp_3.y_long = 149.1653922;
    // wp_3.z_alt = 10.0;

    // wp_4.autocontinue = true;
    // wp_4.command = 16;
    // wp_4.frame = 3;
    // wp_4.is_current = false;
    // wp_4.param1 = 0;
    // wp_4.param2 = 0;
    // wp_4.param3 = 0;
    // wp_4.param4 = 0;
    // wp_4.x_lat = -35.3632282;
    // wp_4.y_long = 149.1649255;
    // wp_4.z_alt = 10.0;

    // wp_5.autocontinue = true;
    // wp_5.command = 16;
    // wp_5.frame = 3;
    // wp_5.is_current = false;
    // wp_5.param1 = 0;
    // wp_5.param2 = 0;
    // wp_5.param3 = 0;
    // wp_5.param4 = 0;
    // wp_5.x_lat = -35.3626398;
    // wp_5.y_long = 149.1643193;
    // wp_5.z_alt = 10.0;

    // wp_6.autocontinue = true;
    // wp_6.command = 16;
    // wp_6.frame = 3;
    // wp_6.is_current = false;
    // wp_6.param1 = 0;
    // wp_6.param2 = 0;
    // wp_6.param3 = 0;
    // wp_6.param4 = 0;
    // wp_6.x_lat = -35.3632785;
    // wp_6.y_long = 149.1652367;
    // wp_6.z_alt = 10.0;

    // wp_7.autocontinue = true;
    // wp_7.command = 177;
    // wp_7.frame = 0;
    // wp_7.is_current = false;
    // wp_7.param1 = 2.0;
    // wp_7.param2 = -1.0;
    // wp_7.param3 = 0;
    // wp_7.param4 = 0;
    // wp_7.x_lat = 0;
    // wp_7.y_long = 0;
    // wp_7.z_alt = 0;

    std::vector<mavros_msgs::Waypoint> lap_2_mission = std::vector<mavros_msgs::Waypoint>();
    lap_2_mission.push_back(wp_0);
    lap_2_mission.push_back(wp_1);
    lap_2_mission.push_back(wp_2);
    lap_2_mission.push_back(wp_3);
    lap_2_mission.push_back(wp_4);
    lap_2_mission.push_back(wp_5);
    lap_2_mission.push_back(wp_6);
    lap_2_mission.push_back(wp_7);
    lap_2_mission.push_back(wp_8);

    waypoint_push_srv.request.waypoints = lap_2_mission;
    waypoint_push_srv.request.start_index = 0;

    if (wp_push_client.call(waypoint_push_srv)) {
        ROS_INFO("Success: %d", waypoint_push_srv.response.success);
        ROS_INFO("Wp Transferred: %d", waypoint_push_srv.response.wp_transfered);
    } else {
        ROS_INFO("Success: %d", waypoint_push_srv.response.success);
        return 1;
    }

    while (!ready_dropping) {
        ROS_INFO("Prepare for dropping");
        ros::spinOnce();
        loop_rate.sleep();
    }

    sleep(0.5);

    // Set mode fbwb
    mavros_msgs::SetMode set_mode_fbwb_srv = mavros_msgs::SetMode();
    set_mode_fbwb_srv.request.custom_mode = "FBWB";
    ros::ServiceClient set_mode_fbwb_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    if (set_mode_fbwb_client.call(set_mode_fbwb_srv)) {
        ROS_INFO("Set mode: %d", set_mode_fbwb_srv.response.mode_sent);
    } else {
        ROS_INFO("Set mode: %d", set_mode_fbwb_srv.response.mode_sent);
        return 1;
    }

    // TODO
    // Droping

    sleep(2);

    // Set mode auto
    if (set_mode_auto_client.call(set_mode_auto_srv)) {
        ROS_INFO("Set mode: %d", set_mode_auto_srv.response.mode_sent);
    } else {
        ROS_INFO("Set mode: %d", set_mode_auto_srv.response.mode_sent);
        return 1;
    }

    return 0;
}
