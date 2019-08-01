//
// Created by firat on 31.07.19.
//

#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tuw_collision_alarm");

    nodelet::Loader manager(false);
    nodelet::M_string remappings;
    nodelet::V_string my_argv(argv + 1, argv + argc);

    manager.load(ros::this_node::getName(), "tuw_collision_alarm/tuw_collision_alarm_node", remappings, my_argv);

    ros::spin();
    return 0;
}