//
// Created by eugen on 13.08.19.
//

#ifndef TUW_COLLISION_ALARM_TUW_COLLISION_PATH_GENERATOR_NODELET_H
#define TUW_COLLISION_ALARM_TUW_COLLISION_PATH_GENERATOR_NODELET_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/builtin_int32.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/builtin_bool.h>


namespace tuw_collision_alarm {
    class CollisionPathGeneratorNodelet : public nodelet::Nodelet {

    public:
        void callbackPath (const nav_msgs::Path::ConstPtr&);
        void callbackCollisionAvoidanceGoalPose(const geometry_msgs::PoseStamped::ConstPtr &);
        void callbackGoalPose(const geometry_msgs::PoseStamped::ConstPtr &);
        void callbackGoalReached(const std_msgs::Bool );


    private:
        virtual void onInit();
        ros::Subscriber sub_path_;
        ros::NodeHandle nh_;
        ros::Publisher pub_goal_pose_;
        ros::Subscriber sub_gaol_reached_;
        geometry_msgs::PoseStamped original_goal_pose_;
        geometry_msgs::PoseStamped avoidance_goal_pose_;
        bool collision_alarm_;
        bool goal_reached;


    };

}
#endif //TUW_COLLISION_ALARM_TUW_COLLISION_PATH_GENERATOR_NODELET_H
