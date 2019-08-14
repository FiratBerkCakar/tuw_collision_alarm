//
// Created by eugen on 13.08.19.
//

#include "tuw_collision_path_generator_nodelet.h"




namespace tuw_collision_alarm {

    void CollisionPathGeneratorNodelet::onInit() {

        NODELET_INFO("INITIALISING COLLISION PATH GENERATOR NODELET");

        sub_path_ = nh_.subscribe("global_planner/planner/plan", 1, &CollisionPathGeneratorNodelet::callbackPath, this);
        sub_path_ = nh_.subscribe("collision_alarm/goal_reached", 1, &CollisionPathGeneratorNodelet::callbackGoalReached, this);
        pub_goal_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("r0/gaol",1);

        collision_alarm_ = false;
        goal_reached = false;


    }

    void CollisionPathGeneratorNodelet::callbackCollisionAvoidanceGoalPose(
            const geometry_msgs::PoseStamped::ConstPtr & avoidance_goal_pose) {
        collision_alarm_ = true;
        avoidance_goal_pose_ = *avoidance_goal_pose;
        pub_goal_pose_.publish(avoidance_goal_pose);


    }

    void CollisionPathGeneratorNodelet::callbackGoalPose(const geometry_msgs::PoseStamped::ConstPtr & x) {
        if(collision_alarm_ == false){
        original_goal_pose_ = *x;
        }
    }

    void CollisionPathGeneratorNodelet::callbackGoalReached(const std_msgs::Bool msg) {
        goal_reached = msg.data;
        if(collision_alarm_ == true){
            pub_goal_pose_.publish(original_goal_pose_);
            collision_alarm_ = false;
            goal_reached = false;

        }

    }

    }


PLUGINLIB_EXPORT_CLASS (tuw_collision_alarm::CollisionPathGeneratorNodelet, nodelet::Nodelet)