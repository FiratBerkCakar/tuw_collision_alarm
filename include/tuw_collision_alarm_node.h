#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <nodelet/nodelet.h>
#include <tuw_geometry/linesegment2d.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>

namespace tuw_collision_alarm {

class CollisionAlarmNodelet: public nodelet::Nodelet{
    ros::Publisher pub_path;
    ros::Subscriber sub_laser_;
    ros::Subscriber sub_path_;
    ros::Timer timer_;
    ros::NodeHandle nh_;
    size_t WaypointArrayLastIndexBehind = 0; // move to ctor
    long WaypointArrayLastIndexFront = -1; // move to ctor

    tf::TransformListener tflistener_;
    tf::TransformListener tflistener2_;
    tf::StampedTransform tftransform;
    tf::StampedTransform tftransform2;

    double distance_threshold = 0.2; // meters, robot footprint can update it, or parameter server
    size_t obstacleOnTheWayVoteThreshold = 5; // parameter server maybe
    sensor_msgs::LaserScanConstPtr laserScanPtr_=nullptr;
    nav_msgs::PathPtr waypointsPtr_= nullptr;
    void filterWaypoints(const nav_msgs::Path::ConstPtr &);
    std::vector<tuw::Point2D> laserEndPoints;
    geometry_msgs::PoseStamped newGoalPoseStamped;


    void setNewGoalPose(size_t &index, const nav_msgs::Path::ConstPtr&);
    tuw::Point2D calculateLaserEndpoints(size_t laserScanIndex);


public:
    virtual void onInit();
    void callbackLaser ( const sensor_msgs::LaserScan::ConstPtr& );
    void callbackPath (const nav_msgs::Path::ConstPtr&);
    void callbackTimer(const ros::TimerEvent& event);




};
}
