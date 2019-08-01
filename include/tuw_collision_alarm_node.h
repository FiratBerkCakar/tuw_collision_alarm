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
    tf::TransformListener tflistener_;
    tf::StampedTransform tftransform;

    size_t minWaypointCount = 5; // take from parameter server
    double distance_threshold = 1; // meters, robot footprint can update it, or parameter server
    size_t obstacleOnTheWayVoteThreshold = 5; // parameter server maybe
    sensor_msgs::LaserScanConstPtr laserScanPtr_=nullptr;
    nav_msgs::PathConstPtr waypointsPtr_= nullptr;


    size_t calculateNumberOfWaypointsToBeConsidered(const nav_msgs::Path::ConstPtr& );
    nav_msgs::Path createNewWaypoints(size_t index, const nav_msgs::Path::ConstPtr&);
    tuw::Point2D calculateLaserEndpoints(size_t laserScanIndex);


public:
    virtual void onInit();
    void callbackLaser ( const sensor_msgs::LaserScan::ConstPtr& );
    void callbackPath (const nav_msgs::Path::ConstPtr& );
    void callbackTimer(const ros::TimerEvent& event);




};
}
