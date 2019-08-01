#include <tuw_collision_alarm_node.h>

namespace tuw_collision_alarm {

    void CollisionAlarmNodelet::onInit() {

        NODELET_INFO ("Initializing nodelet CollisionAlarmNodelet...");

        nh_ = getNodeHandle();

        sub_laser_ = nh_.subscribe("laser0/scan", 10, &CollisionAlarmNodelet::callbackLaser, this);
        sub_path_ = nh_.subscribe("global_planner/planner/plan", 1, &CollisionAlarmNodelet::callbackPath, this);
        pub_path = nh_.advertise<nav_msgs::Path>("collision_alarm_path", 10);


    }

    void CollisionAlarmNodelet::callbackLaser(const sensor_msgs::LaserScan::ConstPtr &input_scan) {
        //laserScanPtr_ = boost::make_shared<sensor_msgs::LaserScan::ConstPtr> (input_scan);
        laserScanPtr_ = input_scan;
        NODELET_INFO ("Laser CallBack Received...");

    }

    void CollisionAlarmNodelet::callbackPath(const nav_msgs::Path::ConstPtr &poseArray) {
        //making the assumption that the Poses are ordered in such a way that the first is the closest to the robot
        //size_t goalArraySize = poseArray->poses.size();
        NODELET_INFO ("PathCB received...");
        size_t obstacleOnTheWayVote = 0;
        size_t maxWaypointsIndex = calculateNumberOfWaypointsToBeConsidered(poseArray); // dont take every pose
        size_t i = 0;
        for (i = 0; i < (maxWaypointsIndex); i++) { // start from the farthest, statistically higher
            tuw::Point2D wayPoint0(poseArray->poses[i].pose.position.x, poseArray->poses[i].pose.position.y);
            tuw::Point2D wayPoint1(poseArray->poses[i + 1].pose.position.x, poseArray->poses[i + 1].pose.position.y);
            tuw::LineSegment2D lineSegment(wayPoint0, wayPoint1);

            for (size_t j = 0; j < laserScanPtr_->ranges.size(); j++) {
                if (std::isnan(laserScanPtr_->ranges[j])) { //skip the NaN measurements
                    continue;
                }
                tuw::Point2D laserEndpoint = calculateLaserEndpoints(j);
                double distance_to_line = lineSegment.distanceTo(laserEndpoint);
                if (distance_to_line < distance_threshold) {
                    obstacleOnTheWayVote += 1;
                }
            }
            if (obstacleOnTheWayVote > obstacleOnTheWayVoteThreshold) {
                break;
            }
        }

        if (i == maxWaypointsIndex) {
            return; // everything is aight, dont do anything

        } else { // there is an obstacle between waypoint i and i +1 , better send the i-1 as the last pose

            pub_path.publish(createNewWaypoints(i - 1, poseArray));

        }


    }

    size_t CollisionAlarmNodelet::calculateNumberOfWaypointsToBeConsidered(const nav_msgs::Path::ConstPtr &poseArray) {
        double firstWaypointX = poseArray->poses[0].pose.position.x;
        double firstWaypointY = poseArray->poses[0].pose.position.y;
        double radius = laserScanPtr_->range_max;
        size_t numberofWaypointsToBeConsidered = 0;
        for (const auto &pose : poseArray->poses) {
            if (sqrt(pow(firstWaypointX - pose.pose.position.x, 2) + pow(firstWaypointY - pose.pose.position.y, 2)) <
                radius) {
                numberofWaypointsToBeConsidered++;

            } else {
                break;
            }

        }
        return numberofWaypointsToBeConsidered;

    }

    nav_msgs::Path CollisionAlarmNodelet::createNewWaypoints(size_t index, const nav_msgs::Path::ConstPtr &poseArray) {
        if (index < minWaypointCount) {
            size_t missingWaypoints = (minWaypointCount - index) + 1;
            while (missingWaypoints != 0) {
                break; //what evah, gotta think about it

            }

        } else {
            nav_msgs::Path newWayPoints;
            newWayPoints.header.stamp = ros::Time::now();
            for (size_t i = 0; i <= index; i++) {
                //newWayPoints.header.frame_id = ???
                newWayPoints.poses.push_back(poseArray->poses[i]);

            }
        }

        return nav_msgs::Path();
    }

    tuw::Point2D CollisionAlarmNodelet::calculateLaserEndpoints(size_t laserScanIndex) {
        double beamAngle = laserScanPtr_->angle_min + (laserScanPtr_->angle_increment * laserScanIndex);
        double endPointX = cos(beamAngle) * laserScanPtr_->ranges[laserScanIndex];
        double endPointY = sin(beamAngle) * laserScanPtr_->ranges[laserScanIndex];
        tf::Vector3 point(endPointX, endPointY, 0);
        try {
        tflistener.lookupTransform("/map", "/laser0/scan", ros::Time::now(), tftransform); }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        tf::Vector3 point_bl = tftransform * point;
        return tuw::Point2D(point_bl.getX(), point_bl.getY());
    }
}
PLUGINLIB_EXPORT_CLASS ( tuw_collision_alarm::CollisionAlarmNodelet, nodelet::Nodelet )
//int main(int argc, char **argv) {}