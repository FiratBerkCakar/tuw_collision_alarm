#include <tuw_collision_alarm_node.h>

namespace tuw_collision_alarm {

    void CollisionAlarmNodelet::onInit() {

        NODELET_INFO ("Initializing nodelet CollisionAlarmNodelet...");

        nh_ = getNodeHandle();

        sub_laser_ = nh_.subscribe("laser0/scan", 10, &CollisionAlarmNodelet::callbackLaser, this);
        sub_path_ = nh_.subscribe("global_planner/planner/plan", 1, &CollisionAlarmNodelet::callbackPath, this);
        pub_path = nh_.advertise<nav_msgs::Path>("collision_alarm_path", 10);
        timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&CollisionAlarmNodelet::callbackTimer, this, _1));


    }

    void CollisionAlarmNodelet::callbackLaser(const sensor_msgs::LaserScan::ConstPtr &input_scan) {
        //laserScanPtr_ = boost::make_shared<sensor_msgs::LaserScan::ConstPtr> (input_scan);
        laserScanPtr_ = input_scan;
        NODELET_INFO ("Laser CallBack Received...");

    }

    void CollisionAlarmNodelet::callbackPath(const nav_msgs::Path::ConstPtr &poseArray) {
        //making the assumption that the Poses are ordered in such a way that the first is the closest to the robot
        //size_t goalArraySize = poseArray->poses.size();
        waypointsPtr_ = poseArray;
        NODELET_INFO ("PathCB received...");


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
        nav_msgs::Path newWayPoints;
        if (index < minWaypointCount) {
            NODELET_INFO ("I aint no motherfucking ...");
            size_t missingWaypoints = (minWaypointCount - index) + 1;
            while (missingWaypoints != 0) {
                break; //what evah, gotta think about it

            }

        } else {

            newWayPoints.header.stamp = ros::Time::now();
            newWayPoints.header.frame_id = "map";
            for (size_t i = 0; i <= index; i++) {


                newWayPoints.poses.push_back(poseArray->poses[i]);
                newWayPoints.poses[i].header.stamp = ros::Time::now();

            }
        }

        return newWayPoints;
    }

    tuw::Point2D CollisionAlarmNodelet::calculateLaserEndpoints(size_t laserScanIndex) {
        double beamAngle = laserScanPtr_->angle_min + (laserScanPtr_->angle_increment * laserScanIndex);
        double endPointX = cos(beamAngle) * laserScanPtr_->ranges[laserScanIndex];
        double endPointY = sin(beamAngle) * laserScanPtr_->ranges[laserScanIndex];
        tf::Vector3 point(endPointX, endPointY, 0);

        tf::Vector3 point_bl = tftransform * point;
        return tuw::Point2D(point_bl.getX(), point_bl.getY());
    }


    void CollisionAlarmNodelet::callbackTimer(const ros::TimerEvent &event) {
        NODELET_INFO ("TimerCB received...");
        try {
            tflistener_.lookupTransform("/map", "r0/laser0", ros::Time(0), tftransform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        if (waypointsPtr_ == nullptr || laserScanPtr_ == nullptr) {
            return;
        }
        NODELET_INFO ("Scan and Waypoints received...");
        size_t obstacleOnTheWayVote = 0;
        NODELET_INFO("Total Way Points %ld " ,waypointsPtr_->poses.size());
        size_t maxWaypointsIndex = calculateNumberOfWaypointsToBeConsidered(waypointsPtr_) - 1; // dont take every pose
        size_t i = 0;
        NODELET_INFO("maxWaypointsIndex %ld " ,maxWaypointsIndex);
        NODELET_INFO("Laser Scan total number %ld " , laserScanPtr_->ranges.size());
        for (size_t k = 0; k < maxWaypointsIndex; k++) {
            NODELET_INFO(" k= %ld, x = %lf , y = %lf " ,k, waypointsPtr_->poses[k].pose.position.x, waypointsPtr_->poses[k].pose.position.y);

        }


        for (i = 0; i < maxWaypointsIndex; i++) {
            tuw::Point2D wayPoint0(waypointsPtr_->poses[i].pose.position.x, waypointsPtr_->poses[i].pose.position.y);
            tuw::Point2D wayPoint1(waypointsPtr_->poses[i + 1].pose.position.x,
                                   waypointsPtr_->poses[i + 1].pose.position.y);
            tuw::LineSegment2D lineSegment(wayPoint0, wayPoint1);

            NODELET_INFO ("Line Segment received...");
            for (size_t j = 0; j < laserScanPtr_->ranges.size(); j++) {
                if (std::isnan(laserScanPtr_->ranges[j])) { //skip the NaN measurements
                    continue;
                }
                tuw::Point2D laserEndpoint = calculateLaserEndpoints(j);
                NODELET_INFO ("Endpoint calculated for laser index =  %ld  waypointindex %ld",j,i);
                double distance_to_line = lineSegment.distanceTo(laserEndpoint);
                if (distance_to_line < distance_threshold) {
                    obstacleOnTheWayVote += 1;
                }
            }

            if (obstacleOnTheWayVote > obstacleOnTheWayVoteThreshold) {
                ROS_INFO("I SHOUDLNT BE HERE");
                break;
            }
            ROS_INFO("CALCULATION DONE");
        }
        ROS_INFO("out of the loop");
        if (i == maxWaypointsIndex) {
            return; // everything is aight, dont do anything

        } else { // there is an obstacle between waypoint i and i +1 , better send the i-1 as the last pose

            pub_path.publish(createNewWaypoints(i - 1, waypointsPtr_));

        }


    }


}
PLUGINLIB_EXPORT_CLASS (tuw_collision_alarm::CollisionAlarmNodelet, nodelet::Nodelet)
//int main(int argc, char **argv) {}