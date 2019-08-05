#include <tuw_collision_alarm_node.h>

namespace tuw_collision_alarm {

    void CollisionAlarmNodelet::onInit() {

        NODELET_INFO ("Initializing nodelet CollisionAlarmNodelet...");

        nh_ = getNodeHandle();

        WaypointArrayLastIndexBehind = 0;
        WaypointArrayLastIndexFront = -1;
        distance_threshold = 0.2;
        obstacleOnTheWayVoteThreshold = 5;
        abruptJumpChecker = 0;

        sub_laser_ = nh_.subscribe("laser0/scan", 10, &CollisionAlarmNodelet::callbackLaser, this);
        sub_path_ = nh_.subscribe("global_planner/planner/plan", 1, &CollisionAlarmNodelet::callbackPath, this);
        pub_path = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
        timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&CollisionAlarmNodelet::callbackTimer, this, _1));


    }

    void CollisionAlarmNodelet::callbackLaser(const sensor_msgs::LaserScan::ConstPtr &input_scan) {

        laserEndPointsPtr = std::make_shared<std::vector<tuw::Point2D>>();
        laserScanPtr_ = input_scan;
        for (size_t j = 0; j < laserScanPtr_->ranges.size(); j++) {
            if (std::isnan(laserScanPtr_->ranges[j])) { //skip the NaN measurements
                continue;
            }
            laserEndPointsPtr->push_back(calculateLaserEndpoints(j)); // already transformed to robot coordinates

        }


    }

    void CollisionAlarmNodelet::callbackPath(const nav_msgs::Path::ConstPtr &poseArray) {
        //making the assumption that the Poses are ordered in such a way that the first is the closest to the robot
        //size_t goalArraySize = poseArray->poses.size();
        waypointsPtr_ = poseArray;
        WaypointArrayLastIndexBehind = 0; // start allover again when there is a new waypoint array
        WaypointArrayLastIndexFront = -1;


    }


    void CollisionAlarmNodelet::setNewGoalPose(size_t index, const nav_msgs::Path::ConstPtr &poseArray) {

        newGoalPoseStamped = poseArray->poses[index];
        newGoalPoseStamped.header.stamp = ros::Time::now();

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
        try {
            tflistener_.lookupTransform("r0/base_link", "r0/laser0", ros::Time(0), tftransform); // seems correct
            tflistener_.lookupTransform("r0/base_link", "map", ros::Time(0), tftransform2);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        if (waypointsPtr_ == nullptr || laserScanPtr_ == nullptr) {
            return;
        }

        filterWaypoints(waypointsPtr_);


    }

    void CollisionAlarmNodelet::filterWaypoints(const nav_msgs::Path::ConstPtr &poseArray) {
        //assuming that i got the transfrom and a path
        // transform the path 1 by one, dont do it beforehand
        abruptJumpChecker = WaypointArrayLastIndexBehind;
        size_t obstacleOnTheWayVote = 0;
        tf::Transform temp_tf_object;
        geometry_msgs::Pose poseN0;
        geometry_msgs::Pose poseN1;
        size_t i = 0;
        for (i = WaypointArrayLastIndexBehind;
             i < poseArray->poses.size() - 1; i++) { // -1 cuz, N elements, 0-1, 1-2 ... (N-2) - ( N-1)
            tf::poseMsgToTF(poseArray->poses[i].pose, temp_tf_object);
            tf::poseTFToMsg(tftransform2 * temp_tf_object, poseN0);
            //NODELET_INFO ("Waypoint %lu, transformed to robot frame, x = %lf, y= %lf ",i, poseN0.position.x,poseN0.position.y);
            if (poseN0.position.x < 0.1) { //check if its behind the robot i.e x<0
                WaypointArrayLastIndexBehind = i; // save this index to start from this next time
                continue; // go the next iteration cuz we are not interested
            } else if (poseN0.position.x > (laserScanPtr_->range_max)) {
                NODELET_INFO ("BREAKING DUE TO LASER SCAN RANGE REACHED...");
                WaypointArrayLastIndexFront = i; // save this to check timely loop ending
                break; //no need to check any further, we are beyond the lasers scan range

            } else {
                tf::poseMsgToTF(poseArray->poses[i + 1].pose, temp_tf_object);
                tf::poseTFToMsg(tftransform2 * temp_tf_object, poseN1); // transform the next waypoint
                tuw::Point2D wayPoint0(poseN0.position.x, poseN0.position.y);
                tuw::Point2D wayPoint1(poseN1.position.x, poseN1.position.y);
                tuw::LineSegment2D lineSegment(wayPoint0, wayPoint1); //create the line segment
                for (const auto &laserEndPoint: *laserEndPointsPtr) {
                    double distance_to_line = lineSegment.distanceTo(laserEndPoint);
                    if (distance_to_line < distance_threshold) {
                        obstacleOnTheWayVote += 1;
                    }
                }
                NODELET_INFO("Iterated the laser scan array, number of Obstacle on the way Votes = %lu",
                             obstacleOnTheWayVote);

            }
            if (obstacleOnTheWayVote > obstacleOnTheWayVoteThreshold) { //There is an obstacle close to the current
                break; //segment, no need to check any further
            } else {
                obstacleOnTheWayVote = 0;
            }
        }
        NODELET_INFO("JUST PASSED INDEX %lu", WaypointArrayLastIndexBehind);

        if ((abruptJumpChecker + 5) < WaypointArrayLastIndexBehind) {
            NODELET_INFO("ABRUPT JUMP!");


            WaypointArrayLastIndexBehind = abruptJumpChecker;
        }


        if (WaypointArrayLastIndexBehind == poseArray->poses.size() -
                                            2) { // -2 is due the fact the final point is located at the back side of the robot
            ROS_INFO("AT THE GOAL POSE...");


        }

        if (i == WaypointArrayLastIndexFront ||
            i == poseArray->poses.size() - 1) { // loop broke cuz we are past the laser range, -> all clear
            ROS_INFO("ALL CLEAR...");
            WaypointArrayLastIndexFront = 0;
            return; // everything is aight, dont do anything

        } else { // there is an obstacle between waypoint i and i +1 , better send the i-1 as the last pose
            ROS_INFO("SOMETHING ON THE WAY BETWEEN %lu - %lu , SETTING A NEW GOAL POSE ", i, i + 1);
            setNewGoalPose(i-1 , waypointsPtr_);
            pub_path.publish(newGoalPoseStamped);

        }


    }


}


PLUGINLIB_EXPORT_CLASS (tuw_collision_alarm::CollisionAlarmNodelet, nodelet::Nodelet)
//int main(int argc, char **argv) {}