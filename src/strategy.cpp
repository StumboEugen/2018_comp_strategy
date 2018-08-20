float SAFE_RANGE = 0.8;
float STOP_RANGE = 0.2;
float MAP_ORIGIN_TO_STRUCT_X = 1.0;
float MAP_ORIGIN_TO_STRUCT_Y = 1.0;
float SCAN_DELAY = 2.0;
float SCAN_SPEED = 1.0;
float BRAKE_COE = 1.0;
float HEIGHT = 1.0;
float SPD_CLIMB = 0.1;
float SPD_TURN = 0.5;
float ANGLE_TOLERANCE = 0.3;
int WAYPOINT_GROUP = 1;
bool OUTPUT_GAZEBO = true;
bool OUTPUT_PX4 = true;

float REFRESH_RATE = 10;

float HEIGHT_MAX = 3;
float SPD_Z_MAX = 0.8;
float TOLLERANCE_Z = 0.1;
float SPD_Z_SMALL = 0.2;
float SAFE_RANGE_Z = 0.5;

float SPD_XY_MAX = 1;
float TOLLERANCE_XY = 0.15;
float SPD_XY_SMALL = 0.2;
float SAFE_RANGE_XY = 0.8;

float CRUISE_HEIGHT = 1.0;

float QR_OFFSET23 = 0.0f;
float QR_GLOBOL_OFFSET_X = 0.0f;
float QR_GLOBOL_OFFSET_Y = 0.0f;

#include "ros/ros.h"
#include "strategy.h"
#include "SimpleTarget.h"
#include <vector>

#include <iostream>

using std::vector;
using ros::Publisher;

Publisher pub_Pose;
Publisher pub_Vel;
Publisher pub_TakeOff;

vec3f_t POSgolbalInMind = {0.0, 0.0, 0.0};
vec3f_t SPgolbalInMind = {0.0, 0.0, 0.0};
vec3f_t currentPX4Pos = {0.0, 0.0, 0.0};

vector<SimpleTarget> SimpleTargets;
vector<vector<QRTarget>> QRTargets;

uint8_t automony_status;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comp2018_core");
    ros::NodeHandle n;

    InitPlaces();

    ROS_INFO_STREAM_DELAYED_THROTTLE(2, "ZED Pos:\n\t\t"
                                        << currentPX4Pos.toString() << "\n");

    ROS_INFO_STREAM_DELAYED_THROTTLE(2, "Pos In Mind:\n\t\t"
                                        << POSgolbalInMind.toString() << "\n");


    n.getParam("/comp2018_core/REFRESH_RATE", REFRESH_RATE);

    n.getParam("/comp2018_core/HEIGHT_MAX", HEIGHT_MAX);
    n.getParam("/comp2018_core/SPD_Z_MAX", SPD_Z_MAX);
    n.getParam("/comp2018_core/TOLLERANCE_Z", TOLLERANCE_Z);
    n.getParam("/comp2018_core/SPD_Z_SMALL", SPD_Z_SMALL);
    n.getParam("/comp2018_core/SAFE_RANGE_Z", SAFE_RANGE_Z);

    n.getParam("/comp2018_core/SPD_XY_MAX", SPD_XY_MAX);
    n.getParam("/comp2018_core/TOLLERANCE_XY", TOLLERANCE_XY);
    n.getParam("/comp2018_core/SPD_XY_SMALL", SPD_XY_SMALL);
    n.getParam("/comp2018_core/SAFE_RANGE_XY", SAFE_RANGE_XY);

    n.getParam("/comp2018_core/CRUISE_HEIGHT", CRUISE_HEIGHT);

    n.getParam("/comp2018_core/QR_OFFSET23", QR_OFFSET23);
    n.getParam("/comp2018_core/QR_GLOBOL_OFFSET_X", QR_GLOBOL_OFFSET_X);
    n.getParam("/comp2018_core/QR_GLOBOL_OFFSET_Y", QR_GLOBOL_OFFSET_Y);
    n.getParam("/comp2018_core/TOLLERANCE_Z", TOLLERANCE_Z);
    n.getParam("/comp2018_core/TOLLERANCE_Z", TOLLERANCE_Z);
    n.getParam("/comp2018_core/TOLLERANCE_Z", TOLLERANCE_Z);
    n.getParam("/comp2018_core/TOLLERANCE_Z", TOLLERANCE_Z);
    n.getParam("/comp2018_core/TOLLERANCE_Z", TOLLERANCE_Z);
    n.getParam("/comp2018_core/TOLLERANCE_Z", TOLLERANCE_Z);
    n.getParam("/comp2018_core/TOLLERANCE_Z", TOLLERANCE_Z);

    n.getParam("/comp2018_core/origin_x", MAP_ORIGIN_TO_STRUCT_X);
    n.getParam("/comp2018_core/origin_y", MAP_ORIGIN_TO_STRUCT_Y);
    n.getParam("/comp2018_core/scan_spd", SCAN_SPEED);
    n.getParam("/comp2018_core/scan_delay", SCAN_DELAY);
    n.getParam("/comp2018_core/stop_range", STOP_RANGE);
    n.getParam("/comp2018_core/safe_range", SAFE_RANGE);
    n.getParam("/comp2018_core/brake_coe", BRAKE_COE);
    n.getParam("/comp2018_core/waypoint_group", WAYPOINT_GROUP);
    n.getParam("/comp2018_core/output_gazebo", OUTPUT_GAZEBO);
    n.getParam("/comp2018_core/output_px4", OUTPUT_PX4);
    n.getParam("/comp2018_core/height", HEIGHT);
    n.getParam("/comp2018_core/spd_climb", SPD_CLIMB);
    n.getParam("/comp2018_core/spd_turn", SPD_TURN);
    n.getParam("/comp2018_core/angle_tolerance", ANGLE_TOLERANCE);

    ROS_INFO_STREAM("hahaha" << SPD_XY_MAX);

    ros::Subscriber sub_localPos = n.subscribe("/px4/pose", 1, CB_PX4Pose);
    ros::Subscriber sub_status = n.subscribe("/px4/status", 1, CB_status);

    pub_Pose = n.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1);
    pub_Vel = n.advertise<px4_autonomy::Velocity>("/px4/cmd_vel", 1);
    pub_TakeOff = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1);

    while(ros::ok()) {
        ros::Duration(1).sleep();
    }
}

void MoveBy(float disX, float disY, float disZ, bool usingVelSP) {
    ros::spinOnce();
    vec3f_t dis2go(disX, disY, disZ);
    vec3f_t startPX4Pos = currentPX4Pos;
    SPgolbalInMind = POSgolbalInMind + dis2go;

    //Check the Z
    float tempSetPoint = SPgolbalInMind.z;
    SPgolbalInMind.z = constrainF(tempSetPoint, 0.4f, HEIGHT_MAX);
    if (tempSetPoint != SPgolbalInMind.z) {
        ROS_WARN_STREAM("the Z_sp comes to" << tempSetPoint
                                            << ", constrain to:" << SPgolbalInMind.z);
        dis2go.z = SPgolbalInMind.z - POSgolbalInMind.z;
    }

    //cal the Target in px4 coor
    vec3f_t px4Target = currentPX4Pos + dis2go;

    ros::Rate loopRate(REFRESH_RATE);

    //(de)climb first

    float signZ = 0.0f;
    if (dis2go.z > 0.0) {
        signZ = 1.0f;
        ROS_INFO_STREAM("climb by " << dis2go.z << " to " << SPgolbalInMind.z);
    } else if (dis2go.z < 0.0) {
        signZ = -1.0f;
        ROS_INFO_STREAM("go down by " << -dis2go.z << " to " << SPgolbalInMind.z);
    }
    if (dis2go.z != 0.0) {
        if (usingVelSP) {
            const float spdZ = signZ * SPD_Z_MAX;
            while (ros::ok()) {
                ros::spinOnce();
                POSgolbalInMind.z = currentPX4Pos.z;
                float height2go = fabsf(currentPX4Pos.z - px4Target.z);
                if (height2go > SAFE_RANGE_Z) {
                    // move at fast spd
                    vec3f_t speedSp(0, 0, spdZ);
                    pub_Vel.publish(speedSp.toVelCmd());
                }
                else if(height2go > TOLLERANCE_Z) {
                    // slow down
                    float slowerSpd = signZ * slopeCal(
                            height2go, SAFE_RANGE_Z, TOLLERANCE_Z, SPD_Z_MAX, SPD_Z_SMALL);
                    vec3f_t speedSp(0, 0, slowerSpd);
                    pub_Vel.publish(speedSp.toVelCmd());

                } else {
                    break;
                }
                loopRate.sleep();
            }
        } else {
            const float incZ = (signZ * SPD_Z_MAX) / REFRESH_RATE;
            while (ros::ok()) {
                ros::spinOnce();
                POSgolbalInMind.z = currentPX4Pos.z;
                float heightRest = fabsf(currentPX4Pos.z - px4Target.z);
                if (heightRest > SAFE_RANGE_Z) {
                    // move at fast spd
                    vec3f_t poseSp = startPX4Pos;
                    poseSp.z = currentPX4Pos.z + incZ;
                    pub_Pose.publish(poseSp.toPosCmd());
                }
                else if(heightRest > TOLLERANCE_Z) {
                    // slow down
                    float smallerIncZ = signZ / REFRESH_RATE * slopeCal(
                            heightRest, SAFE_RANGE_Z, TOLLERANCE_Z, SPD_Z_MAX, SPD_Z_SMALL);
                    vec3f_t poseSp = startPX4Pos;
                    poseSp.z = currentPX4Pos.z + smallerIncZ;
                    pub_Pose.publish(poseSp.toPosCmd());
                } else {
                    break;
                }
                loopRate.sleep();
            }
        }
        // finally set the point to target directly
        vec3f_t poseSp = startPX4Pos;
        poseSp.z = px4Target.z;
        pub_Pose.publish(poseSp.toPosCmd());
        if (fabsf(dis2go.z) > SAFE_RANGE_Z) {
            ros::Duration(0.5).sleep(); //TODO might need to be canceled
        } else {
            ros::Duration(0.2).sleep();
        }
        POSgolbalInMind.z = currentPX4Pos.z;
        ROS_INFO_STREAM("arrive height: " << SPgolbalInMind.z);
    }

    //Then move at XY

    if (disX != 0 || disY != 0) {
        ROS_INFO_STREAM("move by: (" << disX << ",\t" << disY << ")");
        ROS_INFO_STREAM("move to: (" << SPgolbalInMind.x << ",\t" << SPgolbalInMind.y << ")");
        if (usingVelSP) {
            while(ros::ok()) {
                ros::spinOnce();
                vec3f_t distVec = px4Target - currentPX4Pos;
                POSgolbalInMind = SPgolbalInMind - distVec;
                float dist = distVec.distXY();
                if (dist > TOLLERANCE_XY) {
                    float spdXY = slopeCal(
                            dist, SAFE_RANGE_XY, TOLLERANCE_XY, SPD_XY_MAX, SPD_XY_SMALL);
                    float spdX = spdXY / dist * distVec.x;
                    float spdY = spdXY / dist * distVec.y;
                    vec3f_t spdCmd(spdX, spdY, 0.0f);
                    pub_Vel.publish(spdCmd.toVelCmd());
                } else {
                    break;
                }
                loopRate.sleep();
            }
        } else {
            while(ros::ok()) {
                ros::spinOnce();
                vec3f_t distVec = px4Target - currentPX4Pos;
                POSgolbalInMind = SPgolbalInMind - distVec;
                float dist = distVec.distXY();
                if (dist > TOLLERANCE_XY) {
                    float spdXY = slopeCal(
                            dist, SAFE_RANGE_XY, TOLLERANCE_XY, SPD_XY_MAX, SPD_XY_SMALL);
                    float stepX = spdXY / dist * distVec.x / REFRESH_RATE;
                    float stepY = spdXY / dist * distVec.y / REFRESH_RATE;
                    vec3f_t poseCmd = currentPX4Pos.operator+({stepX, stepY, 0});
                    pub_Pose.publish(poseCmd.toPosCmd());
                } else {
                    break;
                }
                loopRate.sleep();
            }
        }
        ROS_INFO_STREAM("arrive pos("
                        << SPgolbalInMind.x << ",\t" << SPgolbalInMind.y << ")");
    }

    pub_Pose.publish(px4Target.toPosCmd());
    if (fabsf(dis2go.distXY()) > SAFE_RANGE_XY) {
        ros::Duration(0.5).sleep(); //TODO might need to be canceled
    } else {
        ros::Duration(0.2).sleep();
    }
    POSgolbalInMind = SPgolbalInMind;

    ros::spinOnce();
}

void CB_PX4Pose(const px4_autonomy::Position &msg) {
    currentPX4Pos = msg;
}

void TakeOff() {
    while (automony_status != 1) {
        ros::Duration(1).sleep();
        ROS_INFO_THROTTLE(2, "[CORE] waitting for OFFBOARD");
        ros::spinOnce();
    }

    px4_autonomy::Takeoff cmd_tf;
    cmd_tf.take_off = 1;
    cmd_tf.header.stamp = ros::Time::now();
    pub_TakeOff.publish(cmd_tf);

    while (automony_status != 5) {
        ros::Duration(1).sleep();
        ROS_INFO_THROTTLE(2, "[CORE] taking off...");
        ros::spinOnce();
    }

    POSgolbalInMind.z = currentPX4Pos.z;

    if (fabsf(CRUISE_HEIGHT - POSgolbalInMind.z) > TOLLERANCE_Z) {
        MoveBy(0, 0, CRUISE_HEIGHT - POSgolbalInMind.z, false);
        ROS_INFO_STREAM("Start Climbing to Cruise height:" << CRUISE_HEIGHT);
    }

    Hover();

    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Take off complete");
}

void CB_status(const std_msgs::UInt8 & msg) {
    automony_status = msg.data;
}

void Hover() {
    vec3f_t cmd(0,0,0);
    pub_Vel.publish(cmd.toVelCmd());
}

void InitPlaces() {
    SimpleTarget target(0, 0.0, 0.0, true);
    SimpleTargets.push_back(target);
    target = SimpleTarget(1 , 0.0, 0.0, true);
    SimpleTargets.push_back(target);
    target = SimpleTarget(2 , 0.0, 0.0, true);
    SimpleTargets.push_back(target);
    target = SimpleTarget(3 , 0.0, 0.0, true);
    SimpleTargets.push_back(target);
    target = SimpleTarget(4 , 0.0, 0.0, true);
    SimpleTargets.push_back(target);
    target = SimpleTarget(5 , 0.0, 0.0, true);
    SimpleTargets.push_back(target);
    target = SimpleTarget(6 , 0.0, 0.0, true);
    SimpleTargets.push_back(target);
    target = SimpleTarget(7 , 0.0, 0.0, true);
    SimpleTargets.push_back(target);
    target = SimpleTarget(8 , 0.0, 0.0, true);
    SimpleTargets.push_back(target);
    target = SimpleTarget(9 , 0.0, 0.0, true);
    SimpleTargets.push_back(target);

    vector<QRTarget> layer1;
    layer1.emplace_back(2.5, 0);
    layer1.emplace_back(6.0, 0);
    layer1.emplace_back(9.5, 0);
    layer1.emplace_back(13., 0);
    QRTargets.push_back(std::move(layer1));

    vector<QRTarget> layer2;
    layer2.emplace_back(2.5, 3.5);
    layer2.emplace_back(6.0, 3.5 + QR_OFFSET23);
    layer2.emplace_back(9.5, 3.5 + QR_OFFSET23);
    layer2.emplace_back(13., 3.5 + QR_OFFSET23);
    QRTargets.push_back(std::move(layer2));

    vector<QRTarget> layer3;
    layer3.emplace_back(3.5, 7.0 + QR_OFFSET23);
    layer3.emplace_back(6.0, 7.0 + QR_OFFSET23);
    layer3.emplace_back(9.5, 7.0 + QR_OFFSET23);
    layer3.emplace_back(11.5, 7.0 + QR_OFFSET23);
    QRTargets.push_back(std::move(layer3));

    for (auto & layer: QRTargets) {
        for (auto & QRtarget: layer) {
            QRtarget.globalX += QR_GLOBOL_OFFSET_X;
            QRtarget.globalY += QR_GLOBOL_OFFSET_Y;
            std::cout << QRtarget.globalX << ":" << QRtarget.globalY << "\n";
        }
    }
}



















