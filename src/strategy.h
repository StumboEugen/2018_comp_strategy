//
// Created by stumbo on 18-8-20.
//

#ifndef COMP_STRATEGY_STRATEGY_H
#define COMP_STRATEGY_STRATEGY_H

#define SIGNAL_SLOT_FOUND_CURRENT_TARGET "FirstFoundCurrentTarget"
#define SIGNAL_SLOT_BREAK_MOVE_FOR_FOUND_TARGET "BreakMoveForFoundTarget"
#define SIGNAL_SLOT_QR_CODE_FOUND "QRCodeFound"

#include "ros/ros.h"

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <px4_autonomy/Takeoff.h>
#include <px4_autonomy/Velocity.h>
#include <px4_autonomy/Position.h>

#include <sstream>
#include <string>

#include "SimpleTarget.h"
#include "tools.h"

enum CamMode {DOWNONLY = 1, ZEDBOTH = 2, ZEDCIRCLE = 3, ZEDQR = 4};

void MoveBy(float disX, float disY, float disZ = 0.0, bool usingVelSP = true);
void MoveBy(vec3f_t dist, bool usingVelSp = true);
void MoveTo(float targetX, float targetY, float targetZ, bool usingVelSP = true);
void MoveTo(vec3f_t target, bool usingVelSp = true);
void CB_PX4Pose(const px4_autonomy::Position &msg);
void CB_status(const std_msgs::UInt8 & msg);
void CB_Camera(const geometry_msgs::PoseStamped &msg);
void TakeOff();
void Hover();
void InitPlaces();
int CheckSignal(std::string);
void SendCamCMD(CamMode);
void Land();
void AimBoardDown();
void getParas(ros::NodeHandle & n);
void OutputInfoAtRate(int rate = 2);

#endif //COMP_STRATEGY_STRATEGY_H
