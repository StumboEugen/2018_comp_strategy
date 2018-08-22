#include "ros/ros.h"
#include "strategy.h"
#include "SimpleTarget.h"
#include <vector>
#include <map>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include "tools.h"

#define FAKE_TEST_ENABLE true

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
float X_RESTRICT_LEFT = -6.5f;
float X_RESTRICT_RIGHT = 6.5;
float DETECT_OFFSET = 1.0;

float CIRCLE_SEARCH_OFFSET_Y = 0.75;
float CIRCLE_PASS_DIST = 1.0;
float LOW_CIRCLE_BOARD_HEIGHT = 0.365;
float HIGH_CIRCLE_BOARD_HEIGHT = 0.365;
float FIRST_RANGE_BOARD_AIM = 0.4;
float CIRCLE_BOARD_AIM_TOLLERANCE = 0.1;
float AIM_BOARD_P = 0.6f;
int AIM_BOARD_SATISFIED_COUNT = 5;
float CIRCLE_CLIMB_SLEEP_TIME = 0.75f;

float BOARD_AIM_HEIGHT = 1.0f;
float BOARD_AIM_TOLLERANCE = 0.3f;

float QR_SEARCH_OFFSET = 1.0f;
float QR_TOLLANCE = 0.3f;
float QR_CRUISE_HEIGHT = 1.15f;

bool DYNAMIC_DETECT_ENABLE = false;
float DYNAMIC_DETECT_TOLLERANCE = 0.5f;
float MATCH_DETECT_TOLLERANCE = 1.0;

using std::vector;
using ros::Publisher;

std::map<std::string, int> Signals;

Publisher pub_Pose;
Publisher pub_Vel;
Publisher pub_TakeOff;
Publisher pub_camCmd;
Publisher pub_Fake;

vec3f_t POSgolbalInMind = {0.0, 0.0, 0.0};
vec3f_t SPgolbalInMind = {0.0, 0.0, 0.0};
vec3f_t currentPX4Pos = {0.0, 0.0, 0.0};

vec3f_t BoardPos;
vec3f_t CirclePos;
vec3f_t QRPos;

vector<SimpleTarget> simpleTargets;
vector<vector<QRTarget>> QRTargets;

uint8_t automony_status;
CamMode currentCamMode;

enum SearchDirection {TOWARD_LEFT = -1, UNSET = 0, TOWARD_RIGHT = 1};

int currentTargetID = 0;

int main(int argc, char **argv)
{
    if (FAKE_TEST_ENABLE) {
        BoardPos = {0.0, 0.0, -1.0f};
        CirclePos = {0.0, 1.0, 0.0};
        QRPos = {0.0, 1.0, 0.0};
    } else {
        BoardPos = {0.0, 0.0, 0.0};
        CirclePos = {0.0, 0.0, 0.0};
        QRPos = {0.0, 0.0, 0.0};
    }

    ros::init(argc, argv, "comp2018_core");
    ros::NodeHandle n;

    getParas(n);

    InitPlaces();

    /**
     * init signals
     */
    Signals.insert({SIGNAL_SLOT_FOUND_CURRENT_TARGET, 0});
    Signals.insert({SIGNAL_SLOT_BREAK_MOVE_FOR_FOUND_TARGET, 0});
    Signals.insert({SIGNAL_SLOT_QR_CODE_FOUND, 0});

    ros::Subscriber sub_localPos = n.subscribe("/px4/pose", 1, CB_PX4Pose);
    ros::Subscriber sub_status = n.subscribe("/px4/status", 1, CB_status);
    ros::Subscriber sub_camPose = n.subscribe("/px4/camPos", 1, CB_Camera);

    if (FAKE_TEST_ENABLE) {
        ROS_WARN_STREAM("[CORE][FAKE] You are now at #FAKE TEST# mode!");
        pub_Fake = n.advertise<px4_autonomy::Position>("/px4/pose", 1);
    }
    pub_Pose = n.advertise<px4_autonomy::Position>("/px4/cmd_pose", 1);
    pub_Vel = n.advertise<px4_autonomy::Velocity>("/px4/cmd_vel", 1);
    pub_TakeOff = n.advertise<px4_autonomy::Takeoff>("/px4/cmd_takeoff", 1);
    pub_camCmd = n.advertise<std_msgs::UInt8>("/px4/camCmd", 1);

    ROS_INFO_STREAM("[CORE] Init complete");

    int circleCount = 0;

    /**
     * Stage 1, Board & Circle
     */
    for (int targetID = 1; targetID < simpleTargets.size(); targetID++) {
        ROS_INFO_STREAM("[CORE] Moving On Next Target: #" << targetID << " globalY:"
                                         << simpleTargets[targetID].globalY);
        bool foundTarget = false;
        currentTargetID = targetID; //tell other parts, what are we looking for right now
        Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] = targetID - 1;

        /**
         * if last target is Board, Cail the pos
         */
        const auto & lastTarget = simpleTargets[targetID - 1];
        std::stringstream ss;
        ss << "[CORE] We just leave Pose#" << targetID - 1;
        if (lastTarget.detectedBoard) {
            ss << ", it's a board";
        } else if (lastTarget.detectedCircle) {
            ss << ", it's a circle";
        } else {
            ss << "it's nothing?!?!";
        }
        ss << " at Pos#" << lastTarget.possiblePose << " with coor: "
                        << lastTarget.poses[lastTarget.possiblePose].toString();
        ROS_INFO_STREAM(ss.str());
        if (lastTarget.detectedBoard) {
            ROS_INFO_STREAM("[CORE] last one is Board, take off");
            TakeOff();
            if (targetID != 1) {
                AimBoardDown();
                ros::spinOnce();

                size_t lastPose = lastTarget.possiblePose;
                POSgolbalInMind = lastTarget.poses[lastPose] - BoardPos;
                POSgolbalInMind.z = currentPX4Pos.z;
                ROS_INFO_STREAM("[CORE] Cail complete, Ideal Pose:"
                                << POSgolbalInMind.toString());
                ROS_INFO_STREAM("[CORE] Cail complete, ZED Pose:"
                                << currentPX4Pos.toString());
            }
        }

        auto & target = simpleTargets[targetID];

        /**
         * Search Progress
         */
        while (!foundTarget) {

            /**
             * decide the first pose setpoint
             */
            bool needOffsetY; // add offset to avoid crash with circle
            vec3f_t firstTargetPos;

            /** z at cruise_height*/
            firstTargetPos.z = CRUISE_HEIGHT;

            /** y */
            if (target.onlyBoard) {
                ROS_INFO_STREAM("[CORE] Target#" << targetID << " is Board only");
                needOffsetY = false;
                SendCamCMD(DOWNONLY);
            } else {
                if (!DYNAMIC_DETECT_ENABLE) {
                    ROS_INFO_STREAM("[CORE] Target#" << targetID << " is board UNKNOW");
                    SendCamCMD(ZEDBOTH);
                    needOffsetY = true;
                } else {
                    if (target.detectedCircle) {
                        SendCamCMD(ZEDBOTH);
                        needOffsetY = true;
                        ROS_INFO_STREAM("[CORE][DYNAMIC_DETECTED] Target#"
                                                << targetID << " is circle");
                    }
                    else if (target.detectedBoard) {
                        SendCamCMD(DOWNONLY);
                        needOffsetY = false; //TODO check safety
                        ROS_INFO_STREAM("[CORE][DYNAMIC_DETECTED] Target#"
                                                << targetID << " is board");
                    } else {
                        SendCamCMD(ZEDBOTH);
                        needOffsetY = true;
                        ROS_INFO_STREAM("[CORE][DYNAMIC_DETECTED] Target#"
                                                << targetID << " is unknown");
                    }
                }
            }

            /** x y */
            if (DYNAMIC_DETECT_ENABLE) {
                firstTargetPos.x = target.poses[target.possiblePose].x;
                firstTargetPos.y = target.poses[target.possiblePose].y;
                ROS_INFO_STREAM("[CORE][DYNAMIC_DETECTED] Moving to:"
                                << firstTargetPos.toString()
                                << "\n which is Pose#" << target.possiblePose);
            } else {
                if (POSgolbalInMind.x > 0) {
                    firstTargetPos.x = target.poses.back().x;
                    firstTargetPos.y = target.poses.back().y;
                    ROS_INFO_STREAM("[CORE] Moving to right side:"
                                            << firstTargetPos.toString());
                } else {
                    firstTargetPos.x = target.poses.front().x;
                    firstTargetPos.y = target.poses.front().y;
                    ROS_INFO_STREAM("[CORE] Moving to left side:"
                                            << firstTargetPos.toString());
                }
            }

            if (needOffsetY) {
                firstTargetPos.y -= DETECT_OFFSET;
                ROS_INFO_STREAM("[CORE] Add y offsetL: " << -DETECT_OFFSET);
            }

            /** this is the first move*/
            MoveTo(firstTargetPos);
            if (CheckSignal(SIGNAL_SLOT_FOUND_CURRENT_TARGET) == targetID) {
                //This means we found the Target
                ROS_INFO_STREAM("[CORE] Found target at the first Move!");
                Signals[SIGNAL_SLOT_BREAK_MOVE_FOR_FOUND_TARGET] = 0;
                break;
            }

            ROS_INFO_STREAM("[CORE] Not found at the first Move");
            /** first move misfound, keep on going*/
            if (target.onlyBoard) {
                SendCamCMD(DOWNONLY);
            } else {
                SendCamCMD(ZEDBOTH);
                needOffsetY = true;
            }
            SearchDirection searchDirection = UNSET;
            size_t currentPose = 0;


            /**
             * Check where am I
             */
            auto fixResult = target.checkForClosestPose(POSgolbalInMind);
            if (fixResult.second > 1.5f) {
                ROS_ERROR_STREAM("[CORE] YOU MAY RUN INTO A WRONG LINE!!!\n current POS:"
                                         << POSgolbalInMind.toString()
                                         << "\ncurrent Target ID:" << currentTargetID
                                         << "\nthe Closest pose#" << fixResult.first
                                         << "\nPos: " << target.poses[fixResult.first].toString()
                                         << "\n range:" << fixResult.second);
            }
            currentPose = fixResult.first;
            ROS_WARN_STREAM("[CORE] We think we are at Pose# " << currentPose << "POS:"
                                    << POSgolbalInMind.toString());

            /**
             * Judge which side to go first
             */
            if (currentPose == 0 || currentPose == 1) {
                searchDirection = TOWARD_RIGHT;
            } else if (currentPose == 3 || currentPose == 4) {
                searchDirection = TOWARD_LEFT;
            } else if (currentPose == 2) {
                float disRight = (POSgolbalInMind - target.poses.back()).distXY();
                float disLeft = (POSgolbalInMind - target.poses.front()).distXY();
                if (disRight > disLeft) {
                    searchDirection = TOWARD_RIGHT;
                } else {
                    searchDirection = TOWARD_LEFT;
                }
            }

            if (searchDirection == TOWARD_RIGHT) {
                ROS_INFO_STREAM("[CORE] Decide to go right");
            } else if (searchDirection == TOWARD_LEFT) {
                ROS_INFO_STREAM("[CORE] Decide to go left");
            } else {
                ROS_FATAL_STREAM("[CORE] Not Decide to go left or right! At Pos # "
                                 << currentPose << " POSglobal: " << POSgolbalInMind.toString());
            };

            bool visited0 = false;
            bool visited4 = false;

            //while of the round(5 places in a round)
            while(true) {
                vec3f_t nextPos;
                if (currentPose == 0) {
                    ROS_INFO_STREAM("[CORE] At most left");
                    searchDirection = TOWARD_RIGHT;
                    visited0 = true;
                }
                else if (currentPose == 4) {
                    ROS_INFO_STREAM("[CORE] At most right");
                    searchDirection = TOWARD_LEFT;
                    visited4 = true;
                }

                if (searchDirection == TOWARD_RIGHT) {
                    ROS_INFO_STREAM("[CORE] Moving RIGHT");
                    currentPose++;
                }
                else if (searchDirection == TOWARD_LEFT) {
                    ROS_INFO_STREAM("[CORE] Moving LEFT");
                    currentPose--;
                }

                if (visited0 && visited4) {
                    ROS_INFO_STREAM("[CORE] Each side both arrived, now cancel SearchOffset(Y)");
                    needOffsetY = false;
                }

                nextPos = target.poses[currentPose];
                if (needOffsetY) nextPos.y -= DETECT_OFFSET;
                nextPos.z = CRUISE_HEIGHT;

                MoveTo(nextPos);
                if (CheckSignal(SIGNAL_SLOT_FOUND_CURRENT_TARGET) == targetID) {
                    ROS_WARN_STREAM("[CORE] See target! Current Pose:"
                                    << POSgolbalInMind.toString());
                    Signals[SIGNAL_SLOT_BREAK_MOVE_FOR_FOUND_TARGET] = 0;
                    foundTarget = true;
                    break;
                }
                ROS_INFO_STREAM("[CORE] NOT FIND TARGET");
                ros::Duration(0.75).sleep();
            }
        }

        /**
         * Move through circle
         */
        if(target.detectedCircle) {
            SendCamCMD(ZEDCIRCLE);
            ROS_INFO_STREAM("[CORE] found Circle! sleep 0.5 sec then move closer");
            ros::spinOnce();
            ros::Duration(0.5).sleep();

            //first move to a closer range
            while (true) {
                ros::spinOnce();
                vec3f_t localDis = CirclePos;
                localDis.y -= CIRCLE_SEARCH_OFFSET_Y;
                MoveBy(localDis);
                ROS_INFO_STREAM("[CORE][AIM_CIRCLE_BOARD] Circle Pos: " << CirclePos.toString());
                if (localDis.len() < FIRST_RANGE_BOARD_AIM) {
                    ROS_INFO_STREAM("[CORE] Moved into a closer range");
                    break;
                }
                ros::Duration(0.2).sleep();
            }

            int staisfiedCount = 0;
            while (true) {
                ros::spinOnce();
                vec3f_t localDis = CirclePos;
                localDis.y -= CIRCLE_SEARCH_OFFSET_Y;
                ROS_INFO_STREAM("[CORE][AIM_CIRCLE_BOARD] Circle Pos: " << CirclePos.toString());
                vec3f_t nextPos = currentPX4Pos + localDis * AIM_BOARD_P;
                pub_Pose.publish(nextPos.toPosCmd());
                if (FAKE_TEST_ENABLE) {
                    pub_Fake.publish(nextPos.toPosCmd());
                }
                ros::Duration(0.2).sleep();
                if (localDis.len() < CIRCLE_BOARD_AIM_TOLLERANCE) {
                    staisfiedCount++;
                    ROS_INFO_STREAM("[CORE] Very close, Satisifed count:" << staisfiedCount);
                    if (staisfiedCount >= AIM_BOARD_SATISFIED_COUNT) {
                        break;
                        ROS_INFO_STREAM("[CORE] Circle Board Aim Complete!");
                    }
                    continue;
                }
                staisfiedCount = 0;
            }

            ros::spinOnce();
            float disHigh = fabsf(HIGH_CIRCLE_BOARD_HEIGHT - currentPX4Pos.z);
            float disLow = fabsf(LOW_CIRCLE_BOARD_HEIGHT - currentPX4Pos.z);
            if (disHigh < disLow) {
                ROS_INFO_STREAM("[CORE] It's a high circle, climbing...");
                MoveBy(0, 0, HIGH_CIRCLE_BOARD_HEIGHT - currentPX4Pos.z, false);
            } else {
                ROS_INFO_STREAM("[CORE] It's a low circle, climbing...");
                MoveBy(0, 0, HIGH_CIRCLE_BOARD_HEIGHT - currentPX4Pos.z, false);
            }
            ROS_INFO_STREAM("[CORE] Sleep" << CIRCLE_CLIMB_SLEEP_TIME << "sec Then Pass");
            ros::Duration(CIRCLE_CLIMB_SLEEP_TIME).sleep();
            ROS_INFO_STREAM("[CORE] Passing Circle...");
            ros::spinOnce();
            MoveBy(0, CIRCLE_PASS_DIST + CIRCLE_SEARCH_OFFSET_Y, 0, false);  //check
            ROS_INFO_STREAM("[CORE] Pass Complete");

            circleCount++;
            if (circleCount >= 3) {
                for (int i = targetID + 1; i < simpleTargets.size(); i++) {
                    simpleTargets[i].onlyBoard = true;
                }
                ROS_WARN_STREAM("[CORE] This is the 3rd Circle, "
                                "consider the rest all to be board");
            }
        /**
        * Land on Board
        */
        } else if (target.detectedBoard) {
            ROS_INFO_STREAM("[CORE] found Board! sleep 0.5 sec then move closer");
            ros::spinOnce();
            ros::Duration(0.5).sleep();

            if (currentCamMode == ZEDBOTH) {
                ros::spinOnce();
                vec3f_t moveSp = BoardPos;
                moveSp.z += BOARD_AIM_HEIGHT;
                MoveBy(moveSp);
            }

            SendCamCMD(DOWNONLY);
            ros::spinOnce();
            ros::Duration(0.2).sleep();

            ROS_INFO_STREAM("[CORE] Above Board, Start aiming");
            AimBoardDown();
            ROS_INFO_STREAM("[CORE] Aim Complete, start landing");
            Land();
            ROS_INFO_STREAM("[CORE] Land Complete");
        /**
         * Error!
         */
        } else {
            ROS_ERROR_STREAM("[CORE] PASS SEARCH PROGRESS WITH NO DETECTION!");
            targetID--;
            continue;
        }

        /**
         * after works
         */
        auto checkRes = target.checkForClosestPose(POSgolbalInMind);
        target.possiblePose = checkRes.first;
        vec3f_t place = target.poses[checkRes.first];
        ROS_ERROR_STREAM_COND(checkRes.second > 1.5f,
                              "[CORE] YOU MAY RUN INTO A WRONG LINE!!! current POS:\n"
                              << POSgolbalInMind.toString()
                              << "current Target ID：" << currentTargetID
                              << "\n cloest spot:" << place.toString());

        // Cali pose
        POSgolbalInMind = place;
        if (target.detectedCircle) {
            ROS_INFO_STREAM("[CORE] Not a board, have to self Cail...");
            POSgolbalInMind.y += CIRCLE_PASS_DIST ;
            POSgolbalInMind.z = currentPX4Pos.z;
        }
    }

    /**
     * Board & Circle Complete
     *
     * Take off and cail of the last pos
     */
    ROS_INFO_STREAM("[CORE] Last Take off & Aim, then we move to QR poses");
    TakeOff();
    AimBoardDown();
    ros::spinOnce();

    size_t lastPose = simpleTargets.back().possiblePose;
    POSgolbalInMind = simpleTargets.back().poses[lastPose] - BoardPos;
    POSgolbalInMind.z = currentPX4Pos.z;
    ROS_INFO_STREAM("[CORE] Cail complete, Ideal Pose:"
                            << POSgolbalInMind.toString());
    ROS_INFO_STREAM("[CORE] Cail complete, ZED Pose:"
                            << currentPX4Pos.toString());

    ROS_INFO_STREAM("[CORE] Now we are going to the QR Zone");

    currentTargetID = -1;

    /**
     * Stage 2 Detect QR Code
     */

    SendCamCMD(ZEDQR);
    SearchDirection QRSearchDir;
    int QRcount = 0;

    /**
     * loop of 3 layers
     */
    for(int layer = 0; layer < QRTargets.size(); layer++) {
        ROS_INFO_STREAM("[CORE] Current layer is #" << layer);
        int currentQRNum;
        auto & QRLayer = QRTargets[layer];
        float disLeft = fabsf(QRLayer[0].globalPos.x - POSgolbalInMind.x);
        float disRight = fabsf(POSgolbalInMind.x - QRLayer[3].globalPos.x);
        if (disLeft > disRight) {
            ROS_INFO_STREAM("[CORE] Choose to move from the right side");
            QRSearchDir = TOWARD_LEFT;
            currentQRNum = static_cast<int>(QRTargets.size());
        } else {
            ROS_INFO_STREAM("[CORE] Choose to move from the left side" << layer);
            QRSearchDir = TOWARD_RIGHT;
            currentQRNum = 0;
        }

        /**
         * loop of a layer
         */
        while (true) {
            currentTargetID = layer * 4 + currentQRNum;
            Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] = currentTargetID - 1;
            auto targetPos = QRLayer[currentQRNum].globalPos;
            targetPos.y -= QR_SEARCH_OFFSET;
            targetPos.z = QR_CRUISE_HEIGHT;

            ROS_INFO_STREAM("[CORE] Move to: " << targetPos.toString());
            MoveTo(targetPos);

            /**
             * if found Tree, move to it
             */
            if (CheckSignal(SIGNAL_SLOT_FOUND_CURRENT_TARGET) == currentTargetID) {
                ROS_INFO_STREAM("[CORE] We detect the Tree!");
                Hover();
                ros::spinOnce();
                ros::Duration(0.4).sleep();
                ros::spinOnce();
                vec3f_t disSp = QRPos;
                disSp.y -= QR_SEARCH_OFFSET;
                disSp.z = QR_CRUISE_HEIGHT - currentPX4Pos.z;
                ROS_INFO_STREAM("[CORE] Try to Move closer");
                MoveBy(disSp);
                ros::spinOnce();
                POSgolbalInMind = QRLayer[currentQRNum].globalPos - QRPos;
                POSgolbalInMind.z = currentPX4Pos.z;

                /**
                 * Afterwards if find QR, capture the QR
                 */
                if (CheckSignal(SIGNAL_SLOT_QR_CODE_FOUND) == currentTargetID) {
                    ROS_INFO_STREAM("[CORE] Here is an ER WEI MA!");
                    QRcount ++;
                    while (true) {
                        ros::spinOnce();
                        vec3f_t posSP = QRPos;
                        posSP.y -= QR_SEARCH_OFFSET;
                        if (posSP.len() < QR_TOLLANCE) {
                            ROS_INFO_STREAM("[CORE_QR] Try to move more closer, QR_POSE: "
                                            << QRPos.toString());
                            POSgolbalInMind = QRLayer[currentQRNum].globalPos - QRPos;
                            POSgolbalInMind.z = currentPX4Pos.z;
                            break;
                        }
                        MoveBy(posSP);
                        ros::Duration(0.33).sleep();
                        ros::spinOnce();
                    }
                }
                /**
                 * do not see the tree! do nothing, moving on, good luck
                 */
            } else {
                QRLayer[currentQRNum].disappear = true;
                ROS_INFO_STREAM("[CORE] Tree at Layer" << layer <<
                                             " pos" << currentQRNum << "DISSAPPEAR!");
            }

            if (currentQRNum == QRTargets[layer].size() - 1 && QRSearchDir == TOWARD_RIGHT) {
                if (layer != QRTargets.size() - 1) {
                    ROS_INFO_STREAM("[CORE] Layer#" << layer << " finish, "
                                                               "Leaving from RIGHT site...");
                    MoveBy(-1.5f, 0.f, 0.0, false);
                    MoveBy(0.f, 3.5f, 0.0, false);
                    break;
                } else {
                    ROS_INFO_STREAM("[CORE] QR Finished, Moving to land...");
                    MoveBy(1.9f, 0.f, 0.0);
                    MoveBy(0.f, 2.9f, 0.0);
                    break;
                }
            }

            if (currentQRNum == 0 && QRSearchDir == TOWARD_LEFT) {
                if (layer != QRTargets.size() - 1) {
                    ROS_INFO_STREAM("[CORE] Layer" << layer << " finish, "
                                                               "Leaving from LEFT site...");
                    MoveBy(1.5f, 0.f, 0.0, false);
                    MoveBy(0.f, 3.5f, 0.0, false);
                    break;
                } else {
                    ROS_INFO_STREAM("[CORE] QR Finished, Moving to land...");
                    MoveBy(-1.9f, 0.f, 0.0);
                    MoveBy(0.f, 2.9f, 0.0);
                    break;
                }
            }

            if (QRSearchDir == TOWARD_LEFT) {
                currentQRNum--;
            }

            if (QRSearchDir == TOWARD_RIGHT) {
                currentQRNum++;
            }
        }
    }

    ROS_INFO_STREAM("[CORE] Aiming the last Board...");
    AimBoardDown();
    Land();
    ROS_INFO_STREAM("[CORE] MISSION ACCOMPLISHED");

    return 0;
}

void MoveBy(float disX, float disY, float disZ, bool usingVelSP) {
    if (FAKE_TEST_ENABLE) {
        usingVelSP = false;
    }
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    vec3f_t dis2go(disX, disY, disZ);
    ROS_INFO_STREAM("[CORE_MOVE] dist2go :" << dis2go.toString());
    vec3f_t startPX4Pos = currentPX4Pos;

    //set the Target in Ideal coor
    SPgolbalInMind = POSgolbalInMind + dis2go;

    /**
     * Check the Z
     */
    float tempSetPoint = SPgolbalInMind.z;
    SPgolbalInMind.z = constrainF(tempSetPoint, HEIGHT_MAX, 0.4f);
    if (tempSetPoint != SPgolbalInMind.z) {
        ROS_WARN_STREAM("[CORE_MOVE] the Z_sp comes to" << tempSetPoint
                                            << ", constrain to:" << SPgolbalInMind.z);
        dis2go.z = SPgolbalInMind.z - POSgolbalInMind.z;
    }

    //cal the Target in px4 coor
    vec3f_t px4Target = currentPX4Pos + dis2go;

    ros::Rate loopRate(REFRESH_RATE);

    /**
     * (de)climb Z first
     */
    float signZ = 0.0f;
    if (dis2go.z > 0.0) {
        signZ = 1.0f;
        ROS_INFO_STREAM("[CORE_MOVE] climb by " << dis2go.z << " to " << SPgolbalInMind.z);
    } else if (dis2go.z < 0.0) {
        signZ = -1.0f;
        ROS_INFO_STREAM("[CORE_MOVE] go down by " << -dis2go.z << " to " << SPgolbalInMind.z);
    }

    if (dis2go.z != 0.0) {
        while (ros::ok()) {
            ros::spinOnce();
            OutputInfoAtRate();
            POSgolbalInMind.z = currentPX4Pos.z;
            float height2go = fabsf(currentPX4Pos.z - px4Target.z);
            if (height2go > TOLLERANCE_Z) {
                float spdZ = signZ * slopeCal(
                        height2go, SAFE_RANGE_Z, TOLLERANCE_Z, SPD_Z_MAX, SPD_Z_SMALL);
                if (usingVelSP) {
                    vec3f_t speedSp(0, 0, spdZ);
                    pub_Vel.publish(speedSp.toVelCmd());
                } else {
                    float IncZ = spdZ / REFRESH_RATE;
                    vec3f_t poseSp = startPX4Pos;
                    poseSp.z = currentPX4Pos.z + IncZ;
                    pub_Pose.publish(poseSp.toPosCmd());
                    if (FAKE_TEST_ENABLE) {
                        pub_Fake.publish(poseSp.toPosCmd());
                    }
                }
            } else {
                break;  //move finished
            }
            loopRate.sleep();
        }
        /**
         * finally set the point to target directly
         */
        vec3f_t poseSp = startPX4Pos;
        poseSp.z = px4Target.z;
        pub_Pose.publish(poseSp.toPosCmd());
        if (FAKE_TEST_ENABLE) {
            pub_Fake.publish(poseSp.toPosCmd());
        }
        ROS_INFO_STREAM("[CORE_MOVE] arrive height: " << SPgolbalInMind.z);

        float abd = fabsf(dis2go.z);
        float sleepTime = slopeCal(abd, SAFE_RANGE_Z, TOLLERANCE_Z, 0.5, 0.1);
        ROS_INFO_STREAM("[CORE_MOVE] sleep " << sleepTime << "sec for stable");
        ros::Duration(sleepTime).sleep();

        //POS_SP
        POSgolbalInMind.z = currentPX4Pos.z;
        SPgolbalInMind.z = currentPX4Pos.z;
    }
    // end if (dis2go.z != 0.0)

    /**
     * Then move at XY plane
     */
    if (disX != 0 || disY != 0) {
        ROS_INFO_STREAM("[CORE_MOVE] move by: (" << disX << ",\t" << disY << ")");
        ROS_INFO_STREAM("[CORE_MOVE] move to: (" << SPgolbalInMind.x << ",\t" << SPgolbalInMind.y << ")");
            while(ros::ok()) {
                ros::spinOnce();
                OutputInfoAtRate();
                // cal the dist in px4 coor
                vec3f_t distVec = px4Target - currentPX4Pos;
                // update the pos in Ideal coor
                POSgolbalInMind = SPgolbalInMind - distVec;

                /**
                 * We found the current Target, STOP & HOVER
                 */
                if (currentTargetID == Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET]) {
                    Signals[SIGNAL_SLOT_BREAK_MOVE_FOR_FOUND_TARGET] = 1;
                    ROS_INFO_STREAM("[CORE_MOVE] move interrupted for found current target:"
                                    << currentTargetID);
                    ROS_INFO_STREAM("[CORE_MOVE] current POSgolbal: "
                                    << POSgolbalInMind.toString());
                    ROS_INFO_STREAM("[CORE_MOVE] current ZED Pos: "
                                            << currentPX4Pos.toString());
                    Hover();
                    return;
                }

                float dist = distVec.distXY();
                if (dist > TOLLERANCE_XY) {
                    float spdXY = slopeCal(
                            dist, SAFE_RANGE_XY, TOLLERANCE_XY, SPD_XY_MAX, SPD_XY_SMALL);
                    float spdX = spdXY / dist * distVec.x;
                    float spdY = spdXY / dist * distVec.y;

                    if (usingVelSP) {
                        vec3f_t spdCmd(spdX, spdY, 0.0f);
                        pub_Vel.publish(spdCmd.toVelCmd());
                    } else {
                        float stepX = spdX / REFRESH_RATE;
                        float stepY = spdY / REFRESH_RATE;
                        vec3f_t poseCmd = currentPX4Pos.operator+({stepX, stepY, 0});
                        pub_Pose.publish(poseCmd.toPosCmd());
                        if (FAKE_TEST_ENABLE) {
                            pub_Fake.publish(poseCmd.toPosCmd());
                        }

                    }
                } else {
                    break;
                }
                loopRate.sleep();
            }
        ROS_INFO_STREAM("[CORE_MOVE] arrive pos("
                        << SPgolbalInMind.x << ",\t" << SPgolbalInMind.y << ")");
    }
    //end if (disX != 0 || disY != 0)

    pub_Pose.publish(px4Target.toPosCmd());
    if (FAKE_TEST_ENABLE) {
        pub_Fake.publish(px4Target.toPosCmd());
    }

    float abd = fabsf(dis2go.distXY());
    float sleepTime = slopeCal(abd, SAFE_RANGE_XY, TOLLERANCE_XY, 0.5, 0.1);
    ROS_INFO_STREAM("[CORE_MOVE] sleep " << sleepTime << "sec for stable");
    ros::Duration(sleepTime).sleep();

    ros::spinOnce();
    POSgolbalInMind = SPgolbalInMind;
    POSgolbalInMind.z = currentPX4Pos.z;

    ros::spinOnce();
}

inline void MoveTo(float targetX, float targetY, float targetZ, bool usingVelSP) {
    MoveBy( targetX - POSgolbalInMind.x,
            targetY - POSgolbalInMind.y,
            targetZ - POSgolbalInMind.z, usingVelSP);
}

inline void MoveBy(vec3f_t dist, bool usingVelSp) {
    MoveBy(dist.x, dist.y, dist.z, usingVelSp);
}

inline void MoveTo(vec3f_t target, bool usingVelSp) {
    vec3f_t dist = target - POSgolbalInMind;
    MoveBy(dist, usingVelSp);
}

void TakeOff() {
    if (FAKE_TEST_ENABLE) {
        ROS_WARN_STREAM("[CORE][FAKE] take off");
        ros::Duration(2).sleep();
        ROS_WARN_STREAM("[CORE][FAKE] take off complete");
        POSgolbalInMind.z = CRUISE_HEIGHT;
        pub_Fake.publish(POSgolbalInMind.toPosCmd());
        return;
    }

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
        ROS_INFO_STREAM("[CORE] Moving to Cruise height:" << CRUISE_HEIGHT);
    }

    Hover();

    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Take off complete");
}


void Hover() {
    vec3f_t cmd(0,0,0);
    pub_Vel.publish(cmd.toVelCmd());
    ros::spinOnce();
}

int CheckSignal(const std::string sigName) {
    if (Signals.find(sigName) == Signals.end()) {
        Signals.insert({sigName, 0});
        return 0;
    }
    return Signals[sigName];
}

void SendCamCMD(CamMode camMode) {
    std_msgs::UInt8 msg;
    msg.data = camMode;
    pub_camCmd.publish(msg);
    currentCamMode = camMode;
}

void Land() {
    if (FAKE_TEST_ENABLE) {
        ROS_WARN_STREAM("[CORE][FAKE] landing");
        ros::Duration(2).sleep();
        ROS_WARN_STREAM("[CORE][FAKE] land complete");
        POSgolbalInMind.z = 0;
        pub_Fake.publish(POSgolbalInMind.toPosCmd());
        return;
    }
    while (automony_status != 4 && automony_status != 5) {
        ros::Duration(1).sleep();
        ROS_INFO("[CORE] Ready to land, but status not 4 or 5!!");
        ros::spinOnce();
    }

    px4_autonomy::Takeoff cmd_tf;
    cmd_tf.take_off = 2;
    cmd_tf.header.stamp = ros::Time::now();
    pub_TakeOff.publish(cmd_tf);

    while (automony_status != 1) {
        ros::Duration(1).sleep();
        ROS_INFO("[CORE] landing...");
        ros::spinOnce();
    }

    ROS_INFO("[CORE] landed");
}

void AimBoardDown() {
    while (true) {
        SendCamCMD(DOWNONLY);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        vec3f_t localDis = BoardPos;
        localDis.z += BOARD_AIM_HEIGHT;
        MoveBy(localDis);
        ROS_INFO_STREAM("[CORE][AIM_BOARD] Board Pos: " << BoardPos.toString());
        if (localDis.distXY() < BOARD_AIM_TOLLERANCE) {
            break;
        }
        ros::Duration(0.2).sleep();
    }
}

void CB_PX4Pose(const px4_autonomy::Position &msg) {
    currentPX4Pos = msg;
}

void CB_status(const std_msgs::UInt8 & msg) {
    automony_status = msg.data;
}

void CB_Camera(const geometry_msgs::PoseStamped &msg) {
    vec3f_t poseFromCam;
    poseFromCam.x = static_cast<float>(msg.pose.position.x);
    poseFromCam.y = static_cast<float>(msg.pose.position.y);
    poseFromCam.z = static_cast<float>(msg.pose.position.z);
    bool foundBoard = msg.pose.orientation.x > 0.5;
    bool foundCircle = msg.pose.orientation.y > 0.5;
    bool foundTree = msg.pose.orientation.z > 0.5;
    bool foundQR = msg.pose.orientation.w > 0.5;

    auto & currentSimpleTarget = simpleTargets[currentTargetID];
    SimpleTarget simpleTemp = currentSimpleTarget;
    bool firstDeteced = false;
    auto & currentQRTarget = QRTargets[currentTargetID / 4][currentTargetID % 4];

    /**
     * Only look down find Board
     */
    if (currentCamMode == DOWNONLY) {
        auto searchRes = simpleTargets[currentTargetID].checkForClosestPose(
                poseFromCam + POSgolbalInMind);
        if (searchRes.second < MATCH_DETECT_TOLLERANCE) {
            if (Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] != currentTargetID) {
                Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] = currentTargetID;
                firstDeteced = true;
            } else {
                firstDeteced = false;
            }

            BoardPos = poseFromCam;
            simpleTemp.possiblePose = searchRes.first;
            simpleTemp.possibleX = (poseFromCam + POSgolbalInMind).x;
            simpleTemp.detectedBoard = true;
        }
    }
    /**
     * Look for both Board & Circle
     */
    else if (currentCamMode == ZEDBOTH) {
        if (foundBoard) {
            auto searchRes = simpleTargets[currentTargetID].checkForClosestPose(
                    poseFromCam + POSgolbalInMind);
            if (searchRes.second < MATCH_DETECT_TOLLERANCE) {
                if (Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] != currentTargetID) {
                    Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] = currentTargetID;
                    firstDeteced = true;
                } else {
                    firstDeteced = false;
                }
                BoardPos = poseFromCam;
                simpleTemp.possiblePose = searchRes.first;
                simpleTemp.possibleX = (poseFromCam + POSgolbalInMind).x;
                simpleTemp.detectedBoard = true;
            }
        }
        else if (foundCircle) {
            auto searchRes = simpleTargets[currentTargetID].checkForClosestPose(
                    poseFromCam + POSgolbalInMind);
            if (searchRes.second < MATCH_DETECT_TOLLERANCE) {
                if (Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] != currentTargetID) {
                    Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] = currentTargetID;
                    firstDeteced = true;
                } else {
                    firstDeteced = false;
                }
                CirclePos = poseFromCam;
                simpleTemp.possiblePose = searchRes.first;
                simpleTemp.possibleX = (poseFromCam + POSgolbalInMind).x;
                simpleTemp.detectedCircle = true;
            }
        }
    }
    /**
     * Look for Circle
     */
    else if (currentCamMode == ZEDCIRCLE) {
        if (foundCircle) {
            auto searchRes = simpleTargets[currentTargetID].checkForClosestPose(
                    poseFromCam + POSgolbalInMind);
            if (searchRes.second < MATCH_DETECT_TOLLERANCE) {
                if (Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] != currentTargetID) {
                    Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] = currentTargetID;
                    firstDeteced = true;
                } else {
                    firstDeteced = false;
                }
                CirclePos = poseFromCam;
                simpleTemp.possiblePose = searchRes.first;
                simpleTemp.possibleX = (poseFromCam + POSgolbalInMind).x;
                simpleTemp.detectedCircle = true;
            }
        }
    }
    /**
     * Look for QR
     */
    else if (currentCamMode == ZEDQR) {
        //TODO
        if (foundTree) {
            float dist = (currentQRTarget.globalPos - (POSgolbalInMind + poseFromCam)).distXY();
            if (dist < MATCH_DETECT_TOLLERANCE) {
                Signals[SIGNAL_SLOT_FOUND_CURRENT_TARGET] = currentTargetID;
                QRPos = poseFromCam;
            }
        }
        if (foundQR) {
            float dist = (currentQRTarget.globalPos - (POSgolbalInMind + poseFromCam)).distXY();
            if (dist < MATCH_DETECT_TOLLERANCE) {
                Signals[SIGNAL_SLOT_QR_CODE_FOUND] = currentTargetID;
                QRPos = poseFromCam;
            }
        }
    }
    /**
     * Check the conflict Detection
     */
    if (firstDeteced) {
        currentSimpleTarget.possiblePose = simpleTemp.possiblePose;
        currentSimpleTarget.possibleX = simpleTemp.possibleX;
        currentSimpleTarget.detectedCircle = simpleTemp.detectedCircle;
        currentSimpleTarget.detectedBoard = simpleTemp.detectedBoard;
    } else {
        if (currentSimpleTarget.possiblePose != simpleTemp.possiblePose) {
            ROS_FATAL_STREAM("[CORE] Detected one target at two different Poses!\n"
                             "old pos:" << currentSimpleTarget.possiblePose
                                        << "\t new pos:" << simpleTemp.possiblePose);
        }
        if (currentSimpleTarget.detectedCircle != simpleTemp.detectedCircle) {
            ROS_FATAL_STREAM("[CORE] Detected one target Both Circle & Board!\n"
                             "old pos:" << currentSimpleTarget.possiblePose
                                        << "\t new pos:" << simpleTemp.possiblePose);
        }
    }
    if (DYNAMIC_DETECT_ENABLE) {
        //TODO add the detected things from defferent target
    }

}

void getParas(ros::NodeHandle &n) {
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
    n.getParam("/comp2018_core/X_RESTRICT_LEFT", X_RESTRICT_LEFT);
    n.getParam("/comp2018_core/X_RESTRICT_RIGHT", X_RESTRICT_RIGHT);
    n.getParam("/comp2018_core/DETECT_OFFSET", DETECT_OFFSET);

    n.getParam("/comp2018_core/CIRCLE_SEARCH_OFFSET_Y", CIRCLE_SEARCH_OFFSET_Y);
    n.getParam("/comp2018_core/CIRCLE_PASS_DIST", CIRCLE_PASS_DIST);
    n.getParam("/comp2018_core/LOW_CIRCLE_BOARD_HEIGHT", LOW_CIRCLE_BOARD_HEIGHT);
    n.getParam("/comp2018_core/HIGH_CIRCLE_BOARD_HEIGHT", HIGH_CIRCLE_BOARD_HEIGHT);
    n.getParam("/comp2018_core/FIRST_RANGE_BOARD_AIM", FIRST_RANGE_BOARD_AIM);
    n.getParam("/comp2018_core/CIRCLE_BOARD_AIM_TOLLERANCE", CIRCLE_BOARD_AIM_TOLLERANCE);
    n.getParam("/comp2018_core/AIM_BOARD_P", AIM_BOARD_P);
    n.getParam("/comp2018_core/AIM_BOARD_SATISFIED_COUNT", AIM_BOARD_SATISFIED_COUNT);
    n.getParam("/comp2018_core/CIRCLE_CLIMB_SLEEP_TIME", CIRCLE_CLIMB_SLEEP_TIME);

    n.getParam("/comp2018_core/BOARD_AIM_HEIGHT", BOARD_AIM_HEIGHT);
    n.getParam("/comp2018_core/BOARD_AIM_TOLLERANCE", BOARD_AIM_TOLLERANCE);

    n.getParam("/comp2018_core/QR_SEARCH_OFFSET", QR_SEARCH_OFFSET);
    n.getParam("/comp2018_core/QR_TOLLANCE", QR_TOLLANCE);
    n.getParam("/comp2018_core/QR_CRUISE_HEIGHT", QR_CRUISE_HEIGHT);

    n.getParam("/comp2018_core/DYNAMIC_DETECT_ENABLE", DYNAMIC_DETECT_ENABLE);
    n.getParam("/comp2018_core/DYNAMIC_DETECT_TOLLERANCE", DYNAMIC_DETECT_TOLLERANCE);
    n.getParam("/comp2018_core/MATCH_DETECT_TOLLERANCE", MATCH_DETECT_TOLLERANCE);
}

void OutputInfoAtRate(int rate) {
    ROS_INFO_STREAM_DELAYED_THROTTLE(rate, "ZED Pos:"
            << currentPX4Pos.toString());

    ROS_INFO_STREAM_DELAYED_THROTTLE(rate, "Pos In Mind:"
            << POSgolbalInMind.toString());
}

void InitPlaces() {
    SimpleTarget target(0, 0.0, true);
    target.detectedBoard = true;
    simpleTargets.push_back(target);
    target = SimpleTarget(1, 2.0, true);
    simpleTargets.push_back(target);
    target = SimpleTarget(2, 4.0, true);
    simpleTargets.push_back(target);
    target = SimpleTarget(3, 6.0, false);
    simpleTargets.push_back(target);
    target = SimpleTarget(4, 8.0, false);
    simpleTargets.push_back(target);
    target = SimpleTarget(5, 10.0, false);
    simpleTargets.push_back(target);
    target = SimpleTarget(6, 12.0, false);
    simpleTargets.push_back(target);
    target = SimpleTarget(7, 14.0, false);
    simpleTargets.push_back(target);
    target = SimpleTarget(8, 16.0, false);
    simpleTargets.push_back(target);
    target = SimpleTarget(9, 18.0, true);
    simpleTargets.push_back(target);

    for (auto & simpleTarget : simpleTargets) {
        simpleTarget.possibleX = X_RESTRICT_LEFT;
        simpleTarget.possiblePose = 2;
        simpleTarget.setOffsets({-6, 0, 0},
                                {-3, 0, 0},
                                {0, 0, 0},
                                {3, 0, 0},
                                {6, 0, 0});
    }

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
            QRtarget.globalPos.x += QR_GLOBOL_OFFSET_X;
            QRtarget.globalPos.y += QR_GLOBOL_OFFSET_Y;
            std::cout << QRtarget.globalPos.x << "\t: " << QRtarget.globalPos.y << "\n";
        }
    }
}













