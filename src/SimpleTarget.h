//
// Created by stumbo on 18-8-20.
//

#ifndef COMP_STRATEGY_SIMPLETARGET_H
#define COMP_STRATEGY_SIMPLETARGET_H

#include <vector>
#include <utility>
#include "tools.h"
#include <algorithm>

using std::vector;

struct SimpleTarget {
    int ID;
    float globalY;
    vector<vec3f_t> poses;    //CHANGE TO POSES
    bool onlyBoard;
    bool detectedBoard = false;
    bool detectedCircle = false;
    float possibleX = -6.5f;
    size_t possiblePose = 0;

    std::pair<size_t, float> checkForClosestPose(vec3f_t detected) {
        float minDist = 10.0f;
        size_t minNo;
        for (size_t i = 0; i < poses.size(); i ++) {
            vec3f_t toDetect(poses[i].x, poses[i].y, 0);
            float dist = (toDetect - detected).distXY();
            if (dist < minDist) {
                minDist = dist;
                minNo = i;
            }
        }
        return std::make_pair(minNo, minDist);
    }

    SimpleTarget(int ID, float globalY, bool onlyBoard)
            : ID(ID), globalY(globalY), onlyBoard(onlyBoard)
    {}

    void setOffsets(vec3f_t x1, vec3f_t x2, vec3f_t x3, vec3f_t x4, vec3f_t x5) {
        poses.clear();
        poses.push_back(x1);
        poses.push_back(x2);
        poses.push_back(x3);
        poses.push_back(x4);
        poses.push_back(x5);
        std::sort(poses.begin(), poses.end(), vec3f_s::isXBigger);
        for (auto & pose: poses) {
            pose.y += globalY;
        }
    }

    void addPossibleOffsets(vec3f_t offset) {
        poses.push_back(offset);
    }
};

struct QRTarget {
    vec3f_t globalPos;
    bool disappear = false;

    QRTarget(float globalX, float globalY) :
            globalPos(globalX, globalY, 1)
    {}
};


#endif //COMP_STRATEGY_SIMPLETARGET_H
