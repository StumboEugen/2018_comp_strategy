//
// Created by stumbo on 18-8-20.
//

#ifndef COMP_STRATEGY_SIMPLETARGET_H
#define COMP_STRATEGY_SIMPLETARGET_H

#include <vector>

using std::vector;

struct SimpleTarget {
    int ID;
    float relativeY;
    float globalY;
    vector<float> offSetX;
    vector<float> offSetY;
    bool onlyBoard;
    bool detectedBoard;
    bool detectedCircle;
    float possibleX;

    SimpleTarget(int ID, float relativeY, float globalY, bool onlyBoard)
            : ID(ID), relativeY(relativeY),
              globalY(globalY), onlyBoard(onlyBoard)
    {}

    void addPossibleOffsets(float x, float y) {
        offSetX.push_back(x);
        offSetY.push_back(y);
    }
};

struct QRTarget {
    float globalX;
    float globalY;
    bool hasTarget;
    bool sureNoTarget;

    QRTarget(float globalX, float globalY) :
            globalX(globalX), globalY(globalY)
    {}
};


#endif //COMP_STRATEGY_SIMPLETARGET_H
