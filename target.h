/*
 * File name: target.h
 * Date:      2016/12/09 07:55
 * Author:    Jan Faigl
 */

#ifndef __TARGET_H__
#define __TARGET_H__

#include "coords.h"
#include <memory>

struct TargetSet {
    int inCoordIndex;
    const Coords coords[2];
    double cost;

    TargetSet(const int i, const Coords pts[]) : inCoordIndex(i), coords{pts[0], pts[1]},
                                                 cost(pts[0].distance(pts[1])) {}

    inline Coords outCoord() {
        return coords[(inCoordIndex + 1) % 2];
    }

    inline Coords inCoord() {
        return coords[(inCoordIndex)];
    }

    inline void reverse() {
        inCoordIndex = (inCoordIndex + 1) % 2;
    }

    inline double squared_distance_to(TargetSet destination) {
        return this->outCoord().squared_distance(destination.inCoord());
    }

    inline double distance_to(TargetSet destination) {
        return this->outCoord().distance(destination.inCoord());
    }

    virtual std::shared_ptr <TargetSet> clone() const {
        return std::make_shared<TargetSet>(*this);
    }

};

typedef std::vector <std::shared_ptr<TargetSet>> TargetSetPtrVector;
typedef std::vector <std::vector<std::shared_ptr < TargetSet>>> TargetSetPtrVectorVector;

#endif

/* end of target.h */
