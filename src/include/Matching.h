//
// Created by student on 3/13/18.
//

#ifndef PATHDRIVER_MATCHING_H
#define PATHDRIVER_MATCHING_H

#include <map>
//#include <Point3D.h>
#include <Alternative.h>


// old way of storing match results:
// myMatching[myID] = Point3D(matcher.getResult(0).X, matcher.getResult(0).Y, 0);

class Matching {
private:
    int id;
    int cardinality;
    int cost;

    std::map<int, Alternative> dict;

public:
    explicit Matching(int id)
    {
        this->id = id;
        cardinality = 0;
        cost = 0;
    }

    void add(int robotID, Point3D placeToGo);

    int getCardinality() const;

    void setCost(int cost);

    int getCost() const;
};


#endif //PATHDRIVER_MATCHING_H
