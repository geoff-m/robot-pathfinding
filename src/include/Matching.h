//
// Created by student on 3/13/18.
//

#ifndef PATHDRIVER_MATCHING_H
#define PATHDRIVER_MATCHING_H

#include <map>
#include <Alternative.h>

class Matching {
private:
    int id;
    int cardinality;
    int cost;
    bool everSetCardinality = false;

    std::map<int, Point3D> dict;

public:
    Matching()
    {
        id = -1;
        cardinality = 0;
        cost = 0;
    }

    explicit Matching(int id)
    {
        this->id = id;
        cardinality = 0;
        cost = 0;
    }

    void add(int robotID, Point3D placeToGo);

    void clear();

    int getCardinality() const;

    void setCardinality(int n);

    void setCost(int cost);

    int getCost() const;

	Point3D getPlaceFor(int robotID); // new method, definition not written yet.
	// returns dict[robotId];

	bool hasPlaceFor(int robotID);	// new method, definition not written yet.
	// returns whether dict.contains(robotId);
};


#endif //PATHDRIVER_MATCHING_H
