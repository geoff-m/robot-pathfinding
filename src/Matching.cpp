//
// Created by Geoff M. on 3/13/18.
//

#include "Matching.h"

void Matching::add(int robotID, Point3D placeToGo)
{
    __glibcxx_assert(!everSetCardinality); // Add method and cardinality setter shouldn't be used on same instance.

    dict.emplace(std::make_pair(robotID, placeToGo));
    cardinality += 1; // Increment cardinality since we just added one point.
}

void Matching::clear()
{
    dict.clear();
}

void Matching::setCardinality(int n)
{
    cardinality = n;

    everSetCardinality = true;
}

int Matching::getCardinality() const
{
    if (everSetCardinality)
    {
        return cardinality;
    } else {
        return (int)dict.size();
    }
}

void Matching::setCost(int cost)
{
    this->cost = cost;
}

int Matching::getCost() const
{
    return cost;
}

Point3D Matching::getPlaceFor(int robotID)
{
    return dict.at(robotID);
}

bool Matching::hasPlaceFor(int robotID)
{
    return dict.find(robotID) != dict.end();
}