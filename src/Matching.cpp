//
// Created by student on 3/13/18.
//

#include "Matching.h"

void Matching::add(int robotID, Point3D placeToGo)
{
    __glibcxx_assert(!everSetCardinality); // Add method and cardinality setter shouldn't be used on same instance.

    dict.emplace(std::make_pair(robotID, placeToGo));
}

void Matching::setCardinality(int n)
{
    cardinality = n;

    everSetCardinality = true;
}

int Matching::getCardinality() const
{
    return (int)dict.size();
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
    return dict[robotID];
}

bool Matching::hasPlaceFor(int robotID)
{
    return dict.find(robotID) != dict.end();
}