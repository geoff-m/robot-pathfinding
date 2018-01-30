//
// Created by student on 1/29/18.
//

#ifndef PATHDRIVER_ALTERNATIVE_H
#define PATHDRIVER_ALTERNATIVE_H
#include "Point3D.h"

typedef struct Alternative
{
    Point3D NextStep;
    int TotalCost;
public:
    Alternative(Point3D nextStep, int totalCost)
    {
        NextStep = nextStep;
        TotalCost = totalCost;
    }

    Alternative()
    {
        NextStep = Point3D();
        TotalCost = 0;
    }

} Alternative;



#endif //PATHDRIVER_ALTERNATIVE_H
