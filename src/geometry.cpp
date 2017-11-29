// Static helper functions for geometry.
// Created by Geoff M. on 11/15/17.

#include "geometry.h"

double radiansToDegrees(double radians)
{
    double bearing = (radians - 1.5708) / 3.14159265 * 180.0;

    if (bearing < 0.0){
        bearing = 360 + bearing;
    }

    return (bearing-180) * -1.0;
}


double calcDegreeDifference(double currentBearingDegrees, double destinationBearingDegrees)
{
    double diff = destinationBearingDegrees - currentBearingDegrees;

    if (diff>180.0){
        diff=diff-360.0;
    }

    if(diff<-180.0){
        diff=diff+360.0;
    }

    return diff;
}