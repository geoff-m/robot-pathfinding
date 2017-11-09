//
// Created by Geoff M. on 11/8/17.
//

#ifndef PATHDRIVER_POINTF3D_H
#define PATHDRIVER_POINTF3D_H


struct PointF3D {
    float x, y, z;
    PointF3D(float x, float y, float z);

    PointF3D operator+(const PointF3D other) const;
    PointF3D operator-(const PointF3D other) const;

    float euclideanNorm();

};


#endif //PATHDRIVER_POINTF3D_H
