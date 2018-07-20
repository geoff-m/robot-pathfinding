//
// Created by Geoff M. on 11/8/17.
//

#ifndef PATHDRIVER_POINT3D_H
#define PATHDRIVER_POINT3D_H


struct Point3D {
    int x, y, z;
    Point3D(int x, int y, int z);

    Point3D operator+(const Point3D other) const;
    Point3D operator-(const Point3D other) const;

    int manhattanNorm();

};


#endif //PATHDRIVER_POINT3D_H
