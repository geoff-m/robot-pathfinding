//
// Created by Geoff M. on 11/8/17.
//

#include "main.h"
#include "Grid.h"
#include "VrepPioneerDriver.h"

int main(int argc, char *argv[]) {
    const PointF3D gridOrigin(-5.0f, -5.0f, 0.0f);

    Grid4C grid(gridOrigin, 5, 5, 1, 1.0f, 1.0f, 0.0f);

    VrepPioneerDriver driver("PioneerP3x");

    driver.stop();

    driver.driveTo(grid.getWorldPoint(Point3D(0, 0, 0)));
    driver.driveTo(grid.getWorldPoint(Point3D(4, 0, 0)));
}