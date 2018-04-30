//
// Created by Geoff M. on 11/8/17.
//
#pragma once
#ifndef PATHDRIVER_MAIN_H
#define PATHDRIVER_MAIN_H

#include <condition_variable>
#include <mutex>
#include "Countdown.h"

#define ROBOT_COUNT 2
#define ROW_COLUMN_COUNT 10
#define MINIMUM_SAFE_DISTANCE 2 // measured in cells.

static std::mutex consoleMutex;

#endif //PATHDRIVER_MAIN_H