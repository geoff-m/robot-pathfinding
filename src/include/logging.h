//
// Created by Geoff M. on 4/21/18.
//

#ifndef PATHDRIVER_LOGGING_H
#define PATHDRIVER_LOGGING_H

#include "Point3D.h"


#include "/usr/include/c++/5/string"
#include "/usr/include/c++/5/chrono"

class Log
{
private:
    std::string path;
    FILE** files;
    int fileCount;

    static char* getDateTime();

    static bool exists(std::string path);

public:
    Log(const std::string path, int robotCount)
    {
        fileCount = robotCount;
        this->path = path;
        if (*(path.end()) != '/')
        {
            this->path.append("/");
        }


        // Set up files
        files = (FILE**)malloc(sizeof(FILE*) * robotCount);
        for (int i=0; i<robotCount; ++i)
        {
            std::string filePath = this->path;
            filePath.append(getDateTime());
            filePath.append("_r");
            filePath.append(std::to_string(i));
            filePath.append(".log");

            const char* cpath = filePath.c_str();
            if (exists(cpath)) // Ensure file does not already exist.
            {
                printf("File \"%s\" already exists!\n", cpath);
                __glibcxx_assert(false);
            }

            FILE* f = fopen(cpath, "w");
            if (f == nullptr)
            {
                printf("Failed to open the file for writing: %s!\n", cpath);
            }

            fprintf(f, "ID\tStart\t\tGoal\t\tCurrent\t\tIPL\tCPL\tTime\tSent\tRecvd\tTimes\tRCW\n");

            files[i] = f;
        }
    }

    void writeLog(int id,
                  Point3D startLocation,
                  Point3D goalLocation,
                  Point3D currentLocation,
                  int initialPathLength,
                  int totalDistanceTravelled,
                  double timeElapsed,
                  long messagesSent,
                  long messagesReceived,
                  int timesCoordinated,
                  long robotsCoordinatedWith); // ever

    void closeLog(int id);

    ~Log();
};










#endif //PATHDRIVER_LOGGING_H
