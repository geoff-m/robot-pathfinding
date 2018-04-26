//
// Created by Geoff M. on 4/21/18.
//

#include "logging.h"

void Log::writeLog(   int id,
                      Point3D startLocation,
                      Point3D goalLocation,
                      int initialPathLength,
                      int currentPathLength,
                      double timeElapsed,
                      long messagesSent,
                      long messagesReceived,
                      int timesCoordinated,
                      long robotsCoordinatedWith)
{
    FILE* f = files[id];
    if (startLocation.isUninitialized())

    {
        if (goalLocation.isUninitialized())
        {
            fprintf(f, "%d\tx,x,x\tx,x,x\t%d\t%d\t%.4f\t%lu\t%lu\t%d\t%lu\n",
                    id,
                    initialPathLength,
                    currentPathLength,
                    timeElapsed,
                    messagesSent,
                    messagesReceived,
                    timesCoordinated,
                    robotsCoordinatedWith
            );
        } else {
            fprintf(f, "%d\tx,x,x\t%d,%d,%d\t%d\t%d\t%.4f\t%lu\t%lu\t%d\t%lu\n",
                    id,
                    goalLocation.getX(),
                    goalLocation.getY(),
                    goalLocation.getZ(),
                    initialPathLength,
                    currentPathLength,
                    timeElapsed,
                    messagesSent,
                    messagesReceived,
                    timesCoordinated,
                    robotsCoordinatedWith
            );
        }
    } else {
        if (goalLocation.isUninitialized())
        {
            fprintf(f, "%d\t%d,%d,%d\tx,x,x\t%d\t%d\t%.4f\t%lu\t%lu\t%d\t%lu\n",
                    id,
                    startLocation.getX(),
                    startLocation.getY(),
                    startLocation.getZ(),
                    initialPathLength,
                    currentPathLength,
                    timeElapsed,
                    messagesSent,
                    messagesReceived,
                    timesCoordinated,
                    robotsCoordinatedWith
            );
        }
    }



    fprintf(f, "%d\t%d,%d,%d\t%d,%d,%d\t%d\t%d\t%.4f\t%lu\t%lu\t%d\t%lu\n",
             id,
            startLocation.getX(),
            startLocation.getY(),
            startLocation.getZ(),
            goalLocation.getX(),
            goalLocation.getY(),
            goalLocation.getZ(),
            initialPathLength,
            currentPathLength,
            timeElapsed,
            messagesSent,
            messagesReceived,
            timesCoordinated,
            robotsCoordinatedWith
    );

    fflush(f);
}

bool Log::exists(const std::string path)
{
    FILE* f = fopen(path.c_str(), "r");
    if (f != nullptr)
    {
        fclose(f);
        return true;
    }
    return false;
}

void Log::closeLog(int id)
{
    FILE* f = files[id];
    if (f != nullptr)
        fclose(f);
}

Log::~Log()
{
    for (int i=0; i<fileCount; ++i)
    {
        FILE* f = files[i];
        if (f != nullptr)
            fclose(f);
    }
    free(files);
}

char* Log::getDateTime()
{
    const int BUFFER_SIZE = 40;
    char* ret = (char*)malloc(BUFFER_SIZE);

    time_t rawTime;
    time(&rawTime);
    struct tm* timeInfo = localtime(&rawTime);

    int success = strftime(ret, BUFFER_SIZE, "%y-%m-%d--%H-%M-%S", timeInfo);
    return ret;
}