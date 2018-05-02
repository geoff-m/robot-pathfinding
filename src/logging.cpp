//
// Created by Geoff M. on 4/21/18.
//

#include <sstream>
#include "logging.h"

void Log::writeLog(   int id,
                      Point3D startLocation,
                      Point3D goalLocation,
                      Point3D currentLocation,
                      int initialPathLength,
                      int totalDistanceTravelled,
                      double timeElapsed,
                      long messagesSent,
                      long messagesReceived,
                      int timesCoordinated,
                      long robotsCoordinatedWith)
{
    FILE* f = files[id];
    std::stringstream line;
    line << id;
    line << "\t";
    line << startLocation;
    line << "\t";
    line << goalLocation;
    line << "\t";
    line << currentLocation;
    line << "\t";
    line << initialPathLength;
    line << "\t";
    line << totalDistanceTravelled;
    line << "\t";
    line << timeElapsed;
    line << "\t";
    line << messagesSent;
    line << "\t";
    line << messagesReceived;
    line << "\t";
    line << timesCoordinated;
    line << "\t";
    line << robotsCoordinatedWith;
    line << "\n";

    fputs(line.str().c_str(), f);

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
    {
        //fclose(f); // this crashes for an unknown reason.
        f = nullptr;
    }
}

Log::~Log()
{
    for (int i=0; i<fileCount; ++i)
    {
       closeLog(i);
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