#ifndef __EventTypes_H
#define __EventTypes_H

#include <osg/Timer>

namespace spin
{

class InfoMessage
{
public:
    InfoMessage(const std::string &s) {
        sceneID = s;
        lastUpdate = osg::Timer::instance()->tick();
    }

    std::string sceneID;
    std::string serverAddr;
    int serverUDPPort;
    int serverTCPport;
    std::string multicastAddr;
    int multicastDataPort;
    int multicastSyncPort;
    osg::Timer_t lastUpdate;

};


/*
typedef struct
{
    std::string sceneID;
    std::string serverAddr;
    int serverUDPPort;
    int serverTCPport;
    std::string multicastAddr;
    int multicastDataPort;
    int multicastSyncPort;
    osg::Timer_t lastUpdate;
} InfoMessage;
*/
}

#endif
