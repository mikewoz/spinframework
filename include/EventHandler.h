#ifndef __EventHandler_H
#define __EventHandler_H

#include <vector>
#include <string>
#include <lo/lo.h>
#include "EventTypes.h"

namespace spin
{


class ReferencedNode;

/**
 * EventHandler can be inherited in order to be receive notifications whenever
 * the spin context receives a message.
 */
class EventHandler
{
    public:

    EventHandler(void);
    ~EventHandler(void);

    /**
     * An event that occurs whenever the server list changes (spin clients only)
     */
    virtual void onServerChange(std::vector<InfoMessage*> serverList) {}

    /**
     * Event handler for scene messages (eg, createNode, deleteNode, etc).
     */
    virtual void onSceneMessage(const char * /*types*/, lo_arg ** /*argv*/, int /*argc*/) {}

    /**
     * Event handler for node messages
     */
    virtual void onNodeMessage(ReferencedNode * /*node*/, const char * /*types*/, lo_arg ** /*argv*/, int /*argc*/) {}

    /**
     * Event handler for info messages.
     *
     * Note: this is sent repeatedly at the ping rate of the server (but only in
     * the case where this spin context is running in client mode).
     */
    virtual void onInfoMessage(spin::InfoMessage * /*msg*/) {}

};

} // end namespace spin

#endif
