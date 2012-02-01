#include <spinFramework/spinUtil.h>
#include <spinFramework/spinApp.h>
#include <spinFramework/spinClientContext.h>
#include <spinFramework/ShapeNode.h>
#include <iostream>
#include <cstdlib>
#include <cmath>


//#include <3DconnexionClient/ConnexionClient.h>
#include <3DConnexionClient/ConnexionClientAPI.h>

/**
 * This example uses the 3Dconnexion SpaceNavigator to control the viewer
 * position
 */

UInt16 fConnexionClientID;

pthread_mutex_t spacenavMutex;

// scalars when using setVelocity and setSpin:
#define VELOCITY_SCALAR 0.05
#define SPIN_SCALAR 0.01

// scalars when using move and rotate:
//#define VELOCITY_SCALAR 0.0005
//#define SPIN_SCALAR 0.0005


// Make the linker happy for the framework check (see link below for more info)
// http://developer.apple.com/documentation/MacOSX/Conceptual/BPFrameworks/Concepts/WeakLinking.html
//extern OSErr InstallConnexionHandlers() __attribute__((weak_import));
extern "C" {
extern OSErr InstallConnexionHandlers(ConnexionMessageHandlerProc messageHandler, ConnexionAddedHandlerProc addedHandler, ConnexionRemovedHandlerProc removedHandler) __attribute__((weak_import));
}

void spacenavConnected(io_connect_t connection)
{
   printf("  Using 3rd party device:\t3DConnexion SpaceNavigator\n");
}

void spacenavDisconnected(io_connect_t connection)
{
   printf("  WARNING: 3DConnexion SpaceNavigator disconnected\n");
}

void spacenavCallback(io_connect_t connection, natural_t messageType, void *messageArgument)
{
	static ConnexionDeviceState	lastState;
	ConnexionDeviceState		*state;

	switch(messageType)
	{
		case kConnexionMsgDeviceState:
			state = (ConnexionDeviceState*)messageArgument;
			if (state->client == fConnexionClientID)
			{
               	//std::cout << "SPACENAVIGATOR INFO:";

               	// decipher what command/event is being reported by the driver
                switch (state->command)
                {
                    case kConnexionCmdHandleAxis:
                     	//std::cout << " dir=("<<(int)state->axis[0]<<","<<(int)state->axis[1]<<","<<(int)state->axis[2]<<")";
                    	//std::cout << " rot=("<<(int)state->axis[3]<<","<<(int)state->axis[4]<<","<<(int)state->axis[5]<<")";

                    	spinApp::Instance().NodeMessage(spinApp::Instance().getUserID().c_str(),
                    			"sfff", "setVelocity",
                    			(float)  VELOCITY_SCALAR*state->axis[0],
                    			(float)  VELOCITY_SCALAR*state->axis[2],
                    			(float) -VELOCITY_SCALAR*state->axis[1],
                    			LO_ARGS_END);
                    	spinApp::Instance().NodeMessage(spinApp::Instance().getUserID().c_str(),
                    			"sfff", "setSpin",
                    			(float)  SPIN_SCALAR*state->axis[3],
                    			(float)  SPIN_SCALAR*state->axis[5],
                    			(float) -SPIN_SCALAR*state->axis[4]*10.0, // yaw should be faster
                    			LO_ARGS_END);

                    	break;

                    case kConnexionCmdHandleButtons:
                    	//std::cout << " buttons=" << (int)state->buttons;

                    	spinApp::Instance().NodeMessage(spinApp::Instance().getUserID().c_str(),
                    			"sfff", "setTranslation", 0.0, -10.0, 5.0,
                    			LO_ARGS_END);
                    	spinApp::Instance().NodeMessage(spinApp::Instance().getUserID().c_str(),
                    			"sfff", "setOrientation", 0.0, 0.0, 0.0,
                    			LO_ARGS_END);

                        break;
                }
               	//std::cout << std::endl;

				memcpy(state, &lastState, (long)sizeof(ConnexionDeviceState));
			}
			break;

		default:
			// other messageTypes can happen and should be ignored
			break;
	}



}


void *spacenavThread(void *arg)
{
	// Check if 3DConnexion famework/lib is installed:
	if (InstallConnexionHandlers != NULL)
	{
		// Install our handler and register our client
		OSErr error = InstallConnexionHandlers(spacenavCallback, spacenavConnected, spacenavDisconnected);

		if (error==0)
		{
			// make sure user has proper velocitymode:
        	spinApp::Instance().NodeMessage(spinApp::Instance().getUserID().c_str(),
        			"si", "setVelocityMode", (int)GroupNode::MOVE, LO_ARGS_END);

			// This takes over in our application only:
			//fConnexionClientID = RegisterConnexionClient('spinviewer', (UInt8*)"\pspinviewer with spacenavigator", kConnexionClientModeTakeOver, kConnexionMaskAll);

			// This takes over system-wide:
			fConnexionClientID = RegisterConnexionClient(kConnexionClientWildcard, 0L, kConnexionClientModeTakeOver, kConnexionMaskAll);

			// On OSX, the 3DConnextion driver uses CFRunLoop to monitor the
			// device.
			CFRunLoopRun();

		}
		else std::cout << "ERROR: could not create spacenagivator handlers" << std::endl;
	}
	else std::cout << "ERROR: could not find 3DConnexion library/framework" << std::endl;

	return NULL;
}

int main(int argc, char **argv)
{
	spinClientContext spinListener;
	spinApp &spin = spinApp::Instance();


	if (!spinListener.start())
	{
        std::cout << "ERROR: could not start SPIN client thread" << std::endl;
        exit(EXIT_FAILURE);
	}


	// start a pthread for the spacenavigator:
    pthread_t pthreadID;
    pthread_attr_t pthreadAttr;
    if (pthread_attr_init(&pthreadAttr) < 0)
    {
        std::cout << "Could not prepare spacenavigator thread" << std::endl;
        return false;
    }
    //if (pthread_create( &pthreadID, &pthreadAttr, spacenavThread, this) < 0)
    if (pthread_create( &pthreadID, &pthreadAttr, spacenavThread, &spinListener) < 0)
    {
        std::cout << "Could not create spacenavigator thread" << std::endl;
        return false;
    }

    // create a big grid to help orient ourselves
	spin.SceneMessage("sss", "createNode", "grid", "GridNode", LO_ARGS_END);
	spin.NodeMessage("grid", "sf", "setSize", 100.0, LO_ARGS_END);

	std::cout << "\nRunning example. Press CTRL-C to quit..." << std::endl;

    while (spinListener.isRunning()) // send signal (eg, ctrl-c to stop)
    {
		usleep(1000);
    }

    // Clean up 3DConnextion stuff:
    if (InstallConnexionHandlers != NULL)
    {
    	// Framework is installed...
    	printf("3DConnexion client closing\n");

    	// Unregister client, clean up all handlers
    	if (fConnexionClientID) UnregisterConnexionClient(fConnexionClientID);
    	CleanupConnexionHandlers();
    }

    return 0;
}
