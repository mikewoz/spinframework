#include <spinFramework/spinUtil.h>
#include <spinFramework/spinApp.h>
#include <spinFramework/spinClientContext.h>
#include <spinFramework/ShapeNode.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <time.h>
#include <sys/utsname.h>

//#include <3DconnexionClient/ConnexionClient.h>
#include <3DConnexionClient/ConnexionClientAPI.h>

/**
 * This example uses the 3Dconnexion SpaceNavigator to control the viewer
 * position
 */

UInt16 fConnexionClientID;

pthread_mutex_t spacenavMutex;

// scalars when using setVelocity and setSpin:
#define VELOCITY_SCALAR 0.005
#define SPIN_SCALAR 0.05


lo_address txAddr;
time_t lastTime;
float speedScaleValue;
std::string userID;

// scalars when using move and rotate:
//#define VELOCITY_SCALAR 0.0005
//#define SPIN_SCALAR 0.0005

std::string getHostname()
{
	struct utsname ugnm;
    std::string hostname;

	if (uname(&ugnm) < 0) return "";
	hostname = std::string(ugnm.nodename);
	
	// for OSX, remove .local
	size_t pos = hostname.rfind(".local");
	if (pos!=std::string::npos) hostname = hostname.substr(0,pos);
	
	return hostname;
}


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
	time_t currentTime;

	switch(messageType)
	{
		case kConnexionMsgDeviceState:
			state = (ConnexionDeviceState*)messageArgument;
			if (state->client == fConnexionClientID)
			{

				// reset speedScale if user let go of the puck:
				time(&currentTime);
				double dt = difftime( currentTime, lastTime );
				if (dt > 1.0) speedScaleValue = 1.0;
				lastTime = currentTime;



				//std::cout << "SPACENAVIGATOR INFO: " << state->axis[0] << "," << state->axis[1] << "," << state->axis[2] << " - " << state->axis[3] << "," << state->axis[4] << "," << state->axis[5] << std::endl;


				float x,y,z;
				if (0) // if using puck rotations
				{
					x = state->axis[0];
					y = state->axis[2];
					z = -state->axis[1];
				}
				else {
					// otherwise take average of rotation and push in an axis
					// direction:
					x = (state->axis[0] + (state->axis[5]/2.0));
					y = (state->axis[2] + -(state->axis[3]/2.0));
					z = -state->axis[1];
				}

				float xyMagnitude = sqrt((x*x) + (y*y));


           		// we are pushing hard in some direction, so increment the speed over time:
               	// NOTE, pulling up on the puck is harder to get to the max than
               	// pushing down, so we handle the up/down axis separately
               	if ((xyMagnitude>190) || (z<-200) ||  (z>150)) speedScaleValue += 0.02;
               	else speedScaleValue -= 0.04;

               	// max out at 10x of speed scaling:
               	if (speedScaleValue < 1) speedScaleValue = 1.0;
               	else if (speedScaleValue > 10) speedScaleValue = 10.0;

             	//std::cout << "dt=" << dt << ", translated INFO: " << x << "," << y << "," << z << ", xyMagnitudes="<<xyMagnitude <<", speedScaleValue=" << speedScaleValue << std::endl;


				float dX = pow(x * VELOCITY_SCALAR, 3) * 0.8;
				float dY = pow(y * VELOCITY_SCALAR, 3) * 0.8;
				float dZ = pow(z * VELOCITY_SCALAR, 3);


               	//std::cout << "velocity=" << dX << "," << dY << "," << dZ << " * "<< speedScaleValue << std::endl;


               	// decipher what command/event is being reported by the driver
                switch (state->command)
                {
                    case kConnexionCmdHandleAxis:
                     	//std::cout << " dir=("<<(int)state->axis[0]<<","<<(int)state->axis[1]<<","<<(int)state->axis[2]<<")";
                    	//std::cout << " rot=("<<(int)state->axis[3]<<","<<(int)state->axis[4]<<","<<(int)state->axis[5]<<")";

                    	//lo_send(txAddr, "/SPIN/default/posture??",
                    	lo_send(txAddr, std::string("/SPIN/default/"+userID).c_str(),
                    			"sfff", "setVelocity",
                    			2*dX*speedScaleValue, 2*dY*speedScaleValue, dZ*speedScaleValue,
                    			/*
                    			(float)  VELOCITY_SCALAR*state->axis[0],
                    			(float)  VELOCITY_SCALAR*state->axis[2],
                    			(float) -VELOCITY_SCALAR*state->axis[1],
                    			*/
                    			LO_ARGS_END);
                    	
                    	//lo_send(txAddr, "/SPIN/default/posture??",
                    	lo_send(txAddr, std::string("/SPIN/default/"+userID).c_str(),
                    			"sfff", "setSpin",
                    			(float)  SPIN_SCALAR*state->axis[3],
                    			(float)  SPIN_SCALAR*state->axis[5],
                    			(float) -SPIN_SCALAR*state->axis[4]*10.0, // yaw should be faster
                    			LO_ARGS_END);
                    	


                    	break;

                    case kConnexionCmdHandleButtons:
                    	//std::cout << " buttons=" << (int)state->buttons << std::endl;

                    	if ((int)state->buttons == 2)
                    	{
                    		lo_send(txAddr, "/SPIN/default/menu", "s", "highlightNext", LO_ARGS_END);
                    	}
                    	else if ((int)state->buttons == 1)
                    	{
                    		lo_send(txAddr, "/SPIN/default/menu", "s", "select", LO_ARGS_END);
                    	}

                    	//if (state->buttons != lastState.buttons) std::cout << "buttonState=" << (int)state->buttons << std::endl;




                    	/*
                    	spinApp::Instance().NodeMessage(spinApp::Instance().getUserID().c_str(),
                    			"sfff", "setTranslation", 0.0, -10.0, 5.0,
                    			LO_ARGS_END);
                    	spinApp::Instance().NodeMessage(spinApp::Instance().getUserID().c_str(),
                    			"sfff", "setOrientation", 0.0, 0.0, 0.0,
                    			LO_ARGS_END);
*/
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

int main(int argc, char **argv)
{

	std::cout << "\nRunning example. Press CTRL-C to quit..." << std::endl;

	//txAddr = lo_address_new("10.20.10.104", "54324");
	txAddr = lo_address_new("239.0.0.1", "54324");
    userID = getHostname();
    lo_send(txAddr, std::string("/SPIN/default/"+userID).c_str(),"si","setVelocityMode",1,LO_ARGS_END);

	time( &lastTime );
	speedScaleValue = 1.0;


	// Check if 3DConnexion famework/lib is installed:
	if (InstallConnexionHandlers != NULL)
	{
		// Install our handler and register our client
		OSErr error = InstallConnexionHandlers(spacenavCallback, spacenavConnected, spacenavDisconnected);

		if (error==0)
		{
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
