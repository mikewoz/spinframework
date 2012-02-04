#include <spinFramework/spinUtil.h>
#include <spinFramework/spinApp.h>
#include <spinFramework/spinClientContext.h>
#include <spinFramework/ShapeNode.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

/**
 * This example shows how to send commands to a spin server. The result is an
 * orbiting sphere.
 */

int main(int argc, char **argv)
{
    using namespace spin;
    
    spinClientContext spinListener;
	spinApp &spin = spinApp::Instance();

	if (!spinListener.start())
	{
        std::cout << "ERROR: could not start SPIN client thread" << std::endl;
        exit(EXIT_FAILURE);
	}

	spin.SceneMessage("sss",
			"createNode",
			"shp",
			"ShapeNode",
			LO_ARGS_END);

	spin.NodeMessage("shp", "si",
			"setShape",
			(int)ShapeNode::SPHERE,
			LO_ARGS_END);

	float orbitRadius = 2.0;
	double orbitDuration = 2.0;
	int numSamples = 100;

	std::cout << "\nRunning example. Press CTRL-C to quit..." << std::endl;

    while (spinListener.isRunning()) // send signal (eg, ctrl-c to stop)
    {
		for (int i = 0; i < numSamples; ++i)
		{
			float angle = i * 2.0f*osg::PI/((float)numSamples-1.0f);

			spin.NodeMessage("shp", "sfff",
					"setTranslation",
					sinf(angle)*orbitRadius,
					cosf(angle)*orbitRadius,
					0.0,
					LO_ARGS_END);

			usleep(1000000 * orbitDuration / numSamples);
		}
    }
    return 0;
}
