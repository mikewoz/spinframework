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
    if (argc != 2)
    {
        std::cerr << "Usage: shared_video <path>\n";
        return 1;
    }

	spinClientContext spinListener;
	spinApp &spin = spinApp::Instance();

	if (!spinListener.start())
	{
        std::cout << "ERROR: could not start SPIN client thread" << std::endl;
        exit(EXIT_FAILURE);
	}

	spin.SceneMessage("sss",
			"createStateSet",
			"vid",
			"SharedVideoTexture",
			LO_ARGS_END);
	
    spin.SceneMessage("sss",
			"createNode",
			"box",
			"ShapeNode",
			LO_ARGS_END);

	spin.NodeMessage("box", "si",
			"setShape",
			(int)ShapeNode::BOX,
			LO_ARGS_END);

	spin.NodeMessage("vid", "si",
			"setPath",
			argv[1],
			LO_ARGS_END);

	spin.NodeMessage("box", "si",
			"setStateSet",
			"vid",
			LO_ARGS_END);

	std::cout << "\nRunning example. Press CTRL-C to quit..." << std::endl;

    while (spinListener.isRunning()) // send signal (eg, ctrl-c to stop)
        usleep(1000);
    return 0;
}

