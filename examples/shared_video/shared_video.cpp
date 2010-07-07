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
    if (argc < 2)
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
            "vid1",
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

    spin.NodeMessage("vid1", "ss",
            "setTextureID",
            argv[1],
            LO_ARGS_END);

    spin.NodeMessage("box", "ss",
            "setStateSet",
            "vid1",
            LO_ARGS_END);

    if (argc > 2)
    {
        spin.SceneMessage("sss",
                "createStateSet",
                "vid2",
                "SharedVideoTexture",
                LO_ARGS_END);

        spin.NodeMessage("vid2", "ss",
                "setTextureID",
                argv[2],
                LO_ARGS_END);

        bool done = false;

        std::cout << "\nRunning example. Press CTRL-C to quit or q to quit." << std::endl;
        while (!done and spinListener.isRunning())
        {
            char c;
            std::cin >> c;
            switch (c) {
                case '1':
                    spin.NodeMessage("box", "ss",
                            "setStateSet",
                            "vid1",
                            LO_ARGS_END);
                    break;
                case '2':
                    spin.NodeMessage("box", "ss",
                            "setStateSet",
                            "vid2",
                            LO_ARGS_END);
                    break;
                case 'q':
                case 'Q':
                    done = true;
                    break;
                default:
                    break;
            }
        }
    }

    return 0;
}

