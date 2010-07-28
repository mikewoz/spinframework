#include <spinFramework/spinUtil.h>
#include <spinFramework/spinApp.h>
#include <spinFramework/spinClientContext.h>
#include <spinFramework/ShapeNode.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

/**
 * This example shows how to send commands to a spin server. 
 */

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}




int main(int argc, char **argv)
{
    if (argc < 4)
    {
        std::cerr << "Usage: shared_video <server_name> <shared_video1_path>...<shared_video4_path>\n";
        return 1;
    }

    spinClientContext spinListener;
    spinApp &spin = spinApp::Instance();
    spin.setSceneID(argv[1]);

    if (!spinListener.start())
    {
        std::cout << "ERROR: could not start SPIN client thread" << std::endl;
        exit(EXIT_FAILURE);
    }

    // make box
    spin.SceneMessage("sss",
            "createNode",
            "box",
            "ShapeNode",
            LO_ARGS_END);

    spin.NodeMessage("box", "si",
            "setShape",
            (int)ShapeNode::BOX,
            LO_ARGS_END);

    // make state sets
    spin.SceneMessage("sss",
            "createStateSet",
            "vid1",
            "SharedVideoTexture",
            LO_ARGS_END);

    spin.SceneMessage("sss",
            "createStateSet",
            "vid2",
            "SharedVideoTexture",
            LO_ARGS_END);

    if (argc > 4)
    {
        spin.SceneMessage("sss",
                "createStateSet",
                "vid3",
                "SharedVideoTexture",
                LO_ARGS_END);
    }
    if (argc > 5)
    {
        spin.SceneMessage("sss",
                "createStateSet",
                "vid4",
                "SharedVideoTexture",
                LO_ARGS_END);
    }

    // set textureID on stateset
    spin.NodeMessage("vid1", "ss",
            "setTextureID",
            argv[2],
            LO_ARGS_END);

    spin.NodeMessage("vid2", "ss",
            "setTextureID",
            argv[3],
            LO_ARGS_END);

    if (argc > 4)
    {
        spin.NodeMessage("vid3", "ss",
                "setTextureID",
                argv[4],
                LO_ARGS_END);

    }
    if (argc > 5)
    {
        spin.NodeMessage("vid4", "ss",
                "setTextureID",
                argv[5],
                LO_ARGS_END);

    }

    // apply a stateset to the box
    spin.NodeMessage("box", "ss",
            "setStateSet",
            "vid1",
            LO_ARGS_END);

    usleep(1000);

    std::cout << "\nRunning example." << std::endl;
    std::cout << "  Press 1 or 2 to switch textures." << std::endl;
    std::cout << "  Press q or CTRL-C to quit.\n" << std::endl;

    while (spinListener.isRunning())
    {
        if (kbhit())
        {
            switch (getchar()) {
                case '1':
                    std::cout << "texture1" << std::endl;
                    spin.NodeMessage("box", "ss",
                            "setStateSet",
                            "vid1",
                            LO_ARGS_END);
                    break;
                case '2':
                    std::cout << "texture2" << std::endl;
                    spin.NodeMessage("box", "ss",
                            "setStateSet",
                            "vid2",
                            LO_ARGS_END);
                    break;
                case '3':
                    if (argc > 4)
                    {
                        std::cout << "texture3" << std::endl;
                        spin.NodeMessage("box", "ss",
                                "setStateSet",
                                "vid3",
                                LO_ARGS_END);
                    }
                    break;
                case '4':
                    if (argc > 5)
                    {
                        std::cout << "texture4" << std::endl;
                        spin.NodeMessage("box", "ss",
                                "setStateSet",
                                "vid4",
                                LO_ARGS_END);
                    }
                    break;
                case 'q':
                case 'Q':
                    spinListener.stop();
                    break;
                default:
                    break;
            }
        }
        usleep(1000);
    }
    return 0;
}

