// TODO: add prefix to those - again?
#include <spinUtil.h>
#include <spinApp.h>
#include <spinClientContext.h>
#include <spinServerContext.h>
#include <ShapeNode.h>

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <tr1/memory>

/**
 * This example shows how to send commands to a spin server. The result is an
 * orbiting sphere.
 */

bool check_client_server()
{
    using namespace spin;
    spinApp &spin = spinApp::Instance();

    std::tr1::shared_ptr<spinServerContext> server(new spinServerContext());
    server->start();
    if (! server->isRunning())
    {
        std::cout << "Failed: Could not start a SPIN server." << std::endl;
        return false;
    }

    //spinClientContext spinListener;
    //if (! spinListener.start())
    // {
    //    std::cout << "Failed: could not start SPIN client thread" << std::endl;
    //    return false;
    // }
    spin.SceneMessage("sss", "createNode", "shp", "ShapeNode", LO_ARGS_END);
    spin.NodeMessage("shp", "si", "setShape", (int)ShapeNode::SPHERE, LO_ARGS_END);

    float timeLeft = 5.0f; // seconds
    float orbitRadius = 2.0f;
    double orbitDuration = 2.0;
    int numSamples = 100;

    std::cout << "Info: Running for " << timeLeft << " seconds." << std::endl;
    std::cout << "Info: (CTRL-C to quit)" << std::endl;

    try
    {
        while (true) //spinListener.isRunning()) // send signal (eg, ctrl-c to stop)
        {

            if (! server->isRunning())
            {
                std::cout << "Failed: Our spin server is not running anymore!" << std::endl;
                return false;
            }
            for (int i = 0; i < numSamples; ++i)
            {
                float angle = i * 2.0f * osg::PI / ((float)numSamples - 1.0f);
                spin.NodeMessage("shp", "sfff", "setTranslation",
                    sinf(angle)*orbitRadius, cosf(angle) * orbitRadius, 0.0, LO_ARGS_END);
                long sleepDuration = 1000000 * orbitDuration / numSamples;
                usleep(sleepDuration);
                timeLeft -= sleepDuration * 0.000001f;
            }
            if (timeLeft <= 0.0f)
                break;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed: Got exception " << e.what() << std::endl;
        return false;
    }
    server->stop();
    usleep(100);
    std::cout << "Info: Exited normally." << std::endl;
    return true;
}

int main(int /* argc */, char ** /* argv */)
{
    if (check_client_server())
        return 0;
    else
        return 1;
}

