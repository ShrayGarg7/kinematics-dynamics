// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <ColorDebug.h>

#include "GravityCompensation.hpp"

/**
 * @ingroup kinematics-dynamics-programs
 *
 * \defgroup gravityCompensation
 *
 * @brief Creates an instance of roboticslab::GravityCompensation.
 */

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("gravityCompensation");
    rf.setDefaultConfigFile("gravityCompensation.ini");
    rf.configure(argc, argv);

    roboticslab::GravityCompensation mod;

    yarp::os::Network yarp;

    CD_INFO("gravityCompensation checking for yarp network... ");
    std::fflush(stdout);

    if (!yarp::os::Network::checkNetwork())
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("gravityCompensation found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else CD_SUCCESS_NO_HEADER("[ok]\n");

    return mod.runModule(rf);
}
