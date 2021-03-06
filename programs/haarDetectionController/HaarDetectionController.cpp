// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetectionController.hpp"

#include <string>

#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

using namespace roboticslab;

namespace
{
    const int INITIAL_ACT_DELAY = 3;
    const int FINAL_ACT_DELAY = 5;
}

bool HaarDetectionController::configure(yarp::os::ResourceFinder &rf)
{
    std::string localPort = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PORT),
            "local cartesian port").asString();
    std::string remoteVision = rf.check("remoteVision", yarp::os::Value(DEFAULT_REMOTE_VISION),
            "remote vision port").asString();
    std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_REMOTE_CARTESIAN),
            "remote cartesian port").asString();

    period = rf.check("period", yarp::os::Value(DEFAULT_PERIOD), "period [s]").asFloat64();

    yarp::os::Property cartesianControlClientOptions;
    cartesianControlClientOptions.put("device", "CartesianControlClient");
    cartesianControlClientOptions.put("cartesianLocal", localPort);
    cartesianControlClientOptions.put("cartesianRemote", remoteCartesian);

    if (!cartesianControlDevice.open(cartesianControlClientOptions))
    {
        CD_ERROR("Cartesian control client device not valid.\n");
        close();
        return false;
    }

    if (!cartesianControlDevice.view(iCartesianControl))
    {
        CD_ERROR("Could not view iCartesianControl.\n");
        close();
        return false;
    }

    if (!rf.check("disableSensors"))
    {
        std::string sensorsPort = rf.check("sensorsPort", yarp::os::Value(DEFAULT_PROXIMITY_SENSORS),
                "remote sensors port").asString();

        yarp::os::Property sensorsClientOptions;
        sensorsClientOptions.fromString(rf.toString());
        sensorsClientOptions.put("device", "ProximitySensorsClient");
        sensorsClientOptions.put("local", localPort);
        sensorsClientOptions.put("remote", sensorsPort);

        sensorsClientDevice.open(sensorsClientOptions);

        if (!sensorsClientDevice.isValid())
        {
            CD_ERROR("Proximity sensors device not valid.\n");
            return false;
        }

        if (!sensorsClientDevice.view(iProximitySensors))
        {
            CD_ERROR("Could not view iProximitySensors.\n");
            return false;
        }
    }

    if (!iCartesianControl->act(VOCAB_CC_ACTUATOR_OPEN_GRIPPER))
    {
        CD_ERROR("Unable to actuate tool.\n.");
        close();
        return false;
    }

    if (!rf.check("noMove"))
    {
        grabberResponder.setICartesianControlDriver(iCartesianControl);

        if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::TCP_FRAME))
        {
            CD_ERROR("Unable to set TCP reference frame.\n");
            return false;
        }

        grabberResponder.setNoApproachSetting(rf.check("noApproach"));

        grabberPort.useCallback(grabberResponder);
        grabberPort.open(localPort + "/state:i");

        if (!yarp::os::Network::connect(remoteVision + "/state:o", localPort + "/state:i"))
        {
            CD_ERROR("Unable to connect to remote vision port with prefix: %s.\n", remoteVision.c_str());
            close();
            return false;
        }
    }

    CD_INFO("Delaying %d seconds...\n", INITIAL_ACT_DELAY);
    yarp::os::Time::delay(INITIAL_ACT_DELAY);

    return true;
}

bool HaarDetectionController::updateModule()
{
    if (sensorsClientDevice.isValid() && iProximitySensors->hasTarget())
    {
        CD_INFO("Target detected.\n");

        // disable servo control, stop motors and close stream of sensor data
        grabberPort.interrupt();
        iCartesianControl->stopControl();
        sensorsClientDevice.close();

        // close gripper, delay needed for AMOR
        const double now = yarp::os::Time::now();

        while (yarp::os::Time::now() - now < FINAL_ACT_DELAY)
        {
            iCartesianControl->act(VOCAB_CC_ACTUATOR_CLOSE_GRIPPER);
            yarp::os::Time::delay(0.1);
        }

        return false;
    }

    return true;
}

bool HaarDetectionController::interruptModule()
{
    grabberPort.interrupt();

    if (iCartesianControl != NULL)
    {
        iCartesianControl->stopControl();
    }

    return true;
}

bool HaarDetectionController::close()
{
    grabberPort.close();
    return sensorsClientDevice.close() & cartesianControlDevice.close();
}

double HaarDetectionController::getPeriod()
{
    return period;
}
