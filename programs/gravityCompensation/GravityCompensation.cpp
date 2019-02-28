// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GravityCompensation.hpp"

#include <yarp/os/Property.h>

#include <ColorDebug.h>

namespace roboticslab
{

/************************************************************************/

GravityCompensation::GravityCompensation()
    : iAnalogSensor(NULL),
      iCartesianControl(NULL),
      gain(DEFAULT_GAIN)
{}

/************************************************************************/

bool GravityCompensation::configure(yarp::os::ResourceFinder &rf)
{
    CD_INFO("GravityCompensation config: %s.\n", rf.toString().c_str());

    std::string localPrefix = DEFAULT_LOCAL_PORT;

    std::string analogLocal = rf.check("local", yarp::os::Value(localPrefix + "/sensor"),
            "local analog sensor port").asString();
    std::string analogRemote = rf.check("remote", yarp::os::Value(DEFAULT_REMOTE_SENSOR),
            "remote analog sensor port").asString();

    std::string cartesianLocal = rf.check("localCartesian", yarp::os::Value(localPrefix + "/cartesian"),
            "local cartesian port").asString();
    std::string cartesianRemote = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_REMOTE_CARTESIAN),
            "remote cartesian port").asString();

    gain = rf.check("gain", yarp::os::Value(DEFAULT_GAIN), "gain").asDouble();

    yarp::os::Property sensorOptions;
    sensorOptions.put("device", "analogsensorclient");
    sensorOptions.put("local", analogLocal);
    sensorOptions.put("remote", analogRemote);

    if (!sensorDevice.open(sensorOptions))
    {
        CD_ERROR("Unable to open sensor device.\n");
        return false;
    }

    if (!sensorDevice.view(iAnalogSensor))
    {
        CD_ERROR("Cannot view IAnalogSensor interface.\n");
        return false;
    }

    CD_SUCCESS("Successfully instantiated sensor device.\n");

    yarp::os::Property cartesianOptions;
    cartesianOptions.put("device", "CartesianControlClient");
    cartesianOptions.put("local", cartesianLocal);
    cartesianOptions.put("remote", cartesianRemote);

    if (!cartesianDevice.open(cartesianOptions))
    {
        CD_ERROR("Unable to open cartesian device.\n");
        return false;
    }

    if (!cartesianDevice.view(iCartesianControl))
    {
        CD_ERROR("Cannot view ICartesianControl interface.\n");
        return false;
    }

    CD_SUCCESS("Successfully instantiated cartesian device.\n");

    // TODO: calibrate channel

    if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_FRAME, ICartesianSolver::TCP_FRAME))
    {
        CD_ERROR("Unable to set TCP frame.\n");
        return false;
    }

    if (!iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING, VOCAB_CC_MOVI))
    {
        CD_ERROR("Unable to preset movi command.\n");
        return false;
    }

    return true;
}

/************************************************************************/

double GravityCompensation::getPeriod()
{
    return DEFAULT_PERIOD;
}

/************************************************************************/

bool GravityCompensation::updateModule()
{
    yarp::sig::Vector v;

    if (iAnalogSensor->read(v) != yarp::dev::IAnalogSensor::AS_OK)
    {
        CD_WARNING("Unable to read data from sensors.\n");
        return true;
    }

    std::vector<double> x(6);
    transformData(v, x);

    iCartesianControl->movi(x);

    return true;
}

/************************************************************************/

bool GravityCompensation::interruptModule()
{
    if (!iCartesianControl->stopControl())
    {
        CD_WARNING("Unable to stop control.\n");
    }

    bool ok = true;
    ok &= cartesianDevice.close();
    ok &= sensorDevice.close();
    return ok;
}

/************************************************************************/

void GravityCompensation::transformData(const yarp::sig::Vector & v, std::vector<double> & x)
{
    for (int i = 0; i < x.size(); i++)
    {
        x[i] = v[i] * gain;
    }
}

/************************************************************************/

}  // namespace roboticslab
