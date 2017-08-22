#include "StreamingSpnav.hpp"

#include <string>
#include <cmath>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <ColorDebug.hpp>

namespace roboticslab
{

bool StreamingSpnav::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("StreamingSpnav config: %s.\n", rf.toString().c_str());

    std::string localSpnav = rf.check("localSpnav", yarp::os::Value(DEFAULT_SPNAV_LOCAL), "local spnav port").asString();
    std::string remoteSpnav = rf.check("remoteSpnav", yarp::os::Value(DEFAULT_SPNAV_REMOTE), "remote spnav port").asString();

    std::string localActuator = rf.check("localActuator", yarp::os::Value(DEFAULT_ACTUATOR_LOCAL), "local actuator port").asString();
    std::string remoteActuator = rf.check("remoteActuator", yarp::os::Value(DEFAULT_ACTUATOR_REMOTE), "remote actuator port").asString();

    std::string localCartesian = rf.check("localCartesian", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL), "local cartesian port").asString();
    std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE), "remote cartesian port").asString();

    std::string sensorsPort = rf.check("sensorsPort", yarp::os::Value(DEFAULT_PROXIMITY_SENSORS), "remote sensors port").asString();

    scaling = rf.check("scaling", yarp::os::Value(DEFAULT_SCALING), "scaling factor").asDouble();

    yarp::os::Value axesValue = rf.check("fixedAxes", yarp::os::Value(DEFAULT_FIXED_AXES), "axes with restricted movement");

    fixedAxes.resize(6, false);

    if (axesValue.isList())
    {
        yarp::os::Bottle * axesList = axesValue.asList();

        for (int i = 0; i < axesList->size(); i++)
        {
            std::string str = axesList->get(i).asString();

            if (str == "x")
            {
                fixedAxes[0] = true;
            }
            else if (str == "y")
            {
                fixedAxes[1] = true;
            }
            else if (str == "z")
            {
                fixedAxes[2] = true;
            }
            else if (str == "rotx")
            {
                fixedAxes[3] = true;
            }
            else if (str == "roty")
            {
                fixedAxes[4] = true;
            }
            else if (str == "rotz")
            {
                fixedAxes[5] = true;
            }
            else
            {
                CD_WARNING("Unrecognized fixed axis label: %s. Ignoring...\n", str.c_str());
            }
        }
    }

    if (rf.check("help"))
    {
        printf("StreamingSpnav options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        return false;
    }

    yarp::os::Property spnavClientOptions;
    spnavClientOptions.put("device", "analogsensorclient");
    spnavClientOptions.put("local", localSpnav);
    spnavClientOptions.put("remote", remoteSpnav);

    spnavClientDevice.open(spnavClientOptions);

    if (!spnavClientDevice.isValid())
    {
        CD_ERROR("spnav client device not valid.\n");
        return false;
    }

    if (!spnavClientDevice.view(iAnalogSensor))
    {
        CD_ERROR("Could not view iAnalogSensor.\n");
        return false;
    }

    yarp::os::Property actuatorClientOptions;
    actuatorClientOptions.put("device", "analogsensorclient");
    actuatorClientOptions.put("local", localActuator);
    actuatorClientOptions.put("remote", remoteActuator);

    actuatorClientDevice.open(actuatorClientOptions);

    if (!actuatorClientDevice.isValid())
    {
        CD_ERROR("actuator client device not valid.\n");
        return false;
    }

    if (!actuatorClientDevice.view(iAnalogSensorAct))
    {
        CD_ERROR("Could not view iAnalogSensorAct.\n");
        return false;
    }

    yarp::os::Property cartesianControlClientOptions;
    cartesianControlClientOptions.put("device", "CartesianControlClient");
    cartesianControlClientOptions.put("cartesianLocal", localCartesian);
    cartesianControlClientOptions.put("cartesianRemote", remoteCartesian);

    cartesianControlClientDevice.open(cartesianControlClientOptions);

    if (!cartesianControlClientDevice.isValid())
    {
        CD_ERROR("cartesian control client device not valid.\n");
        return false;
    }

    if (!cartesianControlClientDevice.view(iCartesianControl))
    {
        CD_ERROR("Could not view iCartesianControl.\n");
        return false;
    }

    yarp::os::Property sensorOptions;
    sensorOptions.put("device", "ProximitySensors");

    proximitySensorsDevice.open(sensorOptions);

    if (!proximitySensorsDevice.isValid())
    {
        CD_WARNING("sensors device not valid.\n");
    }

    if (!proximitySensorsDevice.view(iProximitySensors))
    {
        CD_WARNING("Could not view iSensors.\n");
    }

    isStopped = true;
    actuatorState = 2;

    return true;
}

bool StreamingSpnav::updateModule()
{
    yarp::sig::Vector data;

    iAnalogSensorAct->read(data);

    if (data.size() == 2)
    {
        int button1 = data[0];
        int button2 = data[1];

        if (button1 == 1)
        {
            actuatorState = 0;
            iCartesianControl->act(0);
        }
        else if (button2 == 1)
        {
            actuatorState = 1;
            iCartesianControl->act(1);
        }
        else
        {
            if (actuatorState != 2)
            {
                iCartesianControl->act(2);
            }

            actuatorState = 2;
        }
    }

    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() != 6)
    {
        CD_ERROR("Invalid data size: %d.\n", data.size());
        return false;
    }

    std::vector<double> xdot(6, 0.0);
    bool isZero = true;

    if (!fixedAxes[3] || !fixedAxes[4] || !fixedAxes[5])
    {
        yarp::sig::Matrix rot = yarp::math::rpy2dcm(data.subVector(3, 5));
        yarp::sig::Vector axisAngle = yarp::math::dcm2axis(rot);
        yarp::sig::Vector axis = axisAngle.subVector(0, 2);
        double angle = axisAngle[3];

        data[3] = axis[0] * angle;
        data[4] = axis[1] * angle;
        data[5] = axis[2] * angle;
    }

    for (int i = 0; i < data.size(); i++)
    {
        if (!fixedAxes[i] && data[i] != 0.0)
        {
            isZero = false;
            xdot[i] = data[i] / scaling;
        }
    }

    if (isZero || (proximitySensorsDevice.isValid() && iProximitySensors->hasObstacle()))
    {
        if (!isStopped)
        {
            isStopped = iCartesianControl->stopControl();
        }

        return true;
    }
    else
    {
        isStopped = false;
    }

    if (!iCartesianControl->vmos(xdot))
    {
        CD_WARNING("vmos failed.\n");
    }

    return true;
}

bool StreamingSpnav::interruptModule()
{
    bool ok = true;
    ok &= iCartesianControl->stopControl();
    ok &= cartesianControlClientDevice.close();
    ok &= actuatorClientDevice.close();
    ok &= spnavClientDevice.close();
    ok &= proximitySensorsDevice.close();
    return ok;
}

double StreamingSpnav::getPeriod()
{
    return 0.02;  // [s]
}

}  // namespace roboticslab
