// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __GRAVITY_COMPENSATION_HPP__
#define __GRAVITY_COMPENSATION_HPP__

#include <vector>

#include <yarp/os/RFModule.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>

#include <ICartesianControl.h>

#define DEFAULT_PERIOD 0.05 // [s]

#define DEFAULT_LOCAL_PORT "/gcmp"
#define DEFAULT_REMOTE_SENSOR "/jr3/ch3:o"
#define DEFAULT_REMOTE_CARTESIAN "/CartesianServer"

#define DEFAULT_GAIN 0.0

namespace roboticslab
{

/**
 * @ingroup gravityCompensation
 *
 * @brief ...
 */
class GravityCompensation : public yarp::os::RFModule
{
public:
    GravityCompensation();
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool updateModule();
    virtual bool interruptModule();
    virtual double getPeriod();

private:
    void transformData(const yarp::sig::Vector & v, std::vector<double> & x);

    yarp::dev::PolyDriver sensorDevice;
    yarp::dev::IAnalogSensor * iAnalogSensor;

    yarp::dev::PolyDriver cartesianDevice;
    ICartesianControl * iCartesianControl;

    double gain;
};

} // namespace roboticslab

#endif // __GRAVITY_COMPENSATION_HPP__
