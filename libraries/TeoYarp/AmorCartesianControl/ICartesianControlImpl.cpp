// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorCartesianControl.hpp"

#include <algorithm>
#include <functional>

#include <kdl/frames.hpp>

#include "KdlVectorConverter.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool roboticslab::AmorCartesianControl::stat(int &state, std::vector<double> &x)
{
    AMOR_VECTOR7 positions;

    if (amor_get_cartesian_position(handle, positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    x.resize(6);

    x[0] = positions[0] * 0.001;  // [m]
    x[1] = positions[1] * 0.001;
    x[2] = positions[2] * 0.001;

    x[3] = positions[3];  // [deg]
    x[4] = positions[4];
    x[5] = positions[5];

    state = 0;  // dummy value

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    CD_ERROR("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movj(const std::vector<double> &xd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::relj(const std::vector<double> &xd)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movl(const std::vector<double> &xd)
{
    AMOR_VECTOR7 positions;

    positions[0] = xd[0] * 1000;  // [mm]
    positions[1] = xd[1] * 1000;
    positions[2] = xd[2] * 1000;

    positions[3] = toRad(xd[3]);  // [rad]
    positions[4] = toRad(xd[4]);
    positions[5] = toRad(xd[5]);

    if (amor_set_cartesian_positions(handle, positions) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::movv(const std::vector<double> &xdotd)
{
    AMOR_VECTOR7 velocities;

    velocities[0] = xdotd[0] * 1000;  // [mm/s]
    velocities[1] = xdotd[1] * 1000;
    velocities[2] = xdotd[2] * 1000;

    velocities[3] = toRad(xdotd[3]);  // [rad/s]
    velocities[4] = toRad(xdotd[4]);
    velocities[5] = toRad(xdotd[5]);

    if (amor_set_cartesian_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::gcmp()
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::forc(const std::vector<double> &td)
{
    CD_WARNING("Not implemented.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::stopControl()
{
    if (amor_controlled_stop(handle) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::tool(const std::vector<double> &x)
{
    CD_WARNING("Tool change is not supported on AMOR.\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::fwd(const std::vector<double> &rot, double step)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::bkwd(const std::vector<double> &rot, double step)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::rot(const std::vector<double> &rot)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::pan(const std::vector<double> &transl)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::vmos(const std::vector<double> &xdot)
{
    AMOR_VECTOR7 velocities;

    velocities[0] = xdot[0] * 1000;  // [mm/s]
    velocities[1] = xdot[1] * 1000;
    velocities[2] = xdot[2] * 1000;

    velocities[3] = toRad(xdot[3]);  // [rad/s]
    velocities[4] = toRad(xdot[4]);
    velocities[5] = toRad(xdot[5]);

    if (amor_set_cartesian_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::eff(const std::vector<double> &xdotee)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorCartesianControl::pose(const std::vector<double> &x, double interval)
{
    int state;
    std::vector<double> xCurrent;

    if (!stat(state, xCurrent))
    {
        CD_ERROR("stat failed.\n");
        return false;
    }

    KDL::Frame fDesired = KdlVectorConverter::vectorToFrame(x);
    KDL::Frame fCurrent = KdlVectorConverter::vectorToFrame(xCurrent);

    KDL::Twist t = KDL::diff(fCurrent, fDesired, interval);

    std::vector<double> xdot = KdlVectorConverter::twistToVector(t);

    // gain value still experimental
    std::transform(xdot.begin(), xdot.end(), xdot.begin(), std::bind1st(std::multiplies<double>(), 0.005));

    AMOR_VECTOR7 velocities;

    velocities[0] = xdot[0] * 1000;  // [mm/s]
    velocities[1] = xdot[1] * 1000;
    velocities[2] = xdot[2] * 1000;

    velocities[3] = xdot[3];  // [rad/s]
    velocities[4] = xdot[4];
    velocities[5] = xdot[5];

    if (amor_set_cartesian_velocities(handle, velocities) != AMOR_SUCCESS)
    {
        CD_ERROR("%s\n", amor_error());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
