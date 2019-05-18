// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

#include <yarp/math/Math.h>

#include <ColorDebug.h>

#include "AsibotSolverImpl.hpp"

// ------------------- AsibotSolver Related ------------------------------------

roboticslab::AsibotSolver::AsibotSolver()
    : impl(NULL),
      orient(KinRepresentation::AXIS_ANGLE_SCALED)
{}

// ------------------- DeviceDriver Related ------------------------------------

bool roboticslab::AsibotSolver::open(yarp::os::Searchable& config)
{
    CD_DEBUG("config: %s.\n", config.toString().c_str());

    double A0 = config.check("A0", yarp::os::Value(DEFAULT_A0), "length of link 1 (meters)").asFloat64();
    double A1 = config.check("A1", yarp::os::Value(DEFAULT_A1), "length of link 2 (meters)").asFloat64();
    double A2 = config.check("A2", yarp::os::Value(DEFAULT_A2), "length of link 3 (meters)").asFloat64();
    double A3 = config.check("A3", yarp::os::Value(DEFAULT_A3), "length of link 4 (meters)").asFloat64();

    CD_INFO("AsibotSolver using A0: %f, A1: %f, A2: %f, A3: %f.\n", A0, A1, A2, A3);

    if (!config.check("mins") || !config.check("maxs"))
    {
        CD_ERROR("Missing 'mins' and/or 'maxs' option(s).\n");
        return false;
    }

    yarp::os::Bottle *mins = config.findGroup("mins", "joint lower limits (meters or degrees)").get(1).asList();
    yarp::os::Bottle *maxs = config.findGroup("maxs", "joint upper limits (meters or degrees)").get(1).asList();

    if (mins == YARP_NULLPTR || maxs == YARP_NULLPTR)
    {
        CD_ERROR("Empty 'mins' and/or 'maxs' option(s)\n");
        return false;
    }

    if (mins->size() != NUM_MOTORS || maxs->size() != NUM_MOTORS)
    {
        CD_ERROR("mins.size(), maxs.size() (%d, %d) != NUM_MOTORS (%d)\n", mins->size(), maxs->size(), NUM_MOTORS);
        return false;
    }

    std::vector<double> qMin(NUM_MOTORS), qMax(NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        qMin[i] = mins->get(i).asFloat64();
        qMax[i] = maxs->get(i).asFloat64();

        if (qMin[i] == qMax[i])
        {
            CD_WARNING("qMin[%1$d] == qMax[%1$d] (%2$f)\n", i, qMin[i]);
        }
        else if (qMin[i] > qMax[i])
        {
            CD_ERROR("qMin[%1$d] > qMax[%1$d] (%2$f > %3$f)\n", i, qMin[i], qMax[i], i);
            return false;
        }
    }

    impl = new AsibotSolverImpl(A0, A1, A2, A3, qMin, qMax);

    std::string strategy = config.check("invKinStrategy", yarp::os::Value(DEFAULT_STRATEGY), "IK configuration strategy").asString();

    if (!impl->buildStrategyFactory(strategy))
    {
        CD_ERROR("Unsupported IK configuration strategy: %s.\n", strategy.c_str());
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::close()
{
    if (impl != NULL)
    {
        delete impl;
        impl = NULL;
    }

    return true;
}

// ------------------- ICartesianSolver Related ------------------------------------

bool roboticslab::AsibotSolver::getNumJoints(int* numJoints)
{
    return impl->getNumJoints(numJoints);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::appendLink(const std::vector<double> &x)
{
    if (orient == KinRepresentation::AXIS_ANGLE_SCALED)
    {
        return impl->appendLink(x);
    }

    std::vector<double> xOrient;

    if (!KinRepresentation::encodePose(x, xOrient, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("encodePose(x) failed.\n");
        return false;
    }

    return impl->appendLink(xOrient);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::restoreOriginalChain()
{
    return impl->restoreOriginalChain();
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::changeOrigin(const std::vector<double> &x_old_obj, const std::vector<double> &x_new_old, std::vector<double> &x_new_obj)
{
    if (orient == KinRepresentation::AXIS_ANGLE_SCALED)
    {
        return impl->changeOrigin(x_old_obj, x_new_old, x_new_obj);
    }

    std::vector<double> x_old_obj_orient, x_new_old_orient;

    if (!KinRepresentation::encodePose(x_old_obj, x_old_obj_orient, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("encodePose(x_old_obj) failed.\n");
        return false;
    }

    if (!KinRepresentation::encodePose(x_new_old, x_new_old_orient, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("encodePose(x_new_old) failed.\n");
        return false;
    }

    if (!impl->changeOrigin(x_old_obj_orient, x_new_old_orient, x_new_obj))
    {
        return false;
    }

    if (!KinRepresentation::decodePose(x_new_obj, x_new_obj, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("decodePose(x_new_obj) failed.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x)
{
    if (!impl->fwdKin(q, x))
    {
        return false;
    }

    if (orient != KinRepresentation::AXIS_ANGLE_SCALED)
    {
        if (!KinRepresentation::decodePose(x, x, KinRepresentation::CARTESIAN, orient))
        {
            CD_ERROR("decodePose(x) failed.\n");
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut)
{
    if (orient == KinRepresentation::AXIS_ANGLE_SCALED)
    {
        return impl->poseDiff(xLhs, xRhs, xOut);
    }

    std::vector<double> xLhsOrient, xRhsOrient;

    if (!KinRepresentation::encodePose(xLhs, xLhsOrient, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("encodePose(xLhs) failed.\n");
        return false;
    }

    if (!KinRepresentation::encodePose(xRhs, xRhsOrient, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("encodePose(xRhs) failed.\n");
        return false;
    }

    if (!impl->poseDiff(xLhsOrient, xRhsOrient, xOut))
    {
        return false;
    }

    if (!KinRepresentation::decodePose(xOut, xOut, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("decodePose(xOut) failed.\n");
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q, const reference_frame frame)
{
    if (orient == KinRepresentation::AXIS_ANGLE_SCALED)
    {
        return impl->invKin(xd, qGuess, q, frame);
    }

    std::vector<double> xdOrient;

    if (!KinRepresentation::encodePose(xd, xdOrient, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("encodePose(xd) failed.\n");
        return false;
    }

    return impl->invKin(xd, qGuess, q, frame);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot, const reference_frame frame)
{
    if (orient == KinRepresentation::AXIS_ANGLE_SCALED)
    {
        return impl->diffInvKin(q, xdot, qdot, frame);
    }

    std::vector<double> x, xdotOrient;

    if (!impl->fwdKin(q, x))
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }

    if (!KinRepresentation::encodeVelocity(x, xdot, xdotOrient, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("encodeVelocity(xdot) failed.\n");
        return false;
    }

    return impl->diffInvKin(q, xdotOrient, qdot, frame);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invDyn(const std::vector<double> &q, std::vector<double> &t)
{
    if (orient == KinRepresentation::AXIS_ANGLE_SCALED)
    {
        return impl->invDyn(q, t);
    }

    std::vector<double> x, xdot(6, 0.0), tOrient;

    if (!impl->fwdKin(q, x))
    {
        CD_ERROR("fwdKin failed.\n");
        return false;
    }

    if (!KinRepresentation::encodeAcceleration(x, xdot, t, tOrient, KinRepresentation::CARTESIAN, orient))
    {
        CD_ERROR("encodeAcceleration(t) failed.\n");
        return false;
    }

    return impl->invDyn(q, tOrient);
}

// -----------------------------------------------------------------------------

bool roboticslab::AsibotSolver::invDyn(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t)
{
    if (orient != KinRepresentation::AXIS_ANGLE_SCALED)
    {

        CD_ERROR("Unsupported angle representation.\n");
        return false;
    }

    return impl->invDyn(q, qdot, qdotdot, fexts, t);
}

// -----------------------------------------------------------------------------
