// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "KdlSolver.hpp"

#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Matrix.h>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <Eigen/Core> // Eigen::Matrix

#include <ColorDebug.h>

#include "ChainIkSolverPos_ST.hpp"
#include "ChainIkSolverPos_ID.hpp"

#include "KdlSolverImpl.hpp"

// ------------------- KdlSolver Related ------------------------------------

namespace
{
    KDL::Chain * chainClone;
    KDL::ChainFkSolverPos * fkSolverPos;
    KDL::ChainIkSolverPos * ikSolverPos;
    KDL::ChainIkSolverVel * ikSolverVel;
    KDL::ChainIdSolver * idSolver;

    bool getMatrixFromProperties(const yarp::os::Searchable & options, const std::string & tag, yarp::sig::Matrix & H)
    {
        yarp::os::Bottle * bH = options.find(tag).asList();

        if (!bH)
        {
            CD_WARNING("Unable to find tag %s.\n", tag.c_str());
            return false;
        }

        int i = 0;
        int j = 0;

        H.zero();

        for (int cnt = 0; cnt < bH->size() && cnt < H.rows() * H.cols(); cnt++)
        {
            H(i, j) = bH->get(cnt).asFloat64();

            if (++j >= H.cols())
            {
                i++;
                j = 0;
            }
        }

        return true;
    }

    bool parseLmaFromBottle(const yarp::os::Bottle & b, Eigen::Matrix<double, 6, 1> & L)
    {
        if (b.size() != 6)
        {
            CD_WARNING("Wrong bottle size (expected: %d, was: %d).\n", 6, b.size());
            return false;
        }

        for (int i = 0; i < b.size(); i++)
        {
            L(i) = b.get(i).asFloat64();
        }

        return true;
    }

    bool retrieveJointLimits(const yarp::os::Searchable & options, KDL::JntArray & qMin, KDL::JntArray & qMax)
    {
        int nrOfJoints = qMin.rows();

        if (!options.check("mins") || !options.check("maxs"))
        {
            CD_ERROR("Missing 'mins' and/or 'maxs' option(s).\n");
            return false;
        }

        yarp::os::Bottle * maxs = options.findGroup("maxs", "joint upper limits (meters or degrees)").get(1).asList();
        yarp::os::Bottle * mins = options.findGroup("mins", "joint lower limits (meters or degrees)").get(1).asList();

        if (maxs == YARP_NULLPTR || mins == YARP_NULLPTR)
        {
            CD_ERROR("Empty 'mins' and/or 'maxs' option(s)\n");
            return false;
        }

        if (maxs->size() < nrOfJoints || mins->size() < nrOfJoints)
        {
            CD_ERROR("chain.getNrOfJoints (%d) > maxs.size() or mins.size() (%d, %d)\n", nrOfJoints, maxs->size(), mins->size());
            return false;
        }

        for (int motor = 0; motor < nrOfJoints; motor++)
        {
            qMax(motor) = maxs->get(motor).asFloat64();
            qMin(motor) = mins->get(motor).asFloat64();

            if (qMin(motor) == qMax(motor))
            {
                CD_WARNING("qMin[%1$d] == qMax[%1$d] (%2$f)\n", motor, qMin(motor));
            }
            else if (qMin(motor) > qMax(motor))
            {
                CD_ERROR("qMin[%1$d] > qMax[%1$d] (%2$f > %3$f)\n", motor, qMin(motor), qMax(motor));
                return false;
            }
        }

        return true;
    }
}

// -----------------------------------------------------------------------------

roboticslab::KdlSolver::KdlSolver()
    : impl(NULL),
      orient(KinRepresentation::AXIS_ANGLE_SCALED)
{}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::open(yarp::os::Searchable& config)
{
    CD_DEBUG("config: %s.\n", config.toString().c_str());

    //-- kinematics
    std::string kinematics = config.check("kinematics", yarp::os::Value(DEFAULT_KINEMATICS), "path to file with description of robot kinematics").asString();
    CD_INFO("kinematics: %s [%s]\n", kinematics.c_str(), DEFAULT_KINEMATICS);

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext("kinematics");
    std::string kinematicsFullPath = rf.findFileByName(kinematics);

    yarp::os::Property fullConfig;
    fullConfig.fromConfigFile(kinematicsFullPath.c_str());
    fullConfig.fromString(config.toString(), false);  //-- Can override kinematics file contents.
    fullConfig.setMonitor(config.getMonitor(), "KdlSolver");

    CD_DEBUG("fullConfig: %s.\n", fullConfig.toString().c_str());

    std::string angleReprStr = fullConfig.check("angleRepr", yarp::os::Value(""), "angle representation").asString();

    if (!KinRepresentation::parseEnumerator(angleReprStr, &orient))
    {
        CD_ERROR("Empty or unknown angle representation option: %s\n", angleReprStr.c_str());
        return false;
    }

    //-- numlinks
    int numLinks = fullConfig.check("numLinks", yarp::os::Value(DEFAULT_NUM_LINKS), "chain number of segments").asInt32();
    CD_INFO("numLinks: %d [%d]\n", numLinks, DEFAULT_NUM_LINKS);

    //-- gravity
    yarp::os::Value defaultGravityValue;
    yarp::os::Bottle *defaultGravityBottle = defaultGravityValue.asList();
    defaultGravityBottle->addFloat64(0);
    defaultGravityBottle->addFloat64(0);
    defaultGravityBottle->addFloat64(-9.81);

    yarp::os::Value gravityValue = fullConfig.check("gravity", defaultGravityValue, "gravity vector (SI units)");
    yarp::os::Bottle *gravityBottle = gravityValue.asList();
    KDL::Vector gravity(gravityBottle->get(0).asFloat64(), gravityBottle->get(1).asFloat64(), gravityBottle->get(2).asFloat64());
    CD_INFO("gravity: %s [%s]\n", gravityBottle->toString().c_str(), defaultGravityBottle->toString().c_str());

    //-- H0
    yarp::sig::Matrix defaultYmH0(4, 4);
    defaultYmH0.eye();

    yarp::sig::Matrix ymH0(4, 4);
    std::string ymH0_str("H0");

    if (!getMatrixFromProperties(fullConfig, ymH0_str, ymH0))
    {
        ymH0 = defaultYmH0;
    }

    KDL::Chain chain;
    KDL::Vector kdlVec0(ymH0(0, 3), ymH0(1, 3), ymH0(2, 3));
    KDL::Rotation kdlRot0(ymH0(0, 0), ymH0(0, 1), ymH0(0, 2), ymH0(1, 0), ymH0(1, 1), ymH0(1, 2), ymH0(2, 0), ymH0(2, 1), ymH0(2, 2));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRot0,kdlVec0)));  //-- H0 = Frame(kdlRot0,kdlVec0);
    CD_INFO("H0:\n%s\n[%s]\n", ymH0.toString().c_str(), defaultYmH0.toString().c_str());

    //-- links
    for (int linkIndex = 0; linkIndex < numLinks; linkIndex++)
    {
        std::string link("link_");
        std::ostringstream s;
        s << linkIndex;
        link += s.str();

        yarp::os::Bottle &bLink = fullConfig.findGroup(link);

        if (!bLink.isNull())
        {
            //-- Kinematic
            double linkOffset = bLink.check("offset", yarp::os::Value(0.0), "DH joint angle (degrees)").asFloat64();
            double linkD = bLink.check("D", yarp::os::Value(0.0), "DH link offset (meters)").asFloat64();
            double linkA = bLink.check("A", yarp::os::Value(0.0), "DH link length (meters)").asFloat64();
            double linkAlpha = bLink.check("alpha", yarp::os::Value(0.0), "DH link twist (degrees)").asFloat64();

            //-- Dynamic
            if (bLink.check("mass") && bLink.check("cog") && bLink.check("inertia"))
            {
                double linkMass = bLink.check("mass", yarp::os::Value(0.0), "link mass (SI units)").asFloat64();
                yarp::os::Bottle linkCog = bLink.findGroup("cog", "vector of link's center of gravity (SI units)").tail();
                yarp::os::Bottle linkInertia = bLink.findGroup("inertia", "vector of link's inertia (SI units)").tail();

                KDL::Frame frameFromDH = KDL::Frame::DH(linkA, KinRepresentation::degToRad(linkAlpha), linkD, KinRepresentation::degToRad(linkOffset));
                KDL::Vector refToCog(linkCog.get(0).asFloat64(), linkCog.get(1).asFloat64(), linkCog.get(2).asFloat64());
                KDL::RotationalInertia rotInertia(linkInertia.get(0).asFloat64(), linkInertia.get(1).asFloat64(), linkInertia.get(2).asFloat64(), 0, 0, 0);
                KDL::RigidBodyInertia rbInertia(linkMass, refToCog, rotInertia);

                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), frameFromDH, rbInertia));

                CD_INFO("Added: %s (offset %f) (D %f) (A %f) (alpha %f) (mass %f) (cog %f %f %f) (inertia %f %f %f)\n",
                           link.c_str(), linkOffset, linkD, linkA, linkAlpha, linkMass,
                           linkCog.get(0).asFloat64(), linkCog.get(1).asFloat64(), linkCog.get(2).asFloat64(),
                           linkInertia.get(0).asFloat64(), linkInertia.get(1).asFloat64(), linkInertia.get(2).asFloat64());
            }
            else //-- No mass -> skip dynamics
            {
                chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(linkA, KinRepresentation::degToRad(linkAlpha), linkD, KinRepresentation::degToRad(linkOffset))));
                CD_INFO("Added: %s (offset %f) (D %f) (A %f) (alpha %f)\n", link.c_str(), linkOffset, linkD, linkA, linkAlpha);
            }

            continue;
        }

        std::string xyzLink("xyzLink_");
        std::ostringstream xyzS;
        xyzS << linkIndex;
        xyzLink += xyzS.str();
        CD_WARNING("Not found: \"%s\", looking for \"%s\" instead.\n", link.c_str(), xyzLink.c_str());

        yarp::os::Bottle &bXyzLink = fullConfig.findGroup(xyzLink);

        if (bXyzLink.isNull())
        {
            CD_ERROR("Not found: \"%s\" either.\n", xyzLink.c_str());
            return false;
        }

        double linkX = bXyzLink.check("x", yarp::os::Value(0.0), "X coordinate of next frame (meters)").asFloat64();
        double linkY = bXyzLink.check("y", yarp::os::Value(0.0), "Y coordinate of next frame (meters)").asFloat64();
        double linkZ = bXyzLink.check("z", yarp::os::Value(0.0), "Z coordinate of next frame (meters)").asFloat64();

        std::string linkTypes = "joint type (Rot[XYZ]|InvRot[XYZ]|Trans[XYZ]|InvTrans[XYZ]), e.g. 'RotZ'";
        std::string linkType = bXyzLink.check("Type", yarp::os::Value("NULL"), linkTypes.c_str()).asString();

        if (linkType == "RotX")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "RotY")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "RotZ")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "InvRotX")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX, -1.0), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "InvRotY")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY, -1.0), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "InvRotZ")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ, -1.0), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "TransX")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "TransY")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "TransZ")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "InvTransX")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX, -1.0), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "InvTransY")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY, -1.0), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else if (linkType == "InvTransZ")
        {
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ, -1.0), KDL::Frame(KDL::Vector(linkX, linkY, linkZ))));
        }
        else
        {
            CD_WARNING("Link joint type \"%s\" unrecognized!\n", linkType.c_str());
        }

        CD_SUCCESS("Added: %s (Type %s) (x %f) (y %f) (z %f)\n", xyzLink.c_str(), linkType.c_str(), linkX, linkY, linkZ);
    }

    //-- HN
    yarp::sig::Matrix defaultYmHN(4, 4);
    defaultYmHN.eye();

    yarp::sig::Matrix ymHN(4, 4);
    std::string ymHN_str("HN");

    if (!getMatrixFromProperties(fullConfig, ymHN_str, ymHN))
    {
        ymHN = defaultYmHN;
    }

    KDL::Vector kdlVecN(ymHN(0, 3), ymHN(1, 3), ymHN(2, 3));
    KDL::Rotation kdlRotN(ymHN(0, 0), ymHN(0, 1), ymHN(0, 2), ymHN(1, 0), ymHN(1, 1), ymHN(1, 2), ymHN(2, 0), ymHN(2, 1), ymHN(2, 2));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(kdlRotN, kdlVecN)));
    CD_INFO("HN:\n%s\n[%s]\n", ymHN.toString().c_str(), defaultYmHN.toString().c_str());

    CD_INFO("Chain number of segments (post- H0 and HN): %d\n", chain.getNrOfSegments());
    CD_INFO("Chain number of joints (post- H0 and HN): %d\n", chain.getNrOfJoints());

    chainClone = new KDL::Chain(chain);

    fkSolverPos = new KDL::ChainFkSolverPos_recursive(*chainClone);
    ikSolverVel = new KDL::ChainIkSolverVel_pinv(*chainClone);
    idSolver = new KDL::ChainIdSolver_RNE(*chainClone, gravity);

    //-- IK solver algorithm.
    std::string ik = fullConfig.check("ik", yarp::os::Value(DEFAULT_IK_SOLVER), "IK solver algorithm (lma, nrjl, st, id)").asString();

    if (ik == "lma")
    {
        std::string weightsStr = fullConfig.check("weights", yarp::os::Value(DEFAULT_LMA_WEIGHTS), "LMA algorithm weights (bottle of 6 doubles)").asString();
        yarp::os::Bottle weights(weightsStr);
        Eigen::Matrix<double, 6, 1> L;

        if (!parseLmaFromBottle(weights, L))
        {
            CD_ERROR("Unable to parse LMA weights.\n");
            return false;
        }

        ikSolverPos = new KDL::ChainIkSolverPos_LMA(*chainClone, L);
    }
    else if (ik == "nrjl")
    {
        KDL::JntArray qMax(chain.getNrOfJoints());
        KDL::JntArray qMin(chain.getNrOfJoints());

        //-- Joint limits.
        if (!retrieveJointLimits(fullConfig, qMin, qMax))
        {
            CD_ERROR("Unable to retrieve joint limits.\n");
            return false;
        }

        //-- Precision and max iterations.
        double eps = fullConfig.check("eps", yarp::os::Value(DEFAULT_EPS), "IK solver precision (meters)").asFloat64();
        double maxIter = fullConfig.check("maxIter", yarp::os::Value(DEFAULT_MAXITER), "maximum number of iterations").asInt32();

        ikSolverPos = new KDL::ChainIkSolverPos_NR_JL(*chainClone, qMin, qMax, *fkSolverPos, *ikSolverVel, maxIter, eps);
    }
    else if (ik == "st")
    {
        KDL::JntArray qMax(chain.getNrOfJoints());
        KDL::JntArray qMin(chain.getNrOfJoints());

        //-- Joint limits.
        if (!retrieveJointLimits(fullConfig, qMin, qMax))
        {
            CD_ERROR("Unable to retrieve joint limits.\n");
            return false;
        }

        //-- IK configuration selection strategy.
        std::string strategy = fullConfig.check("invKinStrategy", yarp::os::Value(DEFAULT_STRATEGY), "IK configuration strategy").asString();

        if (strategy == "leastOverallAngularDisplacement")
        {
            ConfigurationSelectorLeastOverallAngularDisplacementFactory factory(qMin, qMax);
            ikSolverPos = ChainIkSolverPos_ST::create(*chainClone, factory);
        }
        else
        {
            CD_ERROR("Unsupported IK strategy; %s.\n", strategy.c_str());
            return false;
        }

        if (ikSolverPos == NULL)
        {
            CD_ERROR("Unable to solve IK.\n");
            return false;
        }
    }
    else if (ik == "id")
    {
        KDL::JntArray qMax(chain.getNrOfJoints());
        KDL::JntArray qMin(chain.getNrOfJoints());

        //-- Joint limits.
        if (!retrieveJointLimits(fullConfig, qMin, qMax))
        {
            CD_ERROR("Unable to retrieve joint limits.\n");
            return false;
        }

        ikSolverPos = new ChainIkSolverPos_ID(chain, qMin, qMax, *fkSolverPos);
    }
    else
    {
        CD_ERROR("Unsupported IK solver algorithm: %s.\n", ik.c_str());
        return false;
    }

    impl = new KdlSolverImpl(chainClone, fkSolverPos, ikSolverPos, ikSolverVel, idSolver);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::close()
{
    if (impl != NULL)
    {
        delete impl;
        impl = NULL;
    }

    if (ikSolverPos != NULL)
    {
        delete ikSolverPos;
        ikSolverPos = NULL;
    }

    if (ikSolverVel != NULL)
    {
        delete ikSolverVel;
        ikSolverVel = NULL;
    }

    if (idSolver != NULL)
    {
        delete idSolver;
        idSolver = NULL;
    }

    if (chainClone != NULL)
    {
        delete chainClone;
        chainClone = NULL;
    }

    return true;
}

// ------------------- ICartesianSolver Related ------------------------------------

bool roboticslab::KdlSolver::getNumJoints(int* numJoints)
{
    return impl->getNumJoints(numJoints);
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::appendLink(const std::vector<double>& x)
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

bool roboticslab::KdlSolver::restoreOriginalChain()
{
    return impl->restoreOriginalChain();
}

// -----------------------------------------------------------------------------

bool roboticslab::KdlSolver::changeOrigin(const std::vector<double> &x_old_obj, const std::vector<double> &x_new_old,
        std::vector<double> &x_new_obj)
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

bool roboticslab::KdlSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x)
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

bool roboticslab::KdlSolver::poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut)
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

bool roboticslab::KdlSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q,
        const reference_frame frame)
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

bool roboticslab::KdlSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot,
        const reference_frame frame)
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

bool roboticslab::KdlSolver::invDyn(const std::vector<double> &q, std::vector<double> &t)
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

bool roboticslab::KdlSolver::invDyn(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t)
{
    if (orient != KinRepresentation::AXIS_ANGLE_SCALED)
    {

        CD_ERROR("Unsupported angle representation.\n");
        return false;
    }

    return impl->invDyn(q, qdot, qdotdot, fexts, t);
}

// -----------------------------------------------------------------------------
