// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianRateThread.hpp"

/************************************************************************/
bool teo::CartesianRateThread::threadInit() {

    iEncoders->getAxes( &numMotors );
    CD_INFO("numMotors: %d.\n",numMotors);

    solver->getNumLinks( &solverNumLinks );
    CD_INFO("solverNumLinks: %d.\n",solverNumLinks);

    if( numMotors < solverNumLinks ) {
        CD_ERROR("numMotors < solverNumLinks !!! (must be >=)\n");
        return false;
    }

    q.resize( numMotors );

    iVelocityControl->setVelocityMode();

    return true;
}

/************************************************************************/
void teo::CartesianRateThread::run() {

    iEncoders->getEncoders( q.data() );

    CD_DEBUG("<-- ");
    for(int i=0;i<q.size();i++)
        CD_DEBUG_NO_HEADER("%f ",q[i]);
    CD_DEBUG_NO_HEADER("[deg]\n");

    solver->fwdKin(q,x,o);

    solver->invDyn(q,qdot);

    if( numMotors > numMotors )
        qdot.resize( numMotors );  //-- Extra motors won't care about torques.

    CD_INFO("--> ");
    for(int i=0;i<numMotors;i++) {
        CD_INFO_NO_HEADER("%f ",qdot[i]);
    }
    CD_INFO_NO_HEADER("[Nm]\n");

    //--tRA[0] = 0.0;  //-- Release... let's do this!
    iVelocityControl->velocityMove( qdot.data() );

}

/************************************************************************/
