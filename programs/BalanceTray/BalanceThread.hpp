// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/RateThread.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include "ICartesianSolver.h"
#include <ConfigurationSelector.hpp>
#include <ICartesianTrajectory.hpp>

class BalanceThread : public yarp::os::RateThread
{
public:
    BalanceThread(yarp::dev::IEncoders *iEncoders,                  
                  roboticslab::ICartesianSolver *iCartesianSolver,
                  yarp::dev::IPositionDirect *iPositionDirect,
                  int period)
        : yarp::os::RateThread(period),
          iEncoders(iEncoders),
          iCartesianSolver(iCartesianSolver),
          iPositionDirect(iPositionDirect),
          axes(0),
          startTime(0)
    {}

    void setCartesianPosition(std::vector<double> position) {
       this->position = position;
    }


    void getCartesianPosition(std::vector<double> *position) {
        *position = this->position;
    }



protected:    
    virtual bool threadInit();
    virtual void run();

private:
    yarp::dev::IEncoders *iEncoders;    
    roboticslab::ICartesianSolver *iCartesianSolver;
    yarp::dev::IPositionDirect *iPositionDirect;
    std::vector<double> position;
    int axes;
    double startTime;

};

