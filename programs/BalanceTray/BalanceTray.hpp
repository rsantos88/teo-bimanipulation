// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StaticLibrary.hpp"
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include "ColorDebug.h"
#include "ICartesianSolver.h"
#include <ICartesianTrajectory.hpp>
#include <KdlTrajectory.hpp>
#include "KinematicRepresentation.hpp"

#include "TrajectoryThread.hpp"
#include "BalanceThread.hpp"



#define DEFAULT_ROBOT "teo" // teo or teoSim (default teo)
#define DEFAULT_MODE "keyboard"
#define PT_MODE_MS 50
#define INPUT_READING_MS 10

using namespace yarp::os;
using namespace roboticslab;

namespace teo
{

/**
 * @ingroup teo-bimanipulation_programs
 *
 * @brief Balance Tray Core.
 *
 */

class BalanceTray : public yarp::os::RFModule, public yarp::os::RateThread
{
public:
    BalanceTray() :  yarp::os::RateThread(INPUT_READING_MS) {} // constructor
    virtual bool configure(yarp::os::ResourceFinder &rf);

    /** current vector position of the tray centroid **/
    std::vector<double> rdsxaa;
    std::vector<double> ldsxaa;

private:

    /** robot used (teo/teoSim) **/
    std::string robot;

    /** control mode: jr3/keyboard **/
    bool jr3Balance;
    bool testMov;
    bool keyboard;
    bool jr3ToCsv;


    /** Operating mode: jr3Balance / keyboard / jr3Check2Csv **/
    std::string mode;

    /** RFModule functions */
    virtual bool interruptModule();
    virtual double getPeriod();
    virtual bool updateModule();

    /*-- Right Arm Device --*/
    int numLeftArmJoints;
    /** Devices **/
    yarp::dev::PolyDriver rightArmDevice;
    yarp::dev::IEncoders *rightArmIEncoders;
    yarp::dev::IControlMode *rightArmIControlMode;
    yarp::dev::IPositionControl *rightArmIPositionControl;
    yarp::dev::IPositionDirect *rightArmIPositionDirect;
    yarp::dev::IControlLimits *rightArmIControlLimits;
    yarp::dev::IRemoteVariables *rightArmIRemoteVariables;

    /** Solver device **/
    yarp::dev::PolyDriver rightArmSolverDevice;
    ICartesianSolver *rightArmICartesianSolver;

    TrajectoryThread *rightArmTrajThread;
    BalanceThread *rightArmBalThread; // point2point
    bool getRightArmFwdKin(std::vector<double> *currentX);

    /*-- Left Arm Device --*/
    int numRightArmJoints; //rightArmAxis
    /** Device **/
    yarp::dev::PolyDriver leftArmDevice;
    yarp::dev::IEncoders *leftArmIEncoders;
    yarp::dev::IControlMode *leftArmIControlMode;
    yarp::dev::IPositionControl *leftArmIPositionControl;
    yarp::dev::IPositionDirect *leftArmIPositionDirect;
    yarp::dev::IControlLimits *leftArmIControlLimits;
    yarp::dev::IRemoteVariables *leftArmIRemoteVariables;

    /** Solver device **/
    yarp::dev::PolyDriver leftArmSolverDevice;
    ICartesianSolver *leftArmICartesianSolver;

    TrajectoryThread *leftArmTrajThread;
    BalanceThread *leftArmBalThread;
    bool getLeftArmFwdKin(std::vector<double> *currentX);

    /** JR3 device **/
    yarp::dev::PolyDriver jr3card;
    yarp::dev::IAnalogSensor *iAnalogSensor;
    yarp::sig::Vector sensorValues;

    /** Reference position functions **/
    std::vector<double> rightArmRefpos;
    std::vector<double>  leftArmRefpos;
    bool setRefPosition(std::vector<double> rx, std::vector<double> lx);
    bool getRefPosition(std::vector<double> *rx, std::vector<double> *lx);
    bool homePosition(); // initial pos

    /****** FUNCTIONS ******/

    /** Execute trajectory using a thread and KdlTrajectory**/
    bool executeTrajectory(std::vector<double> rx, std::vector<double> lx, std::vector<double> rxd, std::vector<double> lxd, double duration, double maxvel);
    bool rotateTrayByTrajectory(int axis, double angle, double duration, double maxvel);

    /** Configure functions **/
    bool configArmsToPosition(double sp, double acc);
    bool configArmsToPositionDirect();

    /** Modes to move the joins **/
    bool moveJointsInPosition(std::vector<double> &rightArm, std::vector<double>& leftArm);
    bool moveJointsInPositionDirect(std::vector<double> &rightArm, std::vector<double> &leftArm);

    /** calculate next point in relation to the forces readed by the sensor or key pressed **/
    bool calculatePointOpposedToForce(yarp::sig::Vector sensor, std::vector<double> *rdx, std::vector<double> *ldx);
    bool calculatePointPressingKeyboard(std::vector<double> *rdx, std::vector<double> *ldx);

    /** Check movements functions */
    void checkLinearlyMovement();

    /** Get axis rotation of the tray **/
    bool getAxisRotation(std::vector<double> *axisRotation);

    /** Write information in CSV file **/
    bool passJr3ValuesToCsv();
    FILE *fp;
    bool writeInfo2Csv(double timeStamp, std::vector<double> axisRotation, yarp::sig::Vector jr3Values);
    // ireration
    int i;

    /** Show information **/
    void printFKinAAS();
    void printFKinAA();
    void printJr3(yarp::sig::Vector values);

    /** Current time **/
    double initTime;

    /** Thread run */
    virtual bool threadInit();
    virtual void run();


}; // class BalanceTray
}
