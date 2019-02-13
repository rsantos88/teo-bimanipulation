// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <string>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include "ColorDebug.h"
#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"


#define DEFAULT_ROBOT "/teo" // teo or teoSim

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
   class BalanceTray : public yarp::os::RFModule, public yarp::os::Thread
    {
        public:
             virtual bool configure(yarp::os::ResourceFinder &rf);
        private:

            /** RFModule interruptModule. */
            virtual bool interruptModule();
            /** RFModule getPeriod. */
            virtual double getPeriod();
            /** RFModule updateModule. */
            virtual bool updateModule();

            /*-- Right Arm Device --*/
            /** Axes number **/
            int numLeftArmJoints;
            /** Device **/
            yarp::dev::PolyDriver rightArmDevice;
            /** Encoders **/
            yarp::dev::IEncoders *rightArmIEncoders;
            /** Right Arm ControlMode2 Interface */
            yarp::dev::IControlMode2 *rightArmIControlMode2;
            /** Right Arm PositionControl2 Interface */
            yarp::dev::IPositionControl2 *rightArmIPositionControl2;
            /** Right Arm PositionDirect Interface */
            yarp::dev::IPositionDirect *rightArmIPositionDirect;
            /** Right Arm ControlLimits2 Interface */
            yarp::dev::IControlLimits *rightArmIControlLimits;
            /** Solver device **/
            yarp::dev::PolyDriver rightArmSolverDevice;
            ICartesianSolver *rightArmICartesianSolver;
            /** Forward Kinematic function **/
            bool getRightArmFwdKin(std::vector<double> *currentX);

            /*-- Left Arm Device --*/
            /** Axes number **/
            int numRightArmJoints;
            /** Device **/
            yarp::dev::PolyDriver leftArmDevice;
            /** Encoders **/
            yarp::dev::IEncoders *leftArmIEncoders;
            /** Left Arm ControlMode2 Interface */
            yarp::dev::IControlMode2 *leftArmIControlMode2;
            /** Left Arm PositionControl2 Interface */
            yarp::dev::IPositionControl2 *leftArmIPositionControl2;
            /** Left Arm PositionDirect Interface */
            yarp::dev::IPositionDirect *leftArmIPositionDirect;
            /** Left Arm ControlLimits2 Interface */
            yarp::dev::IControlLimits *leftArmIControlLimits;
            /** Solver device **/
            yarp::dev::PolyDriver leftArmSolverDevice;
            ICartesianSolver *leftArmICartesianSolver;
            /** Forward Kinematic function **/
            bool getLeftArmFwdKin(std::vector<double> *currentX);

            /** Reference position functions **/
            std::vector<double> referencePosition[2];
            bool saveRefPosition();
            bool goToRefPosition(int points, double delay);

            /****** FUNCTIONS ******/

            /** Interpolate function **/
            std::vector<std::vector<double> > interpolate(std::vector<double> startPose, std::vector<double> endPose, int res);

            /** Configure functions **/
            bool configArmsToPosition(double sp, double acc);
            bool configArmsToPositionDirect();

            /** Modes to move the joins **/
            bool moveJointsInPosition(std::vector<double> &rightArm, std::vector<double>& leftArm);
            bool moveJointsInPositionDirect(std::vector<double> &rightArm, std::vector<double> &leftArm);

            /** Modes to move the tray **/
            bool homePosition(); // initial pos
            bool rotateTrayInPosDirect(int axis, double angle, int points, double delay);
            bool moveTrayLinearlyInPosDirect(int axis, double dist, int points, double delay);
            bool recoverPosition(int points, double delay);

            /** Check movements functions */
            void checkLinearlyMovement(int axis, int points, int rep);
            void checkRotateMovement(int axis, int points, int rep );

            /** Show information **/
            void showArmsFKinAAS();
            void showArmsFKinAA();

            /** movement finished */
            bool done;

            /** Input port from dialogue manager */
            yarp::os::RpcServer inDialogPort;

            /** Thread run */
            virtual void run();


     }; // class BalanceTray
}
