

#include <crawl_planner/MPCPlanner.h>
#include <dls_controller/support/ConsoleUtility.h>
#include <stdlib.h>
#include <crawl_controller/GaitSequencer.h>

using namespace Eigen;
using namespace std;
using namespace iit::dog;


#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;

GaitSequencer myGaitSchedule;
double stridePhase;
iit::dog::LegBoolMap swingingFlag = false; //stores the time of the steps
iit::dog::LegBoolMap gait_swing_status = false;
bool detected_switch = false;
double duty_factor = 0.85;
//init params
int horizon_size = 200;//10 default for tests
double Ts = 0.1;
double cycle_time = 4.0;
double robotMass = 86.6;
Vector2d userSpeed;
LegDataMap<MPCPlanner::footState> feetStates;
LegDataMap<MPCPlanner::footState> footHolds;
LegDataMap<double> feetValuesX, feetValuesY;
LegDataMap<double> initial_feet_x;
LegDataMap<double> initial_feet_y;
LegDataMap<VectorXd> grForcesZ;
Vector2d initialBasePos;
VectorXd basePosition_x,basePosition_y, baseVelocity_x,baseVelocity_y ;
VectorXd strideparam;

int main()
{
//get user input
newline::getInt("horizon_size:", horizon_size, horizon_size);
MPCPlanner myPlanner(horizon_size,    Ts,    9.81);
newline::getDouble("cycle time:", cycle_time, cycle_time);

userSpeed(0)=0.05;
userSpeed(1)=0.0;
newline::getDouble("userSpeedX:", userSpeed(0), userSpeed(0));
newline::getDouble("userSpeedY:", userSpeed(1), userSpeed(1));

initialBasePos << 0.0,0.0;
newline::getDouble("initial Base position X:", initialBasePos(0), initialBasePos(0));
newline::getDouble("initial Base position Y:", initialBasePos(1), initialBasePos(1));

newline::getDouble("total robot mass:",robotMass, robotMass);

//init stuff
//sequence of legs in the cycle
myGaitSchedule.setSequence(iit::dog::RH, iit::dog::RF, iit::dog::LH, iit::dog::LF);
myGaitSchedule.setTaskServoRate(1/Ts);
//set duty factors for each leg
myGaitSchedule.setDutyFactor(duty_factor, duty_factor, duty_factor, duty_factor);
//set offsets
myGaitSchedule.setOffsetAboutCycleStart(0.0  ,0.25  , 0.5 , 0.75 ); //TODO fix this it does not reset
//set cycle duration
myGaitSchedule.setTotalCycleDuration(cycle_time); //cycle time is only for 1 step!
strideparam.resize(horizon_size);
basePosition_x.resize(horizon_size);
basePosition_y.resize(horizon_size);
baseVelocity_x.resize(horizon_size);
baseVelocity_y.resize(horizon_size);

//initial feet position
feetValuesX[LF] = 0.3;
feetValuesX[RF] = 0.3;
feetValuesX[LH] = -0.3;
feetValuesX[RH] = -0.3;
//init y all the same
feetValuesY[LF] = 0.2;
feetValuesY[RF] = -0.2;
feetValuesY[LH] = 0.2;
feetValuesY[RH] = -0.2;


for (int leg=LF;leg<=RH;leg++){
    feetStates[leg].resize(horizon_size);
    grForcesZ[leg].resize(horizon_size);
    //set always stances
    feetStates[leg].swing.setConstant(horizon_size,false);
}

baseVelocity_x.setConstant(horizon_size,userSpeed(0));
baseVelocity_y.setConstant(horizon_size,userSpeed(1));

//run
for (int i = 0; i < horizon_size; i++)
{
    myGaitSchedule.updateGaitScheduler(swingingFlag , detected_switch);
    feetStates[LF].x[i] = feetValuesX[LF];
    feetStates[RF].x[i] = feetValuesX[RF];
    feetStates[LH].x[i] = feetValuesX[LH];
    feetStates[RH].x[i] = feetValuesX[RH];

    feetStates[LF].y[i] = feetValuesY[LF];
    feetStates[RF].y[i] = feetValuesY[RF];
    feetStates[LH].y[i] = feetValuesY[LH];
    feetStates[RH].y[i] = feetValuesY[RH];

    gait_swing_status = myGaitSchedule.getSwingLegState();
    feetStates[LF].swing[i] = gait_swing_status[LF];
    feetStates[RF].swing[i] = gait_swing_status[RF];
    feetStates[LH].swing[i] = gait_swing_status[LH];
    feetStates[RH].swing[i] = gait_swing_status[RH];
    //advance step
    if (detected_switch)
    {
        for (int leg=LF; leg<=RH;leg++)
        {
            if (gait_swing_status[leg])
            {
                //compute step as length covered in 1/4th of cycle at userSpeed
                //scaling to the dutyfactor (since thee is a stance phase for the feet to catch up with com they should step longer)
                double scaling = 1/duty_factor;
                feetValuesX[leg] += scaling*cycle_time*userSpeed(0);
                feetValuesY[leg] += scaling*cycle_time*userSpeed(1);

            }
        }
        detected_switch = false;
    }

    double num_stance_legs = compute_stance_legs(!gait_swing_status);

    grForcesZ[LF](i)=    robotMass*rbd::g /num_stance_legs;
    grForcesZ[RF](i)=    robotMass*rbd::g /num_stance_legs;
    grForcesZ[LH](i)=    robotMass*rbd::g /num_stance_legs;
    grForcesZ[RH](i)=    robotMass*rbd::g /num_stance_legs;

    //integrate base position
    if (i>0)
    {
        basePosition_x[i] = basePosition_x[i-1] + baseVelocity_x[i]*Ts;
        basePosition_y[i] = basePosition_y[i-1] + baseVelocity_y[i]*Ts;
    }else {
        basePosition_x[i] =initialBasePos(rbd::X);
        basePosition_y[i] =initialBasePos(rbd::Y);
    }


    strideparam[i] = myGaitSchedule.getStrideParametrization(); //this is for debug

}
//myPlanner.computeSteps(userSpeed, initial_feet_x, initial_feet_y, number_of_steps, horizon_size, feetStates, footHolds, A, b, myPlanner,iit::dog::LF);

myPlanner.saveTraj("footPosLF.txt",  feetStates[LF].x, feetStates[LF].y);
myPlanner.saveTraj("footPosRF.txt",  feetStates[RF].x, feetStates[RF].y);
myPlanner.saveTraj("footPosLH.txt",  feetStates[LH].x, feetStates[LH].y);
myPlanner.saveTraj("footPosRH.txt",  feetStates[RH].x, feetStates[RH].y);


myPlanner.saveTraj("swingLF.txt",  feetStates[LF].swing);
myPlanner.saveTraj("swingRF.txt",  feetStates[RF].swing);
myPlanner.saveTraj("swingLH.txt",  feetStates[LH].swing);
myPlanner.saveTraj("swingRH.txt",  feetStates[RH].swing);


myPlanner.saveTraj("grForcesLF_Z.txt",  grForcesZ[LF]);
myPlanner.saveTraj("grForcesRF_Z.txt",  grForcesZ[RF]);
myPlanner.saveTraj("grForcesLH_Z.txt",  grForcesZ[LH]);
myPlanner.saveTraj("grForcesRH_Z.txt",  grForcesZ[RH]);


myPlanner.saveTraj("basePosition.txt", basePosition_x, basePosition_y);
myPlanner.saveTraj("baseVelocity.txt", baseVelocity_x, baseVelocity_y);

//base position

myPlanner.saveTraj("strideparam.txt",  strideparam);

}




