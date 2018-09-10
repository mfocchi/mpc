/*
 * test.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: mfocchi
 */


#include <crawl_planner/MPCPlanner.h>
#include <dls_controller/support/ConsoleUtility.h>
#include <stdlib.h>
#include <crawl_planner/FootScheduler.h> //TODO fix this


using namespace Eigen;
using namespace std;
using namespace iit::dog;


#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;



int main()
{



//init params
int horizon_size = 200;//10 default for tests
int number_of_steps = 20;
double distance = 3.0;


double height = 0.5;
double Ts = 0.1;
double weight_R = 1e-06;
double weight_Q = 1;

VectorXd  jerk_x, jerk_y;
Vector3d  initial_state_x = Vector3d(0.0, 0.0,0.0);
Vector3d  initial_state_y = Vector3d(0.0,-0.0,0.0);
VectorXd zmp_x, zmp_y, com_x, com_y, viol;
MatrixXd A; VectorXd b;

int optimizeVelocityFlag = true;
double disturbance = 0.0;

VectorXd com_xd, com_yd;
Vector2d userSpeed;
userSpeed(0)=0.15;
userSpeed(1)=0.0;


//get user input
newline::getInt("horizon_size:", horizon_size, horizon_size);
newline::getInt("number_of_steps:", number_of_steps, number_of_steps);

//newline::getDouble("initial state pos:", initial_state_x(0), initial_state_x(0));
//newline::getDouble("initial state vel:", initial_state_x(1), initial_state_x(1));
//newline::getDouble("initial state acc:", initial_state_x(2), initial_state_x(2));

newline::getInt("optimize velocity? [0/1]:", optimizeVelocityFlag, optimizeVelocityFlag);
newline::getDouble("disturbance:", disturbance, disturbance);
newline::getDouble("userSpeedX:", userSpeed(0), userSpeed(0));
newline::getDouble("userSpeedY:", userSpeed(1), userSpeed(1));

MPCPlanner myPlanner(horizon_size,    Ts,    9.81);


//compute polygons
LegDataMap<MPCPlanner::footState> feetStates;
LegDataMap<MPCPlanner::footState> footHolds;
//initial values
//the initial value of the zmp should be inside the initial polygon otherwise it does not find a solution!!
//so be careful with initial values of com!
LegDataMap<double> initial_feet_x;
LegDataMap<double> initial_feet_y;
initial_feet_x[LF] = initial_state_x(0) + 0.1;
initial_feet_x[RF] = initial_state_x(0) + 0.2;
initial_feet_x[LH] = initial_feet_x[LF] - 0.5;
initial_feet_x[RH] = initial_feet_x[RF] - 0.5;
//init y all the same
initial_feet_y[LF] = initial_state_y(0) + 1.0;
initial_feet_y[RF] = initial_state_y(0) -1.0;
initial_feet_y[LH] = initial_state_y(0) + 1.0;
initial_feet_y[RH] = initial_state_y(0) -1.0;

//matlab file plotTrajXYconstraintCoupled
/////old stuff rewritten with compute steps function
myPlanner.computeSteps(initial_feet_x, initial_feet_y, distance, number_of_steps, horizon_size, feetStates, footHolds, A, b, myPlanner);
if (!optimizeVelocityFlag){
    myPlanner.solveQPConstraintCoupled(height,initial_state_x, initial_state_y , A,b,jerk_x,jerk_y);
    prt(A)
            prt(b)
}else {
    weight_R = 0.01; //gives good results
    newline::getDouble("weight R:", weight_R, weight_R);
    newline::getDouble("weight Q:", weight_Q, weight_Q);
    myPlanner.setWeights(weight_R, weight_Q);
    //velocity is enforced at the end of the horizon
    myPlanner.solveQPConstraintCoupled(height,initial_state_x, initial_state_y , A,b, userSpeed, jerk_x,jerk_y);
}
viol = myPlanner.getConstraintViolation(feetStates);
prt(jerk_x.transpose())
prt(jerk_y.transpose())

myPlanner.computeZMPtrajectory( initial_state_x, jerk_x, zmp_x);
myPlanner.computeZMPtrajectory( initial_state_y, jerk_y, zmp_y);
myPlanner.computeCOMtrajectory( initial_state_x, jerk_x, com_x);
myPlanner.computeCOMtrajectory( initial_state_y, jerk_y, com_y);

myPlanner.computeCOMtrajectory( initial_state_x, jerk_x, com_xd, MPCPlanner::VELOCITY);
myPlanner.computeCOMtrajectory( initial_state_y, jerk_y, com_yd, MPCPlanner::VELOCITY);


myPlanner.saveTraj("jerk_x.txt", jerk_x);
myPlanner.saveTraj("jerk_y.txt", jerk_y);
myPlanner.saveTraj("zmp_x.txt", zmp_x);
myPlanner.saveTraj("zmp_y.txt", zmp_y);
myPlanner.saveTraj("com_x.txt", com_x);
myPlanner.saveTraj("com_y.txt", com_y);
myPlanner.saveTraj("com_xd.txt", com_xd);
myPlanner.saveTraj("com_yd.txt", com_yd);
myPlanner.saveTraj("viol.txt",  viol);

myPlanner.saveTraj("footPosLF.txt",  feetStates[LF].x, feetStates[LF].y);
myPlanner.saveTraj("footPosRF.txt",  feetStates[RF].x, feetStates[RF].y);
myPlanner.saveTraj("footPosLH.txt",  feetStates[LH].x, feetStates[LH].y);
myPlanner.saveTraj("footPosRH.txt",  feetStates[RH].x, feetStates[RH].y);

myPlanner.saveTraj("footHoldsLF.txt",  footHolds[LF].x, footHolds[LF].y, 2*number_of_steps);
myPlanner.saveTraj("footHoldsRF.txt",  footHolds[RF].x, footHolds[RF].y, 2*number_of_steps);
myPlanner.saveTraj("footHoldsLH.txt",  footHolds[LH].x, footHolds[LH].y, 2*number_of_steps);
myPlanner.saveTraj("footHoldsRH.txt",  footHolds[RH].x, footHolds[RH].y, 2*number_of_steps);

myPlanner.saveTraj("swingLF.txt",  feetStates[LF].swing);
myPlanner.saveTraj("swingRF.txt",  feetStates[RF].swing);
myPlanner.saveTraj("swingLH.txt",  feetStates[LH].swing);
myPlanner.saveTraj("swingRH.txt",  feetStates[RH].swing);

}



