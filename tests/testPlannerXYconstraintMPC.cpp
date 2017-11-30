/*
 * test.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: mfocchi
 */



#include <crawl_planner/MPCPlanner.h>
#include <dls_controller/support/ConsoleUtility.h>
#include <crawl_planner/FootScheduler.h> //TODO fix this

using namespace Eigen;
using namespace std;
#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;

void computeSteps(const LegDataMap<double> & initial_feet_x, const LegDataMap<double> & initial_feet_y,
                  const double distance, const int number_of_steps, const int horizon_size,
                  LegDataMap<MPCPlanner::footState> & feetStates, LegDataMap<MPCPlanner::footState> & footHolds,
                  MatrixXd & A,  VectorXd & b, MPCPlanner & myPlanner, FootScheduler & schedule);
int main()
{



//init params
int horizon_size = 200;//10 default for tests
int number_of_steps = 20;
double distance = 3.0;

double height = 0.5;
double lateral_bound = 0.2;
double Ts = 0.1;
double weight_R = 1e-06;
double weight_Q = 1;

VectorXd  jerk_x, jerk_y;
Vector3d  initial_state_x = Vector3d(0.0, 0.0,0.0);
Vector3d  initial_state_y = Vector3d(0.0,-0.0,0.0);
VectorXd zmp_x, zmp_y, com_x, com_y, viol;
MPCPlanner::BoxLimits zmpLimX, zmpLimY;
MatrixXd A; VectorXd b;
FootScheduler schedule; schedule.setSequence(LF, RH,RF,LH);
//get user input
newline::getInt("horizon_size:", horizon_size, horizon_size);
newline::getInt("number_of_steps:", number_of_steps, number_of_steps);
newline::getDouble("weight R:", weight_R, weight_R);
newline::getDouble("weight Q:", weight_Q, weight_Q);
newline::getDouble("initial state pos:", initial_state_x(0), initial_state_x(0));
newline::getDouble("initial state vel:", initial_state_x(1), initial_state_x(1));
newline::getDouble("initial state acc:", initial_state_x(2), initial_state_x(2));

MPCPlanner myPlanner(horizon_size,    Ts,    9.81);
myPlanner.setWeights(weight_R, weight_Q);

zmpLimX.resize(horizon_size);
zmpLimY.resize(horizon_size);


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
initial_feet_y[LF] = 1.0;
initial_feet_y[RF] = -1.0;
initial_feet_y[LH] = 1.0;
initial_feet_y[RH] = -1.0;

//first time solve the optim in the steps
computeSteps(initial_feet_x, initial_feet_y, distance, number_of_steps, horizon_size, feetStates, footHolds, A, b, myPlanner, schedule);
myPlanner.solveQPConstraintCoupled(height,initial_state_x, initial_state_y , A,b,jerk_x,jerk_y);
Vector3d  actual_state_x,actual_state_y;

int horizonWindow = horizon_size/number_of_steps; //after one 4stance and one 3 stance replan using the actual_swing, and actual foot pos and and actual com

//integrate equation for the replanning horizon

for (int sample= 0; sample<horizonWindow; sample++)
{
    actual_state_x = myPlanner.computeCOMtrajectory(initial_state_x, jerk_x.segment(0,sample+1)); //size cannot be lower than 1
    actual_state_y = myPlanner.computeCOMtrajectory(initial_state_y, jerk_y.segment(0,sample+1)); //size cannot be lower than 1
}

//computeSteps(initial_feet_x, initial_feet_y, distance, number_of_steps, horizon_size, feetStates, footHolds, A, b, myPlanner);
//myPlanner.solveQPConstraintCoupled(height,initial_state_x, initial_state_y , A,b,jerk_x,jerk_y);




//old stuff
//computeSteps(initial_feet_x, initial_feet_y, distance, number_of_steps, horizon_size, feetStates, footHolds, A, b, myPlanner);
//myPlanner.solveQPConstraintCoupled(height,initial_state_x, initial_state_y , A,b,jerk_x,jerk_y);
//viol = myPlanner.getConstraintViolation(feetStates);
prt(jerk_x.transpose())
prt(jerk_y.transpose())


myPlanner.computeZMPtrajectory( initial_state_x, jerk_x, zmp_x);
myPlanner.computeZMPtrajectory( initial_state_y, jerk_y, zmp_y);
myPlanner.computeCOMtrajectory( initial_state_x, jerk_x, com_x);
myPlanner.computeCOMtrajectory( initial_state_y, jerk_y, com_y);

VectorXd com_xd, com_yd;
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

myPlanner.saveTraj("footPosLFx.txt",  feetStates[LF].x);
myPlanner.saveTraj("footPosRFx.txt",  feetStates[RF].x);
myPlanner.saveTraj("footPosLHx.txt",  feetStates[LH].x);
myPlanner.saveTraj("footPosRHx.txt",  feetStates[RH].x);
myPlanner.saveTraj("footPosLFy.txt",  feetStates[LF].y);
myPlanner.saveTraj("footPosRFy.txt",  feetStates[RF].y);
myPlanner.saveTraj("footPosLHy.txt",  feetStates[LH].y);
myPlanner.saveTraj("footPosRHy.txt",  feetStates[RH].y);

myPlanner.saveTraj("footHoldsLF.txt",  footHolds[LF].x, footHolds[LF].y, 2*number_of_steps);
myPlanner.saveTraj("footHoldsRF.txt",  footHolds[RF].x, footHolds[RF].y, 2*number_of_steps);
myPlanner.saveTraj("footHoldsLH.txt",  footHolds[LH].x, footHolds[LH].y, 2*number_of_steps);
myPlanner.saveTraj("footHoldsRH.txt",  footHolds[RH].x, footHolds[RH].y, 2*number_of_steps);

myPlanner.saveTraj("swingLF.txt",  feetStates[LF].swing);
myPlanner.saveTraj("swingRF.txt",  feetStates[RF].swing);
myPlanner.saveTraj("swingLH.txt",  feetStates[LH].swing);
myPlanner.saveTraj("swingRH.txt",  feetStates[RH].swing);
//prt(zmpLimX.min.transpose())
//prt(zmpLimX.max.transpose())
}

void computeSteps(const LegDataMap<double> & initial_feet_x, const LegDataMap<double> & initial_feet_y,
                  double distance, const int number_of_steps, const int horizon_size,
                  LegDataMap<MPCPlanner::footState> & feetStates, LegDataMap<MPCPlanner::footState> & footHolds,
                  MatrixXd & A,  VectorXd & b, MPCPlanner & myPlanner, FootScheduler & schedule)
{


    int start_phase_index,  phase_duration, number_of_constraints;
    double distance_per_step = distance/number_of_steps;
    int step_knots = floor(horizon_size/number_of_steps);
    LegDataMap<double> feetValuesX, feetValuesY;

    feetValuesX = initial_feet_x;
    feetValuesY = initial_feet_y;

    //init stuff
    number_of_constraints = 0;
    start_phase_index = 0;
    phase_duration = step_knots/2; //10 samples both swing and phase
    A.resize((4+4)*phase_duration*number_of_steps, horizon_size*2); //assumes all stance phases then we resize
    b.resize((4+4)*phase_duration*number_of_steps);
    A.setZero();
    b.setZero();

    for (int leg=0;leg<4;leg++){
        feetStates[leg].resize(horizon_size);
        //set always stances
        feetStates[leg].swing.setConstant(horizon_size,false);
        footHolds[leg].resize(2*number_of_steps);
    }


    prt(phase_duration)
    for (int i=0; i<number_of_steps;i++)
    {
        //4 stance///////////////////////////
        feetStates[LF].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[LF]);
        feetStates[RF].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[RF]);
        feetStates[LH].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[LH]);
        feetStates[RH].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[RH]);
        feetStates[LF].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[LF]);
        feetStates[RF].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[RF]);
        feetStates[LH].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[LH]);
        feetStates[RH].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[RH]);
        //save footholds
        footHolds[LF].x(2*i) = feetValuesX[LF]; footHolds[LF].y(2*i) = feetValuesY[LF];
        footHolds[RF].x(2*i) = feetValuesX[RF]; footHolds[RF].y(2*i) = feetValuesY[RF];
        footHolds[LH].x(2*i) = feetValuesX[LH]; footHolds[LH].y(2*i) = feetValuesY[LH];
        footHolds[RH].x(2*i) = feetValuesX[RH]; footHolds[RH].y(2*i) = feetValuesY[RH];
        //build inequalities with the set of stance feet and positions
        myPlanner.buildPolygonMatrix(feetStates, start_phase_index,phase_duration, horizon_size, A,  b,  number_of_constraints );
        start_phase_index += phase_duration;

        //3 stance/////////////////////////////////////
        //step
        feetValuesX[schedule.getCurrentSwing()]+= distance_per_step;
        feetValuesY[schedule.getCurrentSwing()]+= 0.2;
        //set swing for that leg
        feetStates[schedule.getCurrentSwing()].swing.segment(start_phase_index, phase_duration).setConstant(true);
        feetStates[LF].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[LF]);
        feetStates[RF].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[RF]);
        feetStates[LH].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[LH]);
        feetStates[RH].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[RH]);
        feetStates[LF].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[LF]);
        feetStates[RF].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[RF]);
        feetStates[LH].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[LH]);
        feetStates[RH].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[RH]);
        //save footholds
        footHolds[LF].x(2*i + 1) = feetValuesX[LF]; footHolds[LF].y(2*i +1) = feetValuesY[LF];
        footHolds[RF].x(2*i + 1) = feetValuesX[RF]; footHolds[RF].y(2*i +1) = feetValuesY[RF];
        footHolds[LH].x(2*i + 1) = feetValuesX[LH]; footHolds[LH].y(2*i +1) = feetValuesY[LH];
        footHolds[RH].x(2*i + 1) = feetValuesX[RH]; footHolds[RH].y(2*i +1) = feetValuesY[RH];
    //    //build inequalities with the set of stance feet and positions
        myPlanner.buildPolygonMatrix(feetStates, start_phase_index,phase_duration,horizon_size, A,  b,  number_of_constraints );
        start_phase_index += phase_duration;
        schedule.next(); //update the step in the schedule
    }
    //compute missing knots last phase is double stance
    int missing_knots = horizon_size - start_phase_index;
    //end with 4 stance
    feetStates[LF].x.segment(start_phase_index, missing_knots).setConstant(feetValuesX[LF]);
    feetStates[RF].x.segment(start_phase_index, missing_knots).setConstant(feetValuesX[RF]);
    feetStates[LH].x.segment(start_phase_index, missing_knots).setConstant(feetValuesX[LH]);
    feetStates[RH].x.segment(start_phase_index, missing_knots).setConstant(feetValuesX[RH]);
    feetStates[LF].y.segment(start_phase_index, missing_knots).setConstant(feetValuesY[LF]);
    feetStates[RF].y.segment(start_phase_index, missing_knots).setConstant(feetValuesY[RF]);
    feetStates[LH].y.segment(start_phase_index, missing_knots).setConstant(feetValuesY[LH]);
    feetStates[RH].y.segment(start_phase_index, missing_knots).setConstant(feetValuesY[RH]);

    myPlanner.buildPolygonMatrix(feetStates, start_phase_index,missing_knots,horizon_size, A,  b,  number_of_constraints);
    //cause you have 3 stances
    A.conservativeResize(number_of_constraints,horizon_size*2);
    b.conservativeResize(number_of_constraints);
    //prt(missing_knots)
}




