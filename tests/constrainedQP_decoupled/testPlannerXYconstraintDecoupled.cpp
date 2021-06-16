/*
 * test.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: mfocchi
 */



#include <crawl_planner/MPCPlanner.h>
#include <crawl_planner/ConsoleUtility.h>
#include <crawl_controller/FootScheduler.h>

using namespace Eigen;
using namespace std;
using namespace iit::dog;
#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;



int main()
{
//init params
int horizon_size = 100;//10 default for tests
int number_of_steps = 10;
double distance = 3.0;

double height = 0.5;
double lateral_bound = 0.2;
double Ts = 0.1;
double weight_R = 1; //jerk
double weight_Q = 1; //not used
double weight_Qs = 1; //slacks
int start_phase_index;
int phase_duration;
int useSlacks = false;

VectorXd  jerk_x, jerk_y;
Vector3d  initial_state_x = Vector3d(0.0, 0.0,0.0);
Vector3d  initial_state_y = Vector3d(0.0,-0.0,0.0);
VectorXd zmp_x, zmp_y, com_x, com_y;
MPCPlanner::BoxLimits zmpLimX, zmpLimY;


//get user input
newline::getInt("horizon_size:", horizon_size, horizon_size);
newline::getInt("number_of_steps:", number_of_steps, number_of_steps);
newline::getDouble("weight R:", weight_R, weight_R);
newline::getDouble("weight Q:", weight_Q, weight_Q);
newline::getDouble("initial state pos:", initial_state_x(0), initial_state_x(0));
newline::getDouble("initial state vel:", initial_state_x(1), initial_state_x(1));
newline::getDouble("initial state acc:", initial_state_x(2), initial_state_x(2));
newline::getInt("use slacks?[0/1]:", useSlacks, useSlacks);

double distance_per_step = distance/number_of_steps;
int step_knots = floor(horizon_size/number_of_steps);

MPCPlanner myPlanner(horizon_size,    Ts,    9.81);
myPlanner.setWeights(weight_R, weight_Q);

//compute polygons
zmpLimX.resize(horizon_size);
zmpLimY.resize(horizon_size);


//DECOUPLED
//iteratilely double stance/single stance
start_phase_index = 0;
phase_duration = step_knots/2;
double phase_lower_x_limit = 0.0;
bool goleft = true;
double phase_overlap = 0.1;
for (int i=0; i<number_of_steps-1;i++)
{
    //double stance
    zmpLimX.max.segment(start_phase_index, phase_duration).setConstant(phase_lower_x_limit + distance_per_step*(0.5 + phase_overlap));
    zmpLimX.min.segment(start_phase_index, phase_duration).setConstant(phase_lower_x_limit - distance_per_step*phase_overlap);
    zmpLimY.max.segment(start_phase_index, phase_duration).setConstant(1.0);
    zmpLimY.min.segment(start_phase_index, phase_duration).setConstant(-1.0);

    //single stance
    start_phase_index += phase_duration;
    phase_lower_x_limit += distance_per_step/2;
    zmpLimX.max.segment(start_phase_index, phase_duration).setConstant(phase_lower_x_limit + distance_per_step/2);
    zmpLimX.min.segment(start_phase_index, phase_duration).setConstant(phase_lower_x_limit);

    if (goleft)
    {
        zmpLimY.max.segment(start_phase_index, phase_duration).setConstant(1.0);
        zmpLimY.min.segment(start_phase_index, phase_duration).setConstant(lateral_bound);
    }else{
        zmpLimY.max.segment(start_phase_index, phase_duration).setConstant(-lateral_bound);
        zmpLimY.min.segment(start_phase_index, phase_duration).setConstant(-1.0);
    }
    goleft = !goleft;
    //update
    start_phase_index += phase_duration;
    phase_lower_x_limit += distance_per_step/2;
}

//compute missing knots last phase is double stance
double missing_knots = horizon_size - start_phase_index;
//double stance
zmpLimX.max.segment(start_phase_index, missing_knots).setConstant(distance);
zmpLimX.min.segment(start_phase_index, missing_knots).setConstant(phase_lower_x_limit - distance_per_step*phase_overlap);
zmpLimY.max.segment(start_phase_index, missing_knots).setConstant(1.0);
zmpLimY.min.segment(start_phase_index, missing_knots).setConstant(-1.0);

//simple test
//horizon_size = 3*10;
//myPlanner.setHorizonSize(horizon_size);
//initial_state_y = Vector3d(0.0,0.1,0.0);
//zmpLimY.max.resize(horizon_size);
//zmpLimY.min.resize(horizon_size);
//zmpLimY.max.segment(0,10).setConstant(1.0);
//zmpLimY.min.segment(0,10).setConstant(0.0);
//zmpLimY.max.segment(10,10).setConstant(1.0);
//zmpLimY.min.segment(10,10).setConstant(-1.0);
//zmpLimY.max.segment(20,10).setConstant(-0.5);
//zmpLimY.min.segment(20,10).setConstant(-1.0);
//myPlanner.solveQPconstraint(height, initial_state_y , zmpLimY,jerk_y);
//myPlanner.computeZMPtrajectory( initial_state_y, jerk_y, zmp_y);


if (!useSlacks)
{
    ////solve the QPs independently (not robust)
    myPlanner.solveQPconstraint(height, initial_state_x , zmpLimX,jerk_x);
    myPlanner.solveQPconstraint(height, initial_state_y , zmpLimY,jerk_y);
} else {
    //to add robustness use slacks! will keep the zmp in the middle of the limits
    weight_R = 1e-06; //jerk
    weight_Qs = 1; //slacks should be higher than R!
    myPlanner.setWeights(weight_R, weight_Q, weight_Qs);
    myPlanner.solveQPconstraintSlack(height, initial_state_x , zmpLimX,jerk_x);
    myPlanner.solveQPconstraintSlack(height, initial_state_y , zmpLimY,jerk_y);
}

prt(jerk_x.transpose())
prt(jerk_y.transpose())
//
myPlanner.computeZMPtrajectory( initial_state_x, jerk_x, zmp_x);
myPlanner.computeZMPtrajectory( initial_state_y, jerk_y, zmp_y);
myPlanner.computeCOMtrajectory( initial_state_x, jerk_x, com_x);
myPlanner.computeCOMtrajectory( initial_state_y, jerk_y, com_y);

////prt(zmp_x.transpose())
//
myPlanner.saveTraj("jerk_x.txt", jerk_x);
myPlanner.saveTraj("jerk_y.txt", jerk_y);
myPlanner.saveTraj("zmp_x.txt", zmp_x);
myPlanner.saveTraj("zmp_y.txt", zmp_y);
myPlanner.saveTraj("com_x.txt", com_x);
myPlanner.saveTraj("com_y.txt", com_y);
myPlanner.saveTraj("min_x.txt", zmpLimX.min);
myPlanner.saveTraj("max_x.txt", zmpLimX.max);
myPlanner.saveTraj("min_y.txt", zmpLimY.min);
myPlanner.saveTraj("max_y.txt", zmpLimY.max);

prt(zmpLimX.min.transpose())
prt(zmpLimX.max.transpose())
}
