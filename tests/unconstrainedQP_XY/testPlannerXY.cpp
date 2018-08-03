/*
 * test.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: mfocchi
 */



#include <crawl_planner/MPCPlanner.h>
#include <dls_controller/support/ConsoleUtility.h>

using namespace Eigen;
using namespace std;
#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;

int main()
{

//init params
int horizon_size = 100;//10 default for tests
int number_of_steps = 5;
double distance = 1.0;
double distance_per_step = floor(horizon_size/number_of_steps);
double lateral_sway = 0.5;
int step_knots = floor(horizon_size/number_of_steps);
double height = 0.5;
double Ts = 0.1;
double weight_R = 1e-06;
double weight_Q = 1;

VectorXd  jerk_x, jerk_y;
Vector3d  initial_state_x = Vector3d(0.0,-0.0,0.0);
Vector3d  initial_state_y = Vector3d(0.0,0.0,0.0);
VectorXd zmp_x, zmp_y, com_x, com_y;


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

VectorXd zmpRef_x, zmpRef_y, stepline;
//set zmprefx X
zmpRef_x.resize(horizon_size);
zmpRef_x.setLinSpaced(horizon_size,0,distance);

//set a piecewise traj for the ZmPref Y
zmpRef_y.resize(horizon_size);
stepline.setLinSpaced(step_knots/2,0,lateral_sway);//take first half  step to left
zmpRef_y.segment(0,step_knots/2) = stepline;
int step_index =step_knots/2;
bool goleft = false;
for (int i=0; i<number_of_steps-1;i++)
{
    if (goleft)
    {
        stepline.setLinSpaced(step_knots,-lateral_sway,lateral_sway);
    } else{
        stepline.setLinSpaced(step_knots,lateral_sway,-lateral_sway);
    }
    zmpRef_y.segment(step_index,step_knots) = stepline;
    goleft = !goleft;
    step_index += step_knots;
}
//compute missing knots (last step can be faster)
double missing_knots = horizon_size - step_index;
//set the last half motion step
if (goleft)
{
    stepline.setLinSpaced(missing_knots,-lateral_sway,0);
} else{
    stepline.setLinSpaced(missing_knots,lateral_sway,0);
}
zmpRef_y.segment(step_index,missing_knots) = stepline;

//solve the QPs independently
myPlanner.solveQP(height, initial_state_x , zmpRef_x,jerk_x);
myPlanner.solveQP(height, initial_state_y , zmpRef_y,jerk_y);

myPlanner.computeZMPtrajectory(initial_state_x, initial_state_y, jerk_x,jerk_y, zmp_x, zmp_y);
myPlanner.computeCOMtrajectory(initial_state_x, initial_state_y, jerk_x, jerk_y, com_x, com_y);

//prt(zmp_x.transpose())
myPlanner.saveTraj("zmpRef_x.txt", zmpRef_x);
myPlanner.saveTraj("zmpRef_y.txt", zmpRef_y);
myPlanner.saveTraj("jerk_x.txt", jerk_x);
myPlanner.saveTraj("jerk_y.txt", jerk_y);
myPlanner.saveTraj("zmp_x.txt", zmp_x);
myPlanner.saveTraj("zmp_y.txt", zmp_y);
myPlanner.saveTraj("com_x.txt", com_x);
myPlanner.saveTraj("com_y.txt", com_y);



}
