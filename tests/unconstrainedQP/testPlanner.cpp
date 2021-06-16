/*
 * test.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: mfocchi
 */



#include <crawl_planner/MPCPlanner.h>
#include <crawl_planner/ConsoleUtility.h>

using namespace Eigen;
using namespace std;
#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;

int main()
{

//init params
int horizon_size = 100;//10 default for tests
double height = 0.5;
double Ts = 0.1;
double weight_R = 1e-06;
double weight_Q = 1;

VectorXd  jerk;
Vector3d  initial_state = Vector3d(0.2,-0.1,0.0);
VectorXd zmp_x, zmp_y, com_x, com_y;

//get user input
newline::getInt("horizon_size:", horizon_size, horizon_size);
newline::getDouble("weight R:", weight_R, weight_R);
newline::getDouble("weight Q:", weight_Q, weight_Q);
newline::getDouble("initial state pos:", initial_state(0), initial_state(0));
newline::getDouble("initial state vel:", initial_state(1), initial_state(1));
newline::getDouble("initial state acc:", initial_state(2), initial_state(2));


MPCPlanner myPlanner(horizon_size,    Ts,    9.81);
myPlanner.setWeights(weight_R, weight_Q);
//myPlanner.debug();

/*
//test input

jerk.resize(horizon_size);
//jerk.setConstant(2.0);
jerk<< 1 , 2, 3,4,1 ,1,1,1,1,1;
//
myPlanner.computeZMPtrajectory(initial_state, jerk, zmp_x);
prt(zmp_x.transpose())
myPlanner.computeCOMtrajectory(initial_state, jerk, com_x, com_y,MPCPlanner::POSITION);
prt(com_x.transpose())
*/
//from matlab
//0.08507      0.07114     0.059209     0.050279     0.045349     0.045419     0.051489     0.064559     0.085628       0.1157

//set a linear traj for the ZmPref
VectorXd zmpRef;
zmpRef.resize(horizon_size);
zmpRef.setLinSpaced(horizon_size,0,0.2);
//prt(zmpRef.transpose())
myPlanner.solveQP(height, initial_state , zmpRef,jerk);
//prt(jerk.transpose())
myPlanner.computeZMPtrajectory(initial_state, jerk, zmp_x);
myPlanner.computeCOMtrajectory(initial_state, jerk, com_x, MPCPlanner::POSITION);

//prt(zmp_x.transpose())
myPlanner.saveTraj("zmpRef.txt", zmpRef);
myPlanner.saveTraj("jerk.txt", jerk);
myPlanner.saveTraj("zmp.txt", zmp_x);
myPlanner.saveTraj("com.txt", com_x);

}
