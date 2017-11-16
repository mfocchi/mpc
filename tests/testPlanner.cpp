/*
 * test.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: mfocchi
 */



#include <crawl_planner/MPCPlanner.h>

using namespace Eigen;
#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;

int main()
{

//init params
int horizon_size = 10;
double height = 0.5;
double Ts = 0.1;
Vector3d  initial_state = Vector3d(0.1,-0.1,0.0);

VectorXd zmp_x, zmp_y;
MPCPlanner myPlanner(horizon_size,    Ts,    9.81);

//myPlanner.debug();
myPlanner.buildZMPMatrix(height);

//input
VectorXd  jerk;
jerk.resize(horizon_size);
//jerk.setConstant(2.0);
jerk<< 1 , 2, 3,4,1 ,1,1,1,1,1;

myPlanner.computeZMPtrajectory(initial_state, jerk, zmp_x, zmp_y);
prt(zmp_x.transpose())

//myPlanner.saveTraj("zmpx.txt", zmp_x);
//from matlab
//0.08507      0.07114     0.059209     0.050279     0.045349     0.045419     0.051489     0.064559     0.085628       0.1157

//set a linear traj for the ZmPref
//VectorXd zmpRef;
//zmpRef.resize(horizon_size);
//
//zmpRef.setLinSpaced(horizon_size,0,0.1);
//prt(zmpRef.transpose())
//myPlanner.setWeights(1e-06, 1.0);
//myPlanner.solveQP(initial_state , zmpRef,jerk);
//prt(jerk.transpose())
//
//myPlanner.computeZMPtrajectory(initial_state, jerk, zmp_x, zmp_y);
//prt(zmp_x.transpose())


}
