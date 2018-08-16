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
    int experiment_duration = 60;

    double height = 0.5;
    double Ts = 0.1;
    double weight_R = 1e-06;
    double weight_Q = 1;

    VectorXd  jerk_x, jerk_y;
    Vector3d  initial_state_x = Vector3d(0.0, 0.0,0.0);
    Vector3d  initial_state_y = Vector3d(0.0,-0.0,0.0);
    VectorXd zmp_x, zmp_y, com_x, com_y, viol;
    MatrixXd A; VectorXd b;

    FootScheduler replanning_schedule; replanning_schedule.setSequence(LF, RH,RF,LH);
    int optimizeVelocityFlag = true;
    int useComStepCorrection = true;
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
    newline::getInt("experiment_duration :", experiment_duration, experiment_duration);
    newline::getInt("use com step correction? [0/1]:", useComStepCorrection, useComStepCorrection);


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

    //matlab file plotTrajXYconstraintCoupledMPCreplanning
    Vector3d  actual_state_x,actual_state_x1,actual_state_y;

    int replanningWindow = horizon_size/number_of_steps; //after one 4stance and one 3 stance replan using the actual_swing, and actual foot pos and and actual com
    prt(replanningWindow)
    int sample, sampleW = 0, replanningStage = 0;
    VectorXd jerk_disturbance; jerk_disturbance.resize(experiment_duration); jerk_disturbance.setZero();

    //init the state
    actual_state_x = initial_state_x;
    actual_state_y = initial_state_y;
    //apply disturbance
    jerk_disturbance.segment(4, experiment_duration-4 ).setConstant(disturbance);
    myPlanner.saveTraj("./replan_data/jerk_disturbance",  jerk_disturbance,  false);

    //loop
    for (int sample = 0; sample<experiment_duration;sample++) //replanning iterations
    {


        if ((sample % replanningWindow) == 0){//do the replan
            replanningStage++;
            std::cout<<"----------------------------------------------------------------------"<<std::endl;
            prt(replanningStage)

            if (sample>0)//when sample =0 the first time just use the initial value
            {
                for (int leg = 0; leg<4;leg++)
                {
                    //update feet with the actual stance
                    initial_feet_x[leg] = feetStates[leg].x(sampleW);
                    initial_feet_y[leg] = feetStates[leg].y(sampleW);
                    //old

                    //                if (feetStates[leg].swing(sampleW))
                    //                {
                    //                    swing_leg_index = LegID(leg);
                    //                    prt(swing_leg_index)
                    //                }

                }
                replanning_schedule.next();
            }

            //find the swing leg in sampleW and update the schedule to that and do the step
            prt(initial_feet_x)
            myPlanner.printSwing(replanning_schedule.getCurrentSwing());
            //recompute the new steps from the actual step
            if (useComStepCorrection)
            {
                myPlanner.computeSteps(userSpeed,  initial_feet_x, initial_feet_y,
                                       number_of_steps, horizon_size, feetStates, footHolds, A, b, myPlanner,
                                       replanning_schedule.getCurrentSwing(), Vector2d(actual_state_x(0),actual_state_y(0)));
            } else {
                myPlanner.computeSteps(userSpeed,  initial_feet_x, initial_feet_y,
                                       number_of_steps, horizon_size, feetStates, footHolds, A, b, myPlanner,
                                       replanning_schedule.getCurrentSwing());
            }
            //replan from the actual state and the new steps overwriting the jerk
            if (!optimizeVelocityFlag)
                myPlanner.solveQPConstraintCoupled(height,actual_state_x, actual_state_y , A,b,jerk_x,jerk_y);
            else {
                weight_R = 0.01; myPlanner.setWeights(weight_R, weight_Q);
                myPlanner.solveQPConstraintCoupled(height,actual_state_x, actual_state_y , A,b, userSpeed, jerk_x,jerk_y, replanningWindow);
            }
            //reset the counter
            sampleW = 0;

            //prt(initial_feet_y)
            //save it
            //for the log compute the whole traj
            myPlanner.computeCOMtrajectory( actual_state_x, jerk_x, com_x);
            myPlanner.computeCOMtrajectory( actual_state_y, jerk_y, com_y);
            myPlanner.computeZMPtrajectory( actual_state_x, jerk_x, zmp_x);
            myPlanner.computeZMPtrajectory( actual_state_y, jerk_y, zmp_y);

            myPlanner.computeCOMtrajectory( actual_state_x, jerk_x, com_xd, MPCPlanner::VELOCITY);
            myPlanner.computeCOMtrajectory( actual_state_y, jerk_y, com_yd, MPCPlanner::VELOCITY);

            myPlanner.saveTraj("./replan_data/com_x"+to_string(replanningStage), com_x,false);
            myPlanner.saveTraj("./replan_data/com_y"+to_string(replanningStage), com_y,false);
            myPlanner.saveTraj("./replan_data/com_xd"+to_string(replanningStage), com_xd,false);
            myPlanner.saveTraj("./replan_data/com_yd"+to_string(replanningStage), com_yd,false);
            myPlanner.saveTraj("./replan_data/zmp_x"+to_string(replanningStage), zmp_x,false);
            myPlanner.saveTraj("./replan_data/zmp_y"+to_string(replanningStage), zmp_y,false);

            myPlanner.saveTraj("./replan_data/footHoldsLF"+to_string(replanningStage),  footHolds[LF].x, footHolds[LF].y,false);
            myPlanner.saveTraj("./replan_data/footHoldsRF"+to_string(replanningStage),  footHolds[RF].x, footHolds[RF].y,false);
            myPlanner.saveTraj("./replan_data/footHoldsLH"+to_string(replanningStage),  footHolds[LH].x, footHolds[LH].y,false);
            myPlanner.saveTraj("./replan_data/footHoldsRH"+to_string(replanningStage),  footHolds[RH].x, footHolds[RH].y,false);

            myPlanner.saveTraj("./replan_data/footPosLF"+to_string(replanningStage),  feetStates[LF].x, feetStates[LF].y,false);
            myPlanner.saveTraj("./replan_data/footPosRF"+to_string(replanningStage),  feetStates[RF].x, feetStates[RF].y,false);
            myPlanner.saveTraj("./replan_data/footPosLH"+to_string(replanningStage),  feetStates[LH].x, feetStates[LH].y,false);
            myPlanner.saveTraj("./replan_data/footPosRH"+to_string(replanningStage),  feetStates[RH].x, feetStates[RH].y,false);

            myPlanner.saveTraj("./replan_data/swingLF"+to_string(replanningStage),  feetStates[LF].swing,false);
            myPlanner.saveTraj("./replan_data/swingRF"+to_string(replanningStage),  feetStates[RF].swing,false);
            myPlanner.saveTraj("./replan_data/swingLH"+to_string(replanningStage),  feetStates[LH].swing,false);
            myPlanner.saveTraj("./replan_data/swingRH"+to_string(replanningStage),  feetStates[RH].swing,false);

            std::ofstream file;
            file.open("./replan_data/exp_data");
            file<<horizon_size <<" ";
            file<<number_of_steps <<" ";
            file<<experiment_duration <<std::endl;
            file.close();

        } else {
            sampleW++;
        }

        //integrate equation for the replanning horizon
        //actual_state_x = myPlanner.computeCOMtrajectory(initial_state_x, jerk_x.segment(0,sampleW+1)); //size cannot be lower than 1
        myPlanner.computeCOMupdate(actual_state_x, jerk_x(sampleW));
        myPlanner.computeCOMupdate(actual_state_y, jerk_y(sampleW) + jerk_disturbance(sample));

    }


}




