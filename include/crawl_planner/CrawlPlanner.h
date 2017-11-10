/*
 * CrawlPlanner.h
 *
 *  Created on: Nov 9, 2017
 *      Author: mfocchi
 */

#ifndef CRAWLPLANNER_H_
#define CRAWLPLANNER_H_

//External Libraries
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <string>
#include <math.h>

//Includes
#include <iit/robots/hyq/default_parameters_getter.h>
#include <iit/robots/hyq/jacobians.h>
#include <iit/robots/hyq2max/default_parameters_getter.h>
#include <iit/robots/hyq2max/jacobians.h>
#include <iit/robots/hyq/transforms.h>
#include <iit/rbd/utils.h>

//IK
#include <hyq2max_ik/ik.h>
#include <hyq2max_ik/RobotLengths.h>
#include <hyq_ik/ik.h>

#include <dls_planner/Planner.h>
#include <dls_planner/PlannerBase.h>
#include <pluginlib/class_list_macros.h>


//crawl
#include <terrain_estimator/CTerrainEstimator.h>
#include <crawl_controller/FootScheduler.h>
#include <crawl_controller/task_globals.h>
#include <crawl_controller/StepHandler.h>
#include <crawl_controller/BodyTargetHandler.h>
#include <crawl_controller/TrunkController.h>
#include <crawl_controller/TrunkIdentification.h>
#include <crawl_controller/SlipRecovery.h>




namespace dls_planner
{

class CrawlPlanner : public dls_planner::PlannerBase
{
public:
    /** @brief Constructor function */
    CrawlPlanner();

    /** @brief Destructor function */
    ~CrawlPlanner();


    /** @brief Initialization procedure of the planner */
    bool init();

    /** @brief Starting procedure of the planner, we have access of the
     * actual whole-body state at this phase
     */
    void starting(double time);

    /** @brief Running procedure of the planner */
    void run(double time,
             double period);

    /** @brief Stopping procedure of the planner */
    void kill();


protected:



    dwl::WholeBodyState planned_ws_;

    /** @brief Number of joints */
    unsigned int num_joints_;

    /** @brief Joint id mapping between urdf to robcogen */
    std::vector<unsigned int> joint_id_map_;


    //Joint state
    dog::JointState des_tau_;
    dog::JointState des_q_;
    dog::JointState des_qd_;
    dog::JointState des_qdd_;
    dog::JointState q_;
    dog::JointState qd_;
    dog::JointState qdd_;
    dog::JointState tau_;


    dog::LegDataMap<rbd::Vector3d> jointDesPos;
    dog::LegDataMap<rbd::Vector3d> jointDesVel;
    dog::LegDataMap<rbd::Vector3d> jointDesAcc;

    ///IK
    RobotLengths rl;
    std::shared_ptr<InverseKinematicsHyQ2Max> hyq2maxIK;
    std::shared_ptr<InverseKinematicsHyQ>     hyqIK;

    //crawl
    bool update_base_position(double time);
    bool update_load_force(dog::LegID swing_leg_index, double time);
    void update_leg_correction(dog::LegID swing_leg_index, double time);
    bool update_swing_position(dog::LegID swing_leg_index, double time);
    void update_phase_duration(double cycle_time);
    void start_crawl();

    void crawlStateMachine(double time);

    void computeTerrainEstimation();
    void printCharOptions();
    //void changeCrawlParams(); TODO
    //void stop_crawl();

    //objects
    TaskGlobals gl;
    //std::shared_ptr<FastCrawler> fastCrawler;
    std::shared_ptr<StepHandler>  stepHandler;
    std::shared_ptr<BodyTargetHandler> bodyTargetHandler;
    std::shared_ptr<SlipRecovery > slipRecovery;
    std::shared_ptr<BaseState > bs;
    iit::ROBOT::WholeBodyOptimization::ConstrViolation constr_viol;
    CTerrainEstimator terrainEstimator;

    //Flags
    bool stopping_crawl = false;
    bool Terrain_Estimation;
    bool hapticCrawl = true;
    bool roughTerrainFlag = false;

    //crawl variables
    dog::LegID swing_leg_index;
    enum the_states {idle,
        move_base_to_next_support_triangle,
        unloadleg,
        swingleg,
        loadleg
    };
    the_states state_machine;
    dog::LegDataMap<the_states> state_machine_leg;
    the_states state_machineRF;
    the_states state_machineLH;
    the_states state_machineRH;
    Vector3d BaryTriangleW;
    double force_th = 0;
    double linearSpeedX;     double linearSpeedX_dsp;
    double linearSpeedY;     double linearSpeedY_dsp;
    double headingSpeed;     double headingSpeed_dsp;
    double linearSpeedXmax = 0.07;
    double headingSpeedmax = 0.07;
    double linearSpeedYmax = 0.07;
    double step_height_max = 0.2;
    double KdLX_posture_max = 800;
    double KdLY_posture_max = 800;
    double KdLZ_posture_max = 800;

    double initial_height = 0.0;

    double step_height = 0.1;
    int step_count = 0; //count steps!
    dog::LegDataMap<Eigen::Vector3d> sample_footPosDes;
    double step_x, step_y;

    //spliners
    iit::commons::FifthOrderPolySpliner forceLimSpliner;
    commons::FifthOrderPolySpliner::Point force_start, force_end, force_intermediate;
    dog::LegDataMap<iit::Point3d> feet_intermediate;
    dog::LegDataMap<ROBOT::FootSpliner> footSpliner;
    Timer baseTimer, forceTimer, swingTimer;
    std::shared_ptr<FootScheduler>  mySchedule;

    //timers
    double base_motion_duration;
    double swing_motion_duration;
    double force_motion_duration;
    double stab_margin = 0.08;
    double stab_margin_max = 0.2;

    //tracking errors integrators
    Timer intErrorTimer;
    double err_lin_X, err_lin_Y, err_roll, err_pitch;

};



}

PLUGINLIB_EXPORT_CLASS(dls_planner::CrawlPlanner, dls_planner::PlannerBase)

#endif /* CRAWLPLANNER_H_ */
