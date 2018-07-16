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
#include <iit/rbd/utils.h>


#include <dls_planner/Planner.h>
#include <dls_planner/PlannerBase.h>
#include <pluginlib/class_list_macros.h>

#include <iit/locomotionutils/TrunkController.h>

//crawl
#include <terrain_estimator/CTerrainEstimator.h>
#include <crawl_controller/FootScheduler.h>
#include <crawl_controller/task_globals.h>
#include <crawl_controller/StepHandler.h>
#include <crawl_controller/BodyTargetHandler.h>

//replanning
#include <crawl_planner/MPCPlanner.h>

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

    void setRobotModels(std::shared_ptr<iit::dog::FeetJacobians>& feet_jacs,
                                         std::shared_ptr<iit::dog::ForwardKinematics>& fwd_kin,
                                         std::shared_ptr<iit::dog::ShinJacobians>& shin_jacs,
                                         std::shared_ptr<iit::dog::InertiaPropertiesBase> &inertia_props,
                                         std::shared_ptr<iit::dog::HomogeneousTransformsBase> &hom_transforms,
                                         std::shared_ptr<iit::dog::JSIMBase> &jsim,
                                         std::shared_ptr<iit::dog::InverseDynamicsBase> &inv_dyn,
                                         std::shared_ptr<iit::dog::KinDynParams> &params,
                                         std::shared_ptr<iit::dog::LimitsBase> & limits,
                                         std::shared_ptr<iit::dog::FeetContactForces> & feet_forces);
protected:



    dwl::WholeBodyState planned_ws_;

    /** @brief Number of joints */
    unsigned int num_joints_;


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

    //crawl
    bool update_base_position(double time);
    bool update_load_force(dog::LegID swing_leg_index, double time);
    void update_leg_correction(dog::LegID swing_leg_index, double time);
    bool update_swing_position(dog::LegID swing_leg_index, double time);
    void update_phase_duration(double cycle_time);
    void start_crawl();

    void crawlStateMachine(double time);

    void addVarToCollect( double value, std::string var_name);
    void updateVarsForDataLogging();

    void computeTerrainEstimation();
    void printCharOptions();

    //replanning stuff
    void computeSteps(const Vector2d & userSpeed,
                      const iit::dog::LegDataMap<double> & initial_feet_x, const iit::dog::LegDataMap<double> & initial_feet_y,
                      const double distance, const int number_of_steps, const int horizon_size,
                      iit::dog::LegDataMap<MPCPlanner::footState> & feetStates,
                      iit::dog::LegDataMap<MPCPlanner::footState> & footHolds,
                      MatrixXd & A,  VectorXd & b,
                      MPCPlanner & myPlanner,
                      dog::LegID swing_leg_index,
                      Vector2d initialCoM = Vector2d::Zero());


    void computeSteps(const iit::dog::LegDataMap<double> & initial_feet_x, const iit::dog::LegDataMap<double> & initial_feet_y,
                      const double distance, const int number_of_steps, const int horizon_size,
                      iit::dog::LegDataMap<MPCPlanner::footState> & feetStates,
                      iit::dog::LegDataMap<MPCPlanner::footState> & footHolds,
                      MatrixXd & A,  VectorXd & b, MPCPlanner & myPlanner);


    //init params
    int horizon_size = 200;//10 default for tests
    int number_of_steps = 6;
    double distance = 3.0;
    int experiment_duration = 60;

    double lateral_bound = 0.2;
    double Ts = 0.1;
    double weight_R = 1e-06;
    double weight_Q = 1;

    VectorXd  jerk_x, jerk_y;
    Vector3d  initial_state_x = Vector3d(0.0, 0.0,0.0);
    Vector3d  initial_state_y = Vector3d(0.0,-0.0,0.0);
    VectorXd zmp_x, zmp_y, com_x, com_y, viol, com_xd, com_yd;
    iit::dog::LegDataMap<MPCPlanner::footState> feetStates;
    iit::dog::LegDataMap<MPCPlanner::footState> footHolds;
    iit::dog::LegDataMap<double> initial_feet_x;
    iit::dog::LegDataMap<double> initial_feet_y;
    MPCPlanner::BoxLimits zmpLimX, zmpLimY;
    MatrixXd A; VectorXd b;
    int replanningFlag = true;
    int optimizeVelocityFlag = true;
    int useComStepCorrection = true;
    double disturbance = 0.0;
    std::shared_ptr<MPCPlanner> myPlanner;


    //void changeCrawlParams(); TODO
    //void stop_crawl();

    //objects
    TaskGlobals gl;
    std::shared_ptr<StepHandler>  stepHandler;
    std::shared_ptr<BodyTargetHandler> bodyTargetHandler;
    std::shared_ptr<BaseState > bs;
    CTerrainEstimator terrainEstimator;


    //Flags
    bool stopping_crawl = false;
    bool Terrain_Estimation;
    bool hapticCrawl = true;
    bool roughTerrainFlag = false;

    double taskServoTime;
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
    double initial_height = 0.0;

    double step_height = 0.1;
    int step_count = 0; //count steps!
    dog::LegDataMap<Eigen::Vector3d> sample_footPosDes;
    double step_x, step_y;

    //spliners
    iit::commons::FifthOrderPolySpliner forceLimSpliner;
    commons::FifthOrderPolySpliner::Point force_start, force_end, force_intermediate;
    dog::LegDataMap<iit::planning::Point3d> feet_intermediate;
    dog::LegDataMap<dog::FootSpliner> footSpliner;
    Timer baseTimer, forceTimer, swingTimer;
    std::shared_ptr<FootScheduler>  mySchedule;
    iit::planning::Point3d des_com_pos, des_com_posB;
    //timers
    double base_motion_duration;
    double swing_motion_duration;
    double force_motion_duration;
    double stab_margin = 0.08;
    double stab_margin_max = 0.2;

};



}

PLUGINLIB_EXPORT_CLASS(dls_planner::CrawlPlanner, dls_planner::PlannerBase)

#endif /* CRAWLPLANNER_H_ */
