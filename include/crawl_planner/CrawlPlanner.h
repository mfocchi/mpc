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
#include <dls_planner/Planner.h>
#include <dls_planner/PlannerBase.h>
#include <pluginlib/class_list_macros.h>

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

    std::map<int, std::string> legmap;

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

    //Joint state
    dog::JointState des_tau_;
    dog::JointState des_q_;
    dog::JointState des_qd_;
    dog::JointState des_qdd_;
    dog::JointState q_;
    dog::JointState qd_;
    dog::JointState qdd_;
    dog::JointState tau_;

    //crawl functions
    bool update_base_position(double time);
    bool update_load_force(dog::LegID swing_leg_index, double time);
    bool update_swing_position(dog::LegID swing_leg_index, double time);
    bool check_touch_down();
    void update_phase_duration(double cycle_time);
    void start_crawl();
    void crawlStateMachine(double time);
    void addVarToCollect( double value, std::string var_name);
    void updateVarsForDataLogging();
    void computeTerrainEstimation();

    //replanning functions
    void debug();
    void toggleCoMCorrection();
    void toggleOptimizeVelocity();
    void start_replanning_crawl();
    void print_foot_holds();
    iit::dog::LegBoolMap detectLiftOff(iit::dog::LegDataMap<MPCPlanner::footState> feetStates, double actualSample);
    //TODO to remove
    void computeSteps(const Vector2d & userSpeed,
                      const iit::dog::LegDataMap<double> & initial_feet_x, const iit::dog::LegDataMap<double> & initial_feet_y,
                      const int number_of_steps, const int horizon_size,
                      iit::dog::LegDataMap<MPCPlanner::footState> & feetStates,
                      iit::dog::LegDataMap<MPCPlanner::footState> & footHolds,
                      MatrixXd & A,  VectorXd & b,
                      MPCPlanner & myPlanner,
                      dog::LegID swing_leg_index,
                      Vector2d initialCoM = Vector2d::Zero());
    Vector3d mapBToWF(Vector3d B_vec_in);
    Vector3d mapWFToB(Vector3d W_vec_in);
    void interactiveChangeParams(void);
    void resetPlanner();

    //init params for replanning stage
    double horizon_duration = 8.0;
    int horizon_size = 0.0;//10 default for tests
    int number_of_steps = 8;
    int replanningWindow = 0.0;
    int replanning_steps = 1;
    int sample = 0, sampleW = 0, replanningStage = 0;
    bool firstTime = true;
    //this is the time discretization of the trajectory
    double time_resolution = 0.04;
    double task_time_resolution;
    double weight_R = 1e-06;
    double weight_Q = 1;

    Eigen::VectorXd  jerk_x, jerk_y;
    Eigen::VectorXd  des_com_x, des_com_y;
    Eigen::VectorXd  des_com_xd, des_com_yd;
    iit::commons::LinearSpliner interpolateCoMPositionX;
    iit::commons::LinearSpliner interpolateCoMPositionY;
    iit::commons::LinearSpliner interpolateCoMVelocityX;
    iit::commons::LinearSpliner interpolateCoMVelocityY;

    Eigen::Vector3d  actual_state_x = Vector3d(0.0, 0.0,0.0);
    Eigen::Vector3d  actual_state_y = Vector3d(0.0,-0.0,0.0);
    Eigen::VectorXd zmp_x, zmp_y, com_x, com_y, viol, com_xd, com_yd;
    iit::dog::LegDataMap<MPCPlanner::footState> feetStates;
    iit::dog::LegDataMap<MPCPlanner::footState> footHolds;
    iit::dog::LegDataMap<double> initial_feet_x;
    iit::dog::LegDataMap<double> initial_feet_y;
    Eigen::MatrixXd A; VectorXd b;
    iit::dog::LegDataMap<Vector2d> hip_offsets;
    Vector3d footTarget, footTargetW;
    iit::dog::LegDataMap<int> touchDown = 0;
    double disturbance = 0.0;
    std::shared_ptr<MPCPlanner> myPlanner;

    //flags
    int replanningFlag = false;
    bool stoppingFlag = false;
    int optimizeVelocityFlag = true;
    int useComStepCorrection = true;
    iit::dog::LegDataMap<Eigen::Vector3d> dummy1;
    iit::dog::LegDataMap<Eigen::Vector3d> dummy2;
    //end of replanning variables

    //common objects
    TaskGlobals gl;
    std::shared_ptr<StepHandler>  stepHandler;
    std::shared_ptr<BodyTargetHandler> bodyTargetHandler;
    std::shared_ptr<BaseState > bs;
    CTerrainEstimator terrainEstimator;

    //haptic planner com state
    iit::planning::Point3d des_com_pos, des_com_posB;

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
    Vector3d BaryTriangleW;
    double force_th = 0;
    double linearSpeedX;
    double linearSpeedY;
    double headingSpeed;
    double linearSpeedXmax = 0.07;
    double headingSpeedmax = 0.07;
    double linearSpeedYmax = 0.07;
    double step_height_max = 0.2;
    double initial_height = 0.0;
    double step_height = 0.1;
    int step_count = 0; //count steps!
    double step_x, step_y;

    //spliners
    iit::commons::FifthOrderPolySpliner forceLimSpliner;
    commons::FifthOrderPolySpliner::Point  force_intermediate;
    dog::LegDataMap<dog::FootSpliner> footSpliner;
    Timer baseTimer, forceTimer, swingTimer;
    std::shared_ptr<FootScheduler>  mySchedule;

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
