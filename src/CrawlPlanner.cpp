#include <crawl_planner/CrawlPlanner.h>
#include <dls_controller/support/ConsoleUtility.h>

using namespace Eigen;
using namespace std;
using namespace iit::dog;

namespace dls_planner
{

CrawlPlanner::CrawlPlanner() {
    planner_name_ = "crawl_planner";
    //this is construct
    mySchedule.reset(new FootScheduler(iit::dog::RH, iit::dog::RF, iit::dog::LH, iit::dog::LF));
    stepHandler.reset(new StepHandler(gl));
    bodyTargetHandler.reset(new BodyTargetHandler);
    bs.reset(new BaseState());

    legmap[iit::dog::LF] = "LF";
    legmap[iit::dog::RF] = "RF";
    legmap[iit::dog::LH] = "LH";
    legmap[iit::dog::RH] = "RH";
}

CrawlPlanner::~CrawlPlanner()
{

}

bool CrawlPlanner::init()
{
    printf(BLUE "Initialization of the CrawlPlanner\n" COLOR_RESET);

    addConsoleFunction("startCrawl",
                       "crawl on with virtual model on",
                       &CrawlPlanner::start_crawl, this);

    addConsoleFunction("toggleReplanning",
                       "start/stop replanning crawl",
                       &CrawlPlanner::start_replanning_crawl, this);

    addConsoleFunction("debug",
                       "debug",
                       &CrawlPlanner::debug, this);

    addConsoleFunction("ictp",
                       "interactive change control parameters",
                       &CrawlPlanner::interactiveChangeParams, this);
    addConsoleFunction("comCorr",
                       "correct footsteps with com position",
                       &CrawlPlanner::toggleCoMCorrection, this);
    addConsoleFunction("velOpt",
                       "optimize user velocity at the end of the replanning window",
                       &CrawlPlanner::toggleOptimizeVelocity, this);

    //init crawl state machine
    state_machine = idle;
    unsigned int num_joints_ = fbs_->getJointDoF();
    planned_ws_.setJointDoF(num_joints_);


    // Declaring ROS clients
    internal_stance_clt_ = node_.serviceClient<std_srvs::Empty>("/internal_stance_service");

    printf(BLUE "Initialization of the CrawlPlanner accomplished\n" COLOR_RESET);

    return true;
}


void CrawlPlanner::starting(double time) {
    std::cout << "Starting CrawlPlanner controller at " << time << " sec.";
    std::cout << std::endl;
    taskServoTime = time;
    gl.param_file = ros::package::getPath(std::string("crawl_planner")) + "/config/planner.ini";

    //Converting WholeBodyState to RobCoGen
    for (unsigned int i = 0; i < fbs_->getJointDoF(); i++) {
        // Getting the joint id
        unsigned int joint_id = getDWLJointId(JointIdentifiers(i));
        // Converting the desired joint states from RobCoGen order
        q_(i) = actual_ws_->joint_pos(joint_id);
        qd_(i) = actual_ws_->joint_vel(joint_id);
        qdd_(i) = 0.;
        tau_(i) = actual_ws_->joint_eff(joint_id);
    }
    //fill in the base state object
    bs->setPosition_W(actual_ws_->getBasePosition());
    bs->setVelocity_W(actual_ws_->getBaseVelocity_W());
    bs->setRotationRate_B(actual_ws_->getBaseAngularVelocity_W());
    bs->setRotAcceleration_B(actual_ws_->getBaseAngularAcceleration_W());
    bs->setOrientation_W(actual_ws_->getBaseOrientation());

    //compute torque limits according to the actual config of the robot
    tau_max_ = robot_limits_->getTorqueLimits(q_);

    //init gl
    gl.init(q_, qd_, qdd_); //this sets also 	footPosDes = footPos;
    gl.sl_to_eigenLogic(q_, qd_, tau_, tau_max_, qdd_, *bs);
    //this is needed to compute qdd_ via numeric differentiation
    gl.setServoRate(planning_rate_);

    //init generic stuff
    //set relationship bw velocity and step
    roughTerrainFlag = gl.config_.get<bool>("Crawl.roughTerrain");
    step_height = gl.config_.get<double>("Crawl.step_height");
    linearSpeedX = gl.config_.get<double>("Crawl.linearSpeedX");
    linearSpeedY = gl.config_.get<double>("Crawl.linearSpeedY");
    headingSpeed = gl.config_.get<double>("Crawl.headingSpeed");
    hapticCrawl = gl.config_.get<bool>("Crawl.hapticCrawl");
    if (!hapticCrawl)
        std::cout<< "haptic crawl is not ON!" <<std::endl;

    //init base / joint variables
    gl.actual_base.x = actual_ws_->getBasePosition();
    //reset desired com linear position to actual readings
    gl.computeCoM(q_, qd_);
    des_com_pos.x = gl.actual_CoM.x;//com control
    des_com_pos.xd.setZero();
    //reset desired orientation
    gl.des_base_orient.x(rbd::X) = gl.roll;
    gl.des_base_orient.x(rbd::Y) = gl.pitch;
    gl.des_base_orient.x(rbd::Z) = gl.yaw;
    //set des height
    gl.des_height = gl.actual_CoM_height;
    initial_height = gl.actual_CoM_height;
    //set feet normals vertical
    gl.terr_normal = Vector3d(0,0,1.0);
    //set all stance legs to true
    gl.stance_legs = true;

    //init crawl specific stuff
    bodyTargetHandler->setBackwardMotion(true);
    bodyTargetHandler->setRoughTerrain(roughTerrainFlag);
    force_th = gl.config_.get<double>("Crawl.force_th");
    Terrain_Estimation  = gl.config_.get<bool>("Crawl.terrain_estimation");
    stepHandler->useTerrainEstimation(Terrain_Estimation);
    gl.cycle_time = gl.config_.get<double>("Crawl.cycle_time");
    update_phase_duration(gl.cycle_time);
    stepHandler->setSpeedParameters(0.2, 0.2, 0.2, 0.1, 0.1, 0.1, 0.022, 0.022, 0.022);
    stepHandler->setHipDefaultYOffset(0.06 + fabs((gl.footPos[iit::dog::LF] - stepHandler->getHipPosition(q_,iit::dog::LF))(rbd::Y)));
    stepHandler->setDefaultStepLength((linearSpeedX * gl.cycle_time));
    des_q_ = q_;
    des_qd_.setZero();

    //init replanning stuff
    task_time_resolution = 1.0/planning_rate_; //time resolution of the haptic planner
    resetPlanner();
    useComStepCorrection = gl.config_.get<bool>("Replanning.useComStepCorrection");
    optimizeVelocityFlag = gl.config_.get<bool>("Replanning.optimizeVelocity");

    des_com_x.resize(horizon_size); des_com_x.setZero();
    des_com_y.resize(horizon_size); des_com_x.setZero();
    des_com_xd.resize(horizon_size); des_com_xd.setZero();
    des_com_yd.resize(horizon_size); des_com_xd.setZero();
    zmp_x.resize(horizon_size); zmp_x.setZero();
    zmp_y.resize(horizon_size); zmp_y.setZero();

    //set initial state for com and feet (replanning variables)
    actual_state_x(0) = gl.actual_CoM.x(rbd::X);
    actual_state_x(1) = gl.actual_CoM.xd(rbd::X);
    actual_state_x(2) = 0.0;
    actual_state_y(0) = gl.actual_CoM.x(rbd::Y);
    actual_state_y(1) = gl.actual_CoM.xd(rbd::Y);
    actual_state_y(2) = 0.0;

    //set position of hip offsets (in base frame)
    hip_offsets[LF] << 0.33, 0.32;
    hip_offsets[RF] << 0.33, -0.32;
    hip_offsets[LH] << -0.33, 0.32;
    hip_offsets[RH] << -0.33, -0.32;

    //set initial position of the feet (in the world frame)
    initial_feet_x[LF] = (gl.actual_base.x + gl.Rt*gl.footPos[LF])(rbd::X);
    initial_feet_x[RF] = (gl.actual_base.x + gl.Rt*gl.footPos[RF])(rbd::X);
    initial_feet_x[LH] = (gl.actual_base.x + gl.Rt*gl.footPos[LH])(rbd::X);
    initial_feet_x[RH] = (gl.actual_base.x + gl.Rt*gl.footPos[RH])(rbd::X);
    initial_feet_y[LF] = (gl.actual_base.x + gl.Rt*gl.footPos[LF])(rbd::Y);
    initial_feet_y[RF] = (gl.actual_base.x + gl.Rt*gl.footPos[RF])(rbd::Y);
    initial_feet_y[LH] = (gl.actual_base.x + gl.Rt*gl.footPos[LH])(rbd::Y);
    initial_feet_y[RH] = (gl.actual_base.x + gl.Rt*gl.footPos[RH])(rbd::Y);
    //TODO maybe remove this
    for (int leg=0;leg<4;leg++){
        feetStates[leg].resize(horizon_size);
        footHolds[leg].resize(number_of_steps); //old 2*number_of_steps
    }
    dummy1= Vector3d::Zero();
    dummy2= Vector3d::Zero();


    std::cout << "Setting internal stance computation in trunk controller (Service Call)" << std::endl;
    std_srvs::Empty internal_stance_msgs;
    if (internal_stance_clt_.call(internal_stance_msgs)) {}
    else {
        ROS_ERROR("Failed to call service for internal stance computation in trunk controller");
    }
    printf("Crawl planner: finished starting\n");
}



void CrawlPlanner::run(double time,
                       double period) {
    taskServoTime = time;
    for (unsigned int i = 0; i < fbs_->getJointDoF(); i++) {
        // Getting the joint id
        unsigned int joint_id = getDWLJointId(JointIdentifiers(i));
        // Converting the desired joint states from RobCoGen order
        q_(i) = actual_ws_->joint_pos(joint_id);
        qd_(i) = actual_ws_->joint_vel(joint_id);
        qdd_(i) = 0.;
        tau_(i) = actual_ws_->joint_eff(joint_id);
    }
    //get grfs
    feet_forces_->getFeetGRF(q_,
                             qd_,
                             tau_,
                             bs->getOrientation_W(),
                             gl.grForces,
                             qdd_,
                             bs->getVelocity_B(),
                             Vector3d::Zero(),
                             bs->getRotationRate_B(),
                             Vector3d::Zero());
    //set log variables
    updateVarsForDataLogging();

    //get the base state
    bs->setPosition_W(actual_ws_->getBasePosition());
    bs->setVelocity_W(actual_ws_->getBaseVelocity_W());
    bs->setRotationRate_B(actual_ws_->getBaseAngularVelocity_W());
    bs->setRotAcceleration_B(actual_ws_->getBaseAngularAcceleration_W());
    bs->setOrientation_W(actual_ws_->getBaseOrientation());
    gl.sl_to_eigenLogic(q_, qd_, tau_,tau_max_, qdd_, *bs);

    //compute terrain estimation
    if (Terrain_Estimation){
        computeTerrainEstimation();
    }

    //normal  crawl (fills in base and joint variables)
    crawlStateMachine(taskServoTime);

    //crawl relanning
    if (replanningFlag)
    {
        //this time keeps track of the haptic planner,  but, differently from taskServoTime it stops if the optimization slows down the thread
        double sampleTime = sample*task_time_resolution;
        Vector2d userSpeed = Vector2d(linearSpeedX, linearSpeedY);

        //this is the optimization part (called only at each touchdown)
        if (firstTime ||
                (touchDown[LF] == replanning_steps) ||
                (touchDown[RF] == replanning_steps) ||
                (touchDown[LH] == replanning_steps) ||
                (touchDown[RH] == replanning_steps) )
        {
            replanningStage++;
            std::cout<<"replanningStage----------------------------------------------: "<<replanningStage<<std::endl;

            //set initial state for optimization to actual state (there might have been disturbances)
            if (!firstTime)
            {
                //set feet and com position to warm start the optimization
                //init feet (in the world frame):
                for (int leg = 0; leg<4;leg++)
                {
                    if (!hapticCrawl)
                    {
                        //use desired values
                        initial_feet_x[leg] = mapBToWF(gl.footPosDes[leg])(rbd::X);
                        initial_feet_y[leg] = mapBToWF(gl.footPosDes[leg])(rbd::Y);
                    } else {
                        //use actual values
                        initial_feet_x[leg] = mapBToWF(gl.footPos[leg])(rbd::X);
                        initial_feet_y[leg] = mapBToWF(gl.footPos[leg])(rbd::Y);
                    }
                }
                //init com:
                //use the actual state
                actual_state_x = Vector3d ( gl.actual_CoM.x(rbd::X),  gl.actual_CoM.xd(rbd::X), 0.0);
                actual_state_y = Vector3d ( gl.actual_CoM.x(rbd::Y),  gl.actual_CoM.xd(rbd::Y), 0.0);
                //reset touchdown for the current swing
                touchDown[mySchedule->getCurrentSwing()] = 0;
                //set the next swing leg in the schedule
                mySchedule->next();
            } else {
                firstTime = false;
                std::cout<<"first time dont update actual state and initial feet they arelady have been initialized"<<std::endl;
            }
            myPlanner->printSwing(mySchedule->getCurrentSwing()); //just print out
            hip_offsets[LF] = gl.Rt.block(0,0,2,2)*hip_offsets[LF]; //rotate to base frame
            hip_offsets[RF] = gl.Rt.block(0,0,2,2)*hip_offsets[RF];
            hip_offsets[LH] = gl.Rt.block(0,0,2,2)*hip_offsets[LH];
            hip_offsets[RH] = gl.Rt.block(0,0,2,2)*hip_offsets[RH];
            myPlanner->setHipOffsets(hip_offsets);

            //recompute the new foothold steps from the actual com (if useComStepCorrection) and foot position
            if (useComStepCorrection)
            {
                myPlanner->computeSteps(userSpeed,
                                        initial_feet_x,
                                        initial_feet_y,
                                        number_of_steps,
                                        horizon_size,
                                        feetStates, footHolds,
                                        A, b, *myPlanner,
                                        mySchedule->getCurrentSwing(),
                                        Vector2d(gl.actual_base.x(rbd::X),gl.actual_base.x(rbd::Y)));
                dummy1 =  myPlanner->getDummyVars(1);
                dummy2 =  myPlanner->getDummyVars(2);
            } else {
              myPlanner->computeSteps(userSpeed,
                                    initial_feet_x,
                                    initial_feet_y,
                                    number_of_steps,
                                    horizon_size,
                                    feetStates, footHolds,
                                    A, b, *myPlanner,
                                    mySchedule->getCurrentSwing());
            }
            //print_foot_holds();

            //replan from the actual state and the new set of footholds overwriting the vector of jerk
            if (!optimizeVelocityFlag){
                weight_R = 1e-06; weight_Q = 1; myPlanner->setWeights(weight_R, weight_Q);
                myPlanner->solveQPConstraintCoupled(gl.actual_CoM_height,actual_state_x, actual_state_y , A,b,jerk_x,jerk_y);
            }
            else {//tries to achieve the userSpeed at the end of the replanning window
                weight_R = 0.01; weight_Q = 1; myPlanner->setWeights(weight_R, weight_Q);
                myPlanner->solveQPConstraintCoupled(gl.actual_CoM_height,actual_state_x, actual_state_y , A,b,userSpeed,jerk_x,jerk_y,replanningWindow);
            }
            //for visualization purposes compute the whole trajectories from the jerk vector
            //com
            myPlanner->computeCOMtrajectory( actual_state_x, jerk_x, des_com_x);
            myPlanner->computeCOMtrajectory( actual_state_y, jerk_y, des_com_y);
            //come velocity
            myPlanner->computeCOMtrajectory( actual_state_x, jerk_x, des_com_xd, MPCPlanner::VELOCITY);
            myPlanner->computeCOMtrajectory( actual_state_y, jerk_y, des_com_yd, MPCPlanner::VELOCITY);
            //zmp
            myPlanner->computeZMPtrajectory( actual_state_x, jerk_x, zmp_x);
            myPlanner->computeZMPtrajectory( actual_state_y, jerk_y, zmp_y);

            //compute the constraint violation (e.g. how close I am o the bounds) for debugging reasons
            viol = myPlanner->getConstraintViolation(feetStates);

            //reset the counter for the replanning window
            sampleW = 0;
            std::cout<<"Start stance"<<std::endl;
        }//end of the optimization

        //this part runs at task_time_resolution

        //interpolate because time_resolution of the traj is different than task_time_resolution
        LegBoolMap liftOffFlag;
        int n = static_cast<int>(sampleTime/time_resolution);
        //remainder
        double rem = sampleTime - n*time_resolution;
        if (rem == 0.0)//update the interpolators only when the samplesW are changing (e.g. each time_resolution)
        {
            //std::cout<<"update interpolators, sampleTime = "<<sampleTime<<std::endl;
            interpolateCoMPositionX.setBoundary(sampleTime,time_resolution,des_com_x(sampleW), des_com_x(sampleW+1));
            interpolateCoMPositionY.setBoundary(sampleTime,time_resolution,des_com_y(sampleW), des_com_y(sampleW+1));
            interpolateCoMVelocityX.setBoundary(sampleTime,time_resolution,des_com_xd(sampleW), des_com_xd(sampleW+1));
            interpolateCoMVelocityY.setBoundary(sampleTime,time_resolution,des_com_yd(sampleW), des_com_yd(sampleW+1));
            //whenever a liftoff is expected from the plan, set the swing params
            liftOffFlag = detectLiftOff(feetStates, sampleW);
            //this update happens each time_resolution samples, at a lower frequency
            sampleW++;
        }

        interpolateCoMPositionX.getPoint(sampleTime, des_com_pos.x(rbd::X));
        interpolateCoMPositionY.getPoint(sampleTime, des_com_pos.x(rbd::Y));
        interpolateCoMVelocityX.getPoint(sampleTime, des_com_pos.xd(rbd::X));
        interpolateCoMVelocityY.getPoint(sampleTime, des_com_pos.xd(rbd::Y));

        //set swing spliner parameters at liftoff
        for (int leg = LF ; leg <=RH; leg++)
        {
            if (liftOffFlag[leg])
            {
                footTarget = mapWFToB(Vector3d(feetStates[leg].x(sampleW), feetStates[leg].y(sampleW), 0.0));
                footTargetW = gl.actual_base.x + gl.Rt*footTarget;
                //this is the delta step in the bse frame (concident with the swing frame for flat terrains)
                //TODO generalize with a compute step strategy
                Vector3d deltaFoot = footTarget - gl.footPos[leg];
                if (!hapticCrawl)
                {
                    footSpliner[leg].setSplineParameters(sampleTime,  horizon_duration/number_of_steps/2, Vector3d(0,0,1), gl.footPosDes[leg],  gl.R, deltaFoot(rbd::X), deltaFoot(rbd::Y),  step_height);

                } else {
                    footSpliner[leg].setSplineParameters(sampleTime,  horizon_duration/number_of_steps/2, gl.vec_incl[leg], gl.footPos[leg],  gl.R, deltaFoot(rbd::X), deltaFoot(rbd::Y),  step_height);
                }
                gl.stance_legs[leg] = false;
                std::cout<<"start swinging "<<legmap[leg]<<" leg"<<std::endl;
            }
        }
        //creates the swing trajectory with the footspliner, and detect haptic touchdown
        for (int leg = LF ; leg <=RH; leg++)
        {
            if (!gl.stance_legs[leg]) //swing leg
            {
                //for the swing get the trajectory point by point
                footSpliner[leg].getPoint(sampleTime, gl.swingFootRef[leg]);
                gl.footPosDes[leg] = gl.swingFootRef[leg].x;
                gl.footVelDes[leg] = gl.swingFootRef[leg].xd;
                bool stop_condition = false;
                //detect touchdown
                if (hapticCrawl) //haptically
                {
                    if (footSpliner[leg].isSwingingDown())
                        stop_condition = (gl.vec_incl[leg].dot(gl.R.transpose()*gl.grForces[leg])>=force_th);
                } else { //programmatically
                    if (!feetStates[leg].swing(sampleW))
                        stop_condition = true;
                }

                if (stop_condition)
                {
                    gl.stance_legs[leg] = true; //the foot pos will be determined by the integrazion of base motion
                    touchDown[mySchedule->getCurrentSwing()]++;
                    std::cout<<touchDown[mySchedule->getCurrentSwing()] <<" touchdown of  "<<legmap[leg]<<" leg"<<std::endl;
                    if (stoppingFlag)
                    {
                        stoppingFlag = false;
                        replanningFlag = false;
                        des_com_xd.setZero();
                        des_com_yd.setZero();
                        des_com_pos.xd.setZero();
                        gl.des_base_orient.xd.setZero();
                    }
                }
            }

        }
        sample++;

        //plotting footsteps
        std::map<int, dwl::ColorType> colormap;
        colormap[iit::dog::LF] = dwl::ColorType::Red;
        colormap[iit::dog::RF] = dwl::ColorType::Blue;
        colormap[iit::dog::LH] = dwl::ColorType::Green;
        colormap[iit::dog::RH] = dwl::ColorType::Yellow;
        for (int leg=LF; leg<=RH; leg++)
        {

            for (int i=0; i<footHolds[leg].x.size(); i++)
            {
               double transparency = (footHolds[leg].x.size() - i)/footHolds[leg].x.size();
               display_->drawSphere(Vector3d(footHolds[leg].x(i),footHolds[leg].y(i),0.02),
                                     0.05,
                                     dwl::Color(colormap[leg], 0.5 + 0.5*transparency),
                                     "world");
            }
        }     

        //plot com trajectory on ground plane
        for (int i=0; i<des_com_x.size(); i++)
        {
            //decimate a bit to avoid overload
            //            if ((i % 10) == 0)
            //            {
            //                display_->drawSphere(Vector3d(des_com_x(i),des_com_y(i),0.0),
            //                                     0.05,
            //                                     dwl::Color(dwl::ColorType::Black, 1.),
            //                                     "world");
            //            }
        }
        //plot com / zmp trajectory on ground plane
        for (int i=0; i<zmp_x.size(); i++)
        {
            //decimate a bit to avoid overload
            if ((i % 1) == 0)
            {
                display_->drawSphere(Vector3d(zmp_x(i),zmp_y(i),0.02),
                                     0.02,
                                     dwl::Color(dwl::ColorType::Orange,1.),
                                     "world");
            }
        }

        //draw the correction
        if (useComStepCorrection)
        {
            dwl::ArrowProperties arrow(0.02, 0.05, 0.0);
            display_->drawArrow(Vector3d(initial_feet_x[LF],initial_feet_y[LF],0.02),Vector3d(initial_feet_x[LF],initial_feet_y[LF],0.02)   + dummy2[LF], arrow, dwl::Color(dwl::ColorType::Red, 1.),"world");
            display_->drawArrow(Vector3d(initial_feet_x[RF],initial_feet_y[RF],0.02),Vector3d(initial_feet_x[RF],initial_feet_y[RF],0.02)   + dummy2[RF], arrow, dwl::Color(dwl::ColorType::Red, 1.),"world");
            display_->drawArrow(Vector3d(initial_feet_x[LH],initial_feet_y[LH],0.02),Vector3d(initial_feet_x[LH],initial_feet_y[LH],0.02)   + dummy2[LH], arrow, dwl::Color(dwl::ColorType::Red, 1.),"world");
            display_->drawArrow(Vector3d(initial_feet_x[RH],initial_feet_y[RH],0.02),Vector3d(initial_feet_x[RH],initial_feet_y[RH],0.02)   + dummy2[RH], arrow, dwl::Color(dwl::ColorType::Red, 1.),"world");
        }

        //debug target
//        display_->drawSphere(Vector3d(footTargetW(rbd::X),footTargetW(rbd::Y),0.02),
//                             0.08,
//                             dwl::Color(dwl::ColorType::Red,0.2),
//                             "world");

    }
    //finished crawl replanning

    //common stuff to crawl and replanning crawl
    //draw des speed
    dwl::ArrowProperties arrow(0.02, 0.05, 0.0);
    display_->drawArrow(Vector3d(.5,0,0),
                        Vector3d(.5,0,0) +   Vector3d(linearSpeedX,linearSpeedY,0)  * 2,
                        arrow, dwl::Color(dwl::ColorType::Red, 1.),
                        "base_link");

    //map com motion into feet motion (assumes that iscomasbodypoint is set in task globals so com motion is directly mapped onto feet motion)
    gl.bodySpliner->updateFeetPoint(des_com_pos.xd, gl.des_base_orient.x, gl.des_base_orient.xd, planning_rate_,gl.stance_legs, gl.offCoM,gl.footPosDes,gl.footVelDes);

    //map feet motion into joint motion
    ik_->getJointState(gl.footPosDes,
                       gl.footVelDes,
                       des_q_, des_qd_, gl.useKinematicLimitsFlag,
                       gl.endStopViolation);

    //set joint state in the planned trajectory
    for (unsigned int i = 0; i < fbs_->getJointDoF(); i++) {
        // Getting the joint id
        unsigned int joint_id = getDWLJointId(JointIdentifiers(i));
        // Converting the actual whole-body states
        planned_ws_.setJointPosition(des_q_(i), joint_id);
        planned_ws_.setJointVelocity(des_qd_(i), joint_id);
        planned_ws_.setJointAcceleration(des_qdd_(i), joint_id);
        //no torques are sent
        planned_ws_.setJointEffort(0., joint_id);
    }

    //send the base orientation in the planned trajectory (remember that first you need to set the orientation part)
    planned_ws_.setBaseRPY(gl.des_base_orient.x);
    planned_ws_.setBaseRPYVelocity_W(gl.des_base_orient.xd);
    planned_ws_.setBaseRPYAcceleration_W(gl.des_base_orient.xdd);

    //compute the base frame version of des_com for debugging reasons
    des_com_posB.x = gl.R * des_com_pos.x;
    des_com_posB.xd = gl.R * des_com_pos.xd;

    //compute the base reference from the com reference (des_target_pos is the base)
    gl.des_target_pos.x = dog::getBaseFromCoM(q_,gl.des_base_orient.x, des_com_pos.x, *gl.linksInertia);
    rbd::Vector6D desComTwist;
    desComTwist <<  gl.des_base_orient.xd , des_com_pos.xd;
    rbd::Vector6D desBaseTwist;
    Matrix3d Rdes = commons::rpyToRot(gl.des_base_orient.x);
    desBaseTwist =  dog::motionVectorTransform(Rdes.transpose()*gl.offCoM, Matrix3d::Identity())  *
            (desComTwist - dog::motionVectorTransform(Eigen::Vector3d(0,0,0), Rdes.transpose())*
             dog::getWholeBodyCOMJacobian(q_, *gl.linksInertia, *gl.homogeneousTransforms)*qd_ );
    gl.des_target_pos.xd = rbd::linearPart(desBaseTwist);

    //set the base position in the planned trajectory
    planned_ws_.setBasePosition(gl.des_target_pos.x);
    planned_ws_.setBaseVelocity_W(gl.des_target_pos.xd);
    planned_ws_.setBaseAcceleration_W(gl.des_target_pos.xdd);//TODO accel

    //set the desired foot positions in the planned trajectory
    planned_ws_.setContactPosition_B("01_lf_foot", gl.footPosDes[LF]);
    planned_ws_.setContactPosition_B("02_rf_foot", gl.footPosDes[RF]);
    planned_ws_.setContactPosition_B("03_lh_foot", gl.footPosDes[LH]);
    planned_ws_.setContactPosition_B("04_rh_foot", gl.footPosDes[RH]);

    //send the desired foot velocities in the planned trajectory
    planned_ws_.setContactVelocity_B("01_lf_foot", gl.footVelDes[LF]);
    planned_ws_.setContactVelocity_B("02_rf_foot", gl.footVelDes[RF]);
    planned_ws_.setContactVelocity_B("03_lh_foot", gl.footVelDes[LH]);
    planned_ws_.setContactVelocity_B("04_rh_foot", gl.footVelDes[RH]);

    //send the desired stance legs in the planned trajectory
    planned_ws_.setContactCondition("01_lf_foot",gl.stance_legs[LF]);
    planned_ws_.setContactCondition("02_rf_foot",gl.stance_legs[RF]);
    planned_ws_.setContactCondition("03_lh_foot",gl.stance_legs[LH]);
    planned_ws_.setContactCondition("04_rh_foot",gl.stance_legs[RH]);

    // Passing the planned whole-body state for the publishing
    planned_wt_->resize(1);
    planned_wt_->at(0) = planned_ws_;

}

//common functions
void CrawlPlanner::kill() {
    std::cout << "Killing CrawlPlanner planner" << std::endl;
}

void CrawlPlanner::setRobotModels(std::shared_ptr<iit::dog::FeetJacobians>& feet_jacs,
                                     std::shared_ptr<iit::dog::ForwardKinematics>& fwd_kin,
                                     std::shared_ptr<iit::dog::ShinJacobians>& shin_jacs,
                                     std::shared_ptr<InertiaPropertiesBase> &inertia_props,
                                     std::shared_ptr<HomogeneousTransformsBase> &hom_transforms,
                                     std::shared_ptr<JSIMBase> &jsim,
                                     std::shared_ptr<InverseDynamicsBase> &inv_dyn,
                                     std::shared_ptr<KinDynParams> &params,
                                     std::shared_ptr<LimitsBase> & limits,
                                     std::shared_ptr<FeetContactForces> & feet_forces)
 {
    PlannerBase::setRobotModels(feet_jacs,
                               fwd_kin,
                               shin_jacs,
                               inertia_props,
                               hom_transforms,
                               jsim,
                               inv_dyn, params,
                               limits,
                               feet_forces);

    // setting the God Object
    gl.feet_jacobians_ = feet_jacs;
    gl.fwd_kin_ = fwd_kin;
    gl.linksInertia = inertia_props;
    gl.homogeneousTransforms = hom_transforms;
    gl.jsim = jsim;
    gl.invDynEngine = inv_dyn;
    gl.default_pg = params;
    gl.limits = limits;
}


//crawl functions
void CrawlPlanner::crawlStateMachine(double time)
{

    //get the current swing
    swing_leg_index = mySchedule->getCurrentSwing();
    //update phase duration according to user input
    update_phase_duration(gl.cycle_time);

    switch (state_machine) {
    for (int leg=dog::LF; leg<=dog::RH; leg++)
    {
        if (leg == mySchedule->getCurrentSwing())
            state_machine_leg[leg] = state_machine;
        else
            state_machine_leg[leg] = idle;
    }
    case(move_base_to_next_support_triangle):
	        if (baseTimer.resetFlag) {//do this just once at the beginning
	            std::cout <<"move base"<<std::endl;
	            step_count++;
	            //compute next base position
	            Vector3d new_triangle_baryW, new_triangle_baryWoff;
	            //get coord of bary in world frame
                bodyTargetHandler->computeBaryNextTriangle(gl.R, swing_leg_index, gl.footPos, gl.terr_normal, stab_margin, new_triangle_baryW);
                //for plotting purposes
	            BaryTriangleW= gl.actual_base.x + new_triangle_baryW;
	            gl.dummy_var1 = gl.actual_base.x +bodyTargetHandler->getDummyVar();
	            gl.dummy_var2 = gl.actual_base.x +new_triangle_baryW;

	            //alternative  WAY (TODO)
	            //prt(new_triangle_baryW)
	            ////the step must be parallel to the terrain
	            //Eigen::Vector3d comToBaryW = new_triangle_baryW - gl.Rt*gl.offCoM; //get the offset of the com in world frame
	            ////add the des_height! comToBaryW is a point on the polygon
	            //comToBaryW += gl.terr_normal*gl.des_height;
	            //prt(comToBaryW)

	            //subtract the offset (in world frame) of the com wrt to the base
	            new_triangle_baryW -= gl.Rt*gl.offCoM; //get the offset of the com in world frame
	            double scaling_factor = Vector3d(0,0,1).dot(gl.terr_normal); //cos alpha
	            double height_shift = (gl.des_height)/scaling_factor; //TODO  to increase robustnessgl.des_height+ (gl.des_height - gl.actual_CoM_height_f)
	            new_triangle_baryW(rbd::Z) += height_shift;

                //compute new setpoint for position/orientation
	            Vector3d target_pos, target_orient, delta_pos, delta_orient;

	            //add the new value
	            target_pos = gl.actual_CoM.x + new_triangle_baryW; //for dead reckoning we update the desired to the actual (no problems for IK cause is at the velocity level)

	            //adapts orientation to terrain inclination
	            bodyTargetHandler->computeOrientationCorrection(gl, delta_orient);
	            target_orient = gl.actual_orient.x + Terrain_Estimation*delta_orient + Vector3d(0.0,0.0,bodyTargetHandler->computeDeltaYaw(gl));

	            //TODO if it is unstable use a different bodyspliner for base and feet for the base dont set = to the actual
	            //set spliner boundaries
	            gl.bodySpliner->setPosBoundary(time, base_motion_duration, gl.actual_CoM.x, target_pos);
	            gl.bodySpliner->setOrientBoundary(time, base_motion_duration, gl.actual_orient.x, target_orient);//for dead reckoning we update the desired to the actual (no problems for IK cause is at the velocity level)
                gl.bodySpliner->setInitialFeetPosition(gl.footPosDes);
	            baseTimer.startTimer(time);
	        }
    if(update_base_position(time)){ //this timer finished
          state_machine = unloadleg;

    }
    break;

    case(unloadleg):
        if (forceTimer.resetFlag)
        {
            std::cout <<"unload leg"<<std::endl;
            //project on the normal (versor) using dot product
            forceLimSpliner.setBoundary(time, force_motion_duration, gl.vec_incl[swing_leg_index].dot(gl.feetForces[swing_leg_index]), 5);
            forceTimer.startTimer(time);
        }
        if(update_load_force(swing_leg_index, time)){
            state_machine = swingleg;
            gl.stance_legs[swing_leg_index] = false;
            //reference for swing only the next iteration
            gl.swingFootRef[swing_leg_index].x = gl.footPosDes[swing_leg_index];
            gl.swingFootRef[swing_leg_index].xd.setZero();//set velocity to zero
            gl.swingFootRef[swing_leg_index].xdd.setZero();
        }
    break;

    case (swingleg):

	        if (swingTimer.resetFlag)
	        {
                std::cout <<"swing leg "<<legmap[swing_leg_index]<<std::endl;
                step_x = 0.0;
                step_y = 0.0;
                stepHandler->computeStepLength(q_, swing_leg_index,  gl.footPos, linearSpeedX, linearSpeedY, headingSpeed, step_x, step_y); //compute the step length in the base frame and updates the cycle duration
                footSpliner[swing_leg_index].setSplineParameters(time,  swing_motion_duration, gl.vec_incl[swing_leg_index], gl.footPosDes[swing_leg_index],  gl.R, step_x, step_y, step_height);
	            swingTimer.startTimer(time);
	        }
    if(update_swing_position(swing_leg_index,time))
    {
        //touchdown event
        state_machine = loadleg;
        gl.stance_legs[swing_leg_index] = true;
        //set gl.vec_incl with terrain inclination
        if (Terrain_Estimation)
        {
            gl.vec_incl[swing_leg_index] = commons::rpyToRot(Vector3d(gl.terrRoll, gl.terrPitch, gl.yaw)).transpose()*Vector3d(0,0,1);
            //prt(vec_incl[swing_leg_index]transpose())
        }
    }
    break;

    case(loadleg):
	        if (forceTimer.resetFlag) {
	            std::cout <<"load leg"<<std::endl;
	            forceLimSpliner.setBoundary(time, force_motion_duration, force_th,800.0);
	            gl.low_force_limit[swing_leg_index] = force_th;
                forceTimer.startTimer(time);
	        }
    if(update_load_force(swing_leg_index,time))
    {
        gl.stance_legs_odometry[swing_leg_index] = true;
        gl.frame_change = true;

        if (stopping_crawl)
        {
            state_machine = idle;
            stopping_crawl = false;
        } else {
            state_machine = move_base_to_next_support_triangle;
        }

        gl.low_force_limit[swing_leg_index] = 5; //allows force to go down almost to zero because if you have modeling errors the com could go very close to the support polygon
        //update next leg to move //the swing leg has changed
        //get next swing leg
        mySchedule->next();
    }
    break;
    default:
        break;
    }
}

void CrawlPlanner::computeTerrainEstimation()
{
    //estimate terrain
    terrainEstimator.setBaseAngles(bs->getRoll_W(), bs->getPitch_W());
    terrainEstimator.setForceThreshold(20);
    terrainEstimator.setFilter((double)1/planning_rate_, 0.1);
    terrainEstimator.setGroundReactionForcesBF(gl.grForces[dog::LF](rbd::Z),gl.grForces[dog::RF](rbd::Z),gl.grForces[dog::LH](rbd::Z),gl.grForces[dog::RH](rbd::Z));
    Eigen::Matrix<double, 3,4> feetPosition;
    feetPosition << gl.footPos[dog::LF](rbd::X),gl.footPos[dog::RF](rbd::X),gl.footPos[dog::LH](rbd::X),gl.footPos[dog::RH](rbd::X),
            gl.footPos[dog::LF](rbd::Y),gl.footPos[dog::RF](rbd::Y),gl.footPos[dog::LH](rbd::Y),gl.footPos[dog::RH](rbd::Y),
            gl.footPos[dog::LF](rbd::Z),gl.footPos[dog::RF](rbd::Z),gl.footPos[dog::LH](rbd::Z),gl.footPos[dog::RH](rbd::Z);
    terrainEstimator.setLegContactPositionsBF(feetPosition);
    if (roughTerrainFlag){
        terrainEstimator.ComputeTerrainEstimationRoughTerrain(gl.terr_normal, gl.terrRoll, gl.terrPitch);
    } else  {
        terrainEstimator.ComputeTerrainEstimation(gl.terrRoll, gl.terrPitch);
    }
    gl.des_height = initial_height*cos(gl.terrPitch);//*fabs(gl.terr_normal.dot(Vector3d(0.0,0.0,1.0)));
}

bool  CrawlPlanner::update_base_position(double time){
    bool stop_condition;
    stop_condition = baseTimer.isTimeElapsed(time);
    if (!stop_condition){
        //get feet reference to move base
        gl.bodySpliner->getBodyPoint(time, des_com_pos, gl.des_base_orient);
        return false; //base is moving
    } else {
        baseTimer.resetTimer(); //reset timer for next state
        return true;
    }
}

bool  CrawlPlanner::update_load_force(dog::LegID swing_leg_index, double time)
{
    if (!forceTimer.isTimeElapsed(time)){
        //gains are splining also for the static case because the setChangePIDGains has been set
        forceLimSpliner.getPoint(time, force_intermediate);
        gl.high_force_limit[swing_leg_index] = force_intermediate.x;
        return false;
    }else{
        forceTimer.resetTimer(); //reset timer for next state
        return true;
    }
}

bool  CrawlPlanner::update_swing_position(dog::LegID swing_leg_index, double time)
{
    bool stop_condition;
    footSpliner[swing_leg_index].getPoint(time, gl.swingFootRef[swing_leg_index]);
    gl.footPosDes[swing_leg_index] = gl.swingFootRef[swing_leg_index].x;
    gl.footVelDes[swing_leg_index] = gl.swingFootRef[swing_leg_index].xd;
    if (!hapticCrawl){
        stop_condition = swingTimer.isTimeElapsed(time); //nohaptic
    } else {
        stop_condition = check_touch_down();
    }

    if (stop_condition){
        //prt("swingstoped")
        swingTimer.resetTimer();
        return true;
    } else{

    }
    return false;
}

bool CrawlPlanner::check_touch_down()
{
    iit::dog::LegDataMap<FootJac > JFootDes;
    JFootDes[LF] = gl.feet_jacobians_->getFootJacobian(des_q_,LF);
    JFootDes[RF] = gl.feet_jacobians_->getFootJacobian(des_q_,RF);
    JFootDes[LH] = gl.feet_jacobians_->getFootJacobian(des_q_,LH);
    JFootDes[RH] = gl.feet_jacobians_->getFootJacobian(des_q_,RH);

    double sigma_ws_limit;
    sigma_ws_limit = 0.009; //this is good also for hyqreal

    //verify if there is any endstop force and discart triggering with force in that case (do not consider it for knee joint! otherwise if the sigma is different for each robot it will not stop)
    bool jointLimitsHit = (gl.endStopViolation[toJointID(swing_leg_index, iit::dog::HAA)]|| gl.endStopViolation[toJointID(swing_leg_index, iit::dog::HFE)]);
    return footSpliner[swing_leg_index].check_stop_condition(jointLimitsHit, JFootDes[swing_leg_index], sigma_ws_limit, gl.grForces[swing_leg_index], force_th);
}

void CrawlPlanner::update_phase_duration(double new_cycle_time){
    //total sum is 4
    base_motion_duration = new_cycle_time * 2.0/4.0;
    swing_motion_duration = new_cycle_time * 1.98 / 4.0;
    force_motion_duration = new_cycle_time * 0.01/4.0;
    baseTimer.setDuration(base_motion_duration);
    forceTimer.setDuration(force_motion_duration);
    swingTimer.setDuration(swing_motion_duration);
}

void CrawlPlanner::start_crawl(void)
{
    if (state_machine == idle)
    {
        gl.frame_change = true;
        //start state machine
        state_machine = move_base_to_next_support_triangle;
        //prt(mySchedule->getCurrentSwing())
        std::cout<<"Crawl started"<<std::endl;

        bs->setPosition_W(actual_ws_->getBasePosition());
        bs->setVelocity_W(actual_ws_->getBaseVelocity_W());
        bs->setRotationRate_B(actual_ws_->getBaseAngularVelocity_W());
        bs->setRotAcceleration_B(actual_ws_->getBaseAngularAcceleration_W());
        bs->setOrientation_W(actual_ws_->getBaseOrientation());
        gl.footPosDes = gl.footPos;
    }
}


//replanning functions
Vector3d CrawlPlanner::mapBToWF(Vector3d B_vec_in)
{
    return gl.Rt*B_vec_in  + gl.actual_base.x;
}

Vector3d CrawlPlanner::mapWFToB(Vector3d W_vec_in)
{
    return gl.R*(W_vec_in  - gl.actual_base.x);
}


void CrawlPlanner::debug()
{

    //1 shoot optimization
    Vector2d userSpeed = Vector2d(linearSpeedX, linearSpeedY);
    //single plan
    myPlanner->computeSteps(userSpeed,
                            initial_feet_x,
                            initial_feet_y,
                            number_of_steps,
                            horizon_size,
                            feetStates, footHolds,
                            A, b, *myPlanner,
                            mySchedule->getCurrentSwing());
    //,Vector2d(gl.actual_CoM.x(rbd::X), gl.actual_CoM.x(rbd::Y)));

    print_foot_holds();
    myPlanner->solveQPConstraintCoupled(gl.actual_CoM_height,
                                        actual_state_x, actual_state_y,
                                        A,b,jerk_x,jerk_y);
    myPlanner->computeCOMtrajectory( actual_state_x, jerk_x, des_com_x);
    myPlanner->computeCOMtrajectory( actual_state_y, jerk_y, des_com_y);
    myPlanner->computeZMPtrajectory( actual_state_x, jerk_x, zmp_x);
    myPlanner->computeZMPtrajectory( actual_state_y, jerk_y, zmp_y);

}

void CrawlPlanner::print_foot_holds()
{
    for (int leg=LF; leg<=RH; leg++)
    {
        std::cout<<std::endl;
        std::cout<<legmap[leg]<<" X : ";
        for (int i=0; i<footHolds[leg].x.size(); i++)
        {
            if ((i % 1) == 0)
                std::cout<<footHolds[leg].x[i]<<"     ";

        }
        std::cout<<std::endl;
        std::cout<<legmap[leg]<<" Y : ";
        for (int i=0; i<footHolds[leg].y.size(); i++)
        {
            if ((i % 1) == 0)
                std::cout<<footHolds[leg].y[i]<<"     ";
        }
    }
    std::cout<<std::endl;
}


void CrawlPlanner::start_replanning_crawl()
{
    if (!replanningFlag)
    {
        newline::getDouble("Horizon duration?:", horizon_duration, horizon_duration);
        newline::getDouble("Time resolution?:", time_resolution, time_resolution);
        newline::getInt("Number_of_steps in the horizon?:", number_of_steps, number_of_steps);
        newline::getInt("Replanning steps?:", replanning_steps, replanning_steps);
        resetPlanner();
        sample = 0;
        sampleW = 0;
        firstTime = true;
        replanningFlag = true;
    } else {
        stoppingFlag = true;
    }
}

LegBoolMap  CrawlPlanner::detectLiftOff(iit::dog::LegDataMap<MPCPlanner::footState> feetStates, double actualSample)
{
    LegBoolMap swingTrigger = false;
    for(int leg = LF; leg<=RH; leg++ )
    {
        //detect only rising edge
        if ((!feetStates[leg].swing(actualSample-1)) && (feetStates[leg].swing(actualSample)))
            swingTrigger[leg] = true;
    }
    return swingTrigger;
}

void CrawlPlanner::interactiveChangeParams(void)
{
    std::cout << "Poor man's ICTP " << std::endl;
    std::cout << "8/2 to +/- linearSpeed X " << std::endl;
    std::cout << "4/6 to +/- linearSpeed Y " << std::endl;
    std::cout << "o/l increase/decrease stepping frequency (needs toggleUseUserFreq) " << std::endl;
    std::cout << "q to exit" << std::endl;

    /* use system call to make terminal send all keystrokes directly to stdin */
    int temp;
    temp = system ("/bin/stty raw");

    int c;
    while(1)
    {
        c=getchar();
        std::string user_text(1, c);
        if (user_text == "4"){
            linearSpeedY+=0.005;
            if (linearSpeedY>0.4)
                linearSpeedY = 0.4;

            std::cout<<" :"<<linearSpeedY<<std::endl<<std::endl;

        }else if (user_text == "6"){
            linearSpeedY-=0.005;
            if (linearSpeedY<-0.4)
                linearSpeedY = -0.4;
            std::cout<<" :"<<linearSpeedY<<std::endl<<std::endl;
        }else if (user_text == "8"){
            linearSpeedX+=0.005;
            if (linearSpeedX>0.5)
                linearSpeedX = 0.5;
            std::cout<<" linearSpeedX:"<<linearSpeedX<<std::endl<<std::endl;
        }else if (user_text == "2"){
            linearSpeedX-=0.005;
            if (linearSpeedX<-0.5)
                linearSpeedX = -0.5;
            std::cout<<" linearSpeedX:"<< linearSpeedX<<std::endl<<std::endl;
        }else if(user_text == "q"){
            break;
        }else if(user_text == "Q"){
            break;
        }
    }
    temp  = 	system ("/bin/stty cooked");
}

void CrawlPlanner::toggleCoMCorrection()
{
    if (useComStepCorrection){
        useComStepCorrection = false;
        std::cout<< "use com correction OFF" <<std::endl;
    }	else {
        useComStepCorrection = true;
        std::cout<< "use user frequency ON" <<std::endl;
    }
}
void CrawlPlanner::toggleOptimizeVelocity()
{
    if (optimizeVelocityFlag){
        optimizeVelocityFlag = false;
        std::cout<< "optimize velocity OFF" <<std::endl;
    }	else {
        optimizeVelocityFlag = true;
        std::cout<< "optimize velocity ON" <<std::endl;
    }
}
void CrawlPlanner::resetPlanner()
{
    horizon_size = horizon_duration / time_resolution;
    std::cout<<"horizon_duration is :"<<horizon_duration<< std::endl;
    std::cout<<"time_resolution is (should be a multiple of task_planning_rate):"<<time_resolution<< std::endl;
    std::cout<<"horizon size is :"<<horizon_size<< std::endl;
    std::cout<<"number of steps is :"<<number_of_steps<< std::endl;
    std::cout<<"userspeed is X:"<<linearSpeedX <<"  Y: "<<linearSpeedY << std::endl;
    replanningWindow = horizon_size/number_of_steps; //after one 4stance and one 3 stance replan using the actual_swing, and actual foot pos and and actual com
    std::cout<<"replanningWindow is :"<<replanningWindow<< std::endl;
    //init the planner
    myPlanner.reset(new MPCPlanner(horizon_size,    time_resolution,   rbd::g));

}

}//namespace
