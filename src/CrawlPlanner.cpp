#include <crawl_planner/CrawlPlanner.h>

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
    //init crawl state machine
    state_machine = idle;

    // Creating real-time subscribers:
    //this is for the console
    //char_sub_ = nh_sub.subscribe<std_msgs::Char> ("char", 1, &CrawlPlanner::charCB, this, ros::TransportHints().tcpNoDelay());

    num_joints_ = fbs_->getJointDoF();
    planned_ws_.setJointDoF(num_joints_);

    printf(BLUE "Initialization of the CrawlPlanner accomplished\n" COLOR_RESET);

    return true;
}

void CrawlPlanner::printCharOptions()
{
    std::cout << "Menu options " << std::endl;
    std::cout << "(to send command use - rosrun dls_console dls_console) " << std::endl;
    std::cout << "(topic should be /robot/char) " << std::endl;
    std::cout << "- f to move forward " << std::endl;

}

void CrawlPlanner::starting(double time) {
    std::cout << "Starting CrawlPlanner controller at " << time << " sec.";
    std::cout << std::endl;
    //printCharOptions();
    //crawl init
    printf("chimney plan starting\n");

    //in the init there is not the state of the robot
    gl.param_file = ros::package::getPath(std::string("crawl_planner")) + "/config/planner.ini";

    //this is stuff that needs the first state to arrive...
    //	// Converting WholeBodyState to RobCoGen
    for (unsigned int i = 0; i < fbs_->getJointDoF(); i++) {
        // Getting the joint id
        unsigned int joint_id = getDWLJointId(JointIdentifiers(i));
        // Converting the desired joint states from RobCoGen order
        q_(i) = actual_ws_->joint_pos(joint_id);
        qd_(i) = actual_ws_->joint_vel(joint_id);
        qdd_(i) = 0.;
        tau_(i) = actual_ws_->joint_eff(joint_id);
    }

    bs->setPosition_W(actual_ws_->getBasePosition());
    bs->setVelocity_W(actual_ws_->getBaseVelocity_W());
    bs->setRotationRate_B(actual_ws_->getBaseAngularVelocity_W());
    bs->setRotAcceleration_B(actual_ws_->getBaseAngularAcceleration_W());
    bs->setOrientation_W(actual_ws_->getBaseOrientation());
    //compute torque limits according to the actual config of the robot
    tau_max_ = robot_limits_->getTorqueLimits(q_);



    gl.init(q_, qd_, qdd_); //this sets also 	footPosDes = footPos;
    gl.sl_to_eigenLogic(q_, qd_, tau_, tau_max_, qdd_, *bs);
    gl.setServoRate(planning_rate_);
    stepHandler->setSpeedParameters(0.2, 0.2, 0.2, 0.1, 0.1, 0.1, 0.022, 0.022, 0.022);
    stepHandler->setSpeedVariation(true);
    roughTerrainFlag = gl.config_.get<bool>("Crawl.roughTerrain");
    bodyTargetHandler->setBackwardMotion(true);
    bodyTargetHandler->setRoughTerrain(roughTerrainFlag);
    //crawl options
    force_th = gl.config_.get<double>("Crawl.force_th");
    hapticCrawl = gl.config_.get<bool>("Crawl.hapticCrawl");
    Terrain_Estimation  = gl.config_.get<bool>("Crawl.terrain_estimation");
    stepHandler->useTerrainEstimation(Terrain_Estimation);
    gl.muEstimate = gl.config_.get<double>("Crawl.mu_estimate");
    step_height = gl.config_.get<double>("Crawl.step_height");
    linearSpeedX = gl.config_.get<double>("Crawl.linearSpeedX");
    linearSpeedY = gl.config_.get<double>("Crawl.linearSpeedY");
    headingSpeed = gl.config_.get<double>("Crawl.headingSpeed");
    gl.cycle_time = gl.config_.get<double>("Crawl.cycle_time");
    prt(gl.cycle_time)
            update_phase_duration(gl.cycle_time);
    //step handler init
    stepHandler->setHipDefaultYOffset(0.06 + fabs((gl.footPos[iit::dog::LF] - stepHandler->getHipPosition(q_,iit::dog::LF))(rbd::Y)));
    stepHandler->setDefaultStepLength((linearSpeedX * gl.cycle_time));
    //set des joints = actual
    gl.footPosDes = sample_footPosDes = gl.footPos;
    des_q_ = q_;
    des_qd_.setZero();

    gl.actual_base.x = actual_ws_->getBasePosition();
    //reset desired linear position to actual readings
    gl.computeCoM(q_, qd_);
    des_com_pos.x = gl.actual_CoM.x;//com control
    //reset desired orientation
    gl.des_base_orient.x(rbd::X) = gl.roll;
    gl.des_base_orient.x(rbd::Y) = gl.pitch;
    gl.des_base_orient.x(rbd::Z) = gl.yaw;
    //set des height
    gl.des_height = gl.actual_CoM_height;
    initial_height = gl.actual_CoM_height;
    gl.frame_change = true;
    gl.terr_normal = Vector3d(0,0,1.0);

   prt(mySchedule->getCurrentSwing())
    printf("finished starting\n");
}


void CrawlPlanner::run(double time,
        double period) {

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

    updateVarsForDataLogging();

    //get actual states!
    bs->setPosition_W(actual_ws_->getBasePosition());
    bs->setVelocity_W(actual_ws_->getBaseVelocity_W());
    bs->setRotationRate_B(actual_ws_->getBaseAngularVelocity_W());
    bs->setRotAcceleration_B(actual_ws_->getBaseAngularAcceleration_W());
    bs->setOrientation_W(actual_ws_->getBaseOrientation());

    gl.sl_to_eigenLogic(q_, qd_, tau_,tau_max_, qdd_, *bs);
    //com identification
    //for display purposes
    linearSpeedX_dsp = linearSpeedX*1000;
    linearSpeedY_dsp = linearSpeedY*1000;
    headingSpeed_dsp = headingSpeed*1000;
    //get the current swing
    swing_leg_index = mySchedule->getCurrentSwing();
    //update phase duration according to user input
    update_phase_duration(gl.cycle_time);
    if (Terrain_Estimation){
        computeTerrainEstimation();
    }


    //Run state-machine!
    //fills in the base and joints variables
    crawlStateMachine(time);

    //map com motion into feet motion
    gl.bodySpliner->updateFeetPoint(des_com_pos.xd, gl.des_base_orient.x, gl.des_base_orient.xd, planning_rate_,gl.stance_legs, gl.offCoM,gl.footPosDes,gl.footVelDes);
    //map feet motion into joint motion
    ik_->getJointState(gl.footPosDes,
                       gl.footVelDes,
                       des_q_, des_qd_, gl.useKinematicLimitsFlag,
                       gl.endStopViolation);


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

    //send the des_base_state (first do the orientation part)


    planned_ws_.setBaseRPY(gl.des_base_orient.x);
    planned_ws_.setBaseRPYVelocity_W(gl.des_base_orient.xd);
    planned_ws_.setBaseRPYAcceleration_W(gl.des_base_orient.xdd);

    Matrix3d Rdes = commons::rpyToRot(gl.des_base_orient.x);
    //compute the base frame version of des_com for plotting reasons
    des_com_posB.x = gl.R * des_com_pos.x;
    des_com_posB.xd = gl.R * des_com_pos.xd;

    //compute the base reference from com reference, (des_target_pos is the base)
    gl.des_target_pos.x = dog::getBaseFromCoM(q_,gl.des_base_orient.x, des_com_pos.x, *gl.linksInertia);
    rbd::Vector6D desComTwist; desComTwist <<  gl.des_base_orient.xd , des_com_pos.xd;
    rbd::Vector6D desBaseTwist;
    desBaseTwist =  dog::motionVectorTransform(Rdes.transpose()*gl.offCoM, Matrix3d::Identity())  *
                        (desComTwist - dog::motionVectorTransform(Eigen::Vector3d(0,0,0), Rdes.transpose())*
                        dog::getWholeBodyCOMJacobian(q_, *gl.linksInertia, *gl.homogeneousTransforms)*qd_ );
    gl.des_target_pos.xd = rbd::linearPart(desBaseTwist);

    planned_ws_.setBasePosition(gl.des_target_pos.x);
    planned_ws_.setBaseVelocity_W(gl.des_target_pos.xd);
    planned_ws_.setBaseAcceleration_W(gl.des_target_pos.xdd);//TODO accel

    //send the desired foot positions
    planned_ws_.setContactPosition_B("01_lf_foot", gl.footPosDes[LF]);
    planned_ws_.setContactPosition_B("02_rf_foot", gl.footPosDes[RF]);
    planned_ws_.setContactPosition_B("03_lh_foot", gl.footPosDes[LH]);
    planned_ws_.setContactPosition_B("04_rh_foot", gl.footPosDes[RH]);

    //send the desired foot velocities
    planned_ws_.setContactVelocity_B("01_lf_foot", gl.footVelDes[LF]);
    planned_ws_.setContactVelocity_B("02_rf_foot", gl.footVelDes[RF]);
    planned_ws_.setContactVelocity_B("03_lh_foot", gl.footVelDes[LH]);
    planned_ws_.setContactVelocity_B("04_rh_foot", gl.footVelDes[RH]);

    //send the desired stance legs
    planned_ws_.setContactCondition("01_lf_foot",gl.stance_legs[LF]);
    planned_ws_.setContactCondition("02_rf_foot",gl.stance_legs[RF]);
    planned_ws_.setContactCondition("03_lh_foot",gl.stance_legs[LH]);
    planned_ws_.setContactCondition("04_rh_foot",gl.stance_legs[RH]);

    //std::cout << "Running the planning loop at " << 1 / period << " Hz" << std::endl;

    // Passing the planned whole-body state for the publishing
    planned_wt_->resize(1);
    planned_wt_->at(0) = planned_ws_;

}

void CrawlPlanner::kill() {
    std::cout << "Killing CrawlPlanner planner" << std::endl;
}




void CrawlPlanner::crawlStateMachine(double time)
{
    taskServoTime = time;
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
	            Matrix3d Rdes = commons::rpyToRot(gl.des_base_orient.x);
	            bodyTargetHandler->computeBaryNextTriangle(gl.R, swing_leg_index, gl.footPos, gl.terr_normal, stab_margin, new_triangle_baryW);
	            prt(gl.terr_normal)
	            prt(gl.R)
	            prt(gl.footPos)
	            prt(new_triangle_baryW)
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

	            sample_footPosDes = gl.footPosDes;//this is only for the static case for the whole dynamics since PD is switched off I dont care
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
	            gl.bodySpliner->setInitialFeetPosition(sample_footPosDes);
	            baseTimer.startTimer(time);
	        }
    if(update_base_position(time)){ //this timer finished
        //state_machine = idle;//for pushDog tests
        state_machine = unloadleg;

    }
    break;

    case(unloadleg):
								        if (forceTimer.resetFlag) {
								            std::cout <<"unload leg"<<std::endl;
								            //project on the normal (versor) using dot product
                                            forceLimSpliner.setBoundary(time, force_motion_duration, gl.vec_incl[swing_leg_index].dot(gl.feetForces[swing_leg_index]), 5);
								            forceTimer.startTimer(time);
								        }
    if(update_load_force(swing_leg_index, time)){
        state_machine = swingleg;

        gl.stance_legs[swing_leg_index] = false;
        gl.stance_legs_odometry[swing_leg_index] = false;
        gl.frame_change = true;

        dog::LegDataMap<Eigen::Vector3d> zero_vel = Eigen::Vector3d(0,0,0);
        //this is important to prevent discontinuities on the torque which can cause instabilities
        //in the dynamics casecause I set the swing leg on but I compute the
        //reference for swing only the next iteration
        gl.swingFootRef[swing_leg_index].x = gl.footPosDes[swing_leg_index];
        gl.swingFootRef[swing_leg_index].xd.setZero();//set velocity to zero
        gl.swingFootRef[swing_leg_index].xdd.setZero();
    }
    break;

    case (swingleg):

	        if (swingTimer.resetFlag)
	        {
	            std::cout <<"swing leg"<<std::endl;
	            //prt(swing_leg_index)
	            //sample actual position to apply the spline from
	            sample_footPosDes = gl.footPosDes; //otherwise since you have the PD in the static case you can have discontinuities
	            double step_x = 0.0;double step_y = 0.0;    
                stepHandler->computeStepLength(q_, swing_leg_index,  gl.footPos, linearSpeedX, linearSpeedY, headingSpeed, step_x, step_y); //compute the step length in the base frame and updates the cycle duration

                footSpliner[swing_leg_index].setSplineParameters(time,  swing_motion_duration, gl.vec_incl[swing_leg_index], sample_footPosDes[swing_leg_index],  gl.R, step_x, step_y, step_height);

	            //for stairs
	            //					ictp_mode = true;
	            //					Vector3d swing_frameZ = gl.vec_incl[swing_leg_index]; // if there is a stair swingZ will be modified
	            //					stepHandler->computeStepLengthStairs(swing_leg_index,  gl.footPos, linearSpeedX, linearSpeedY, headingSpeed, step_x, step_y, swing_frameZ, mySchedule); //compute the step length in the base frame
	            //					footSpliner[swing_leg_index].setSplineParameters(time,  swing_motion_duration, swing_frameZ, sample_footPosDes[swing_leg_index],  gl.R, step_x, step_y, step_height);

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
	            //create a spline in the base frame to recover Z elevation (just when you dont use USE_HEIGHT_CONTROL)
	            double leg_offset = gl.des_height -gl.offCoM(rbd::Z)- (-gl.footPosDes[swing_leg_index](rbd::Z));
	            footSpliner[swing_leg_index].setSplineParameters(time,  force_motion_duration, gl.footPosDes[swing_leg_index], 0, 0, leg_offset);
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
        //for stairs
        //stepHandler->handleStairs(q_, mySchedule); //changes sequence

        //wbOpt->resetStanceSpringConstraint(swing_leg_index); //TODO
    } else{
        //if (!WholeDynamics){update_leg_correction(swing_leg_index);} //TODO use only if you dont use USEHEIGHCONTROL in trunk controller
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
    }
    else
    {
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

void  CrawlPlanner::update_leg_correction(dog::LegID swing_leg_index, double time)
{
    footSpliner[swing_leg_index].getPoint(time, gl.swingFootRef[swing_leg_index]);
    gl.footPosDes[swing_leg_index] = gl.swingFootRef[swing_leg_index].x;
    gl.footVelDes[swing_leg_index] = gl.swingFootRef[swing_leg_index].xd;
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
        iit::dog::LegDataMap<FootJac > JFootDes;
        JFootDes[LF] = gl.feet_jacobians_->getFootJacobian(des_q_,LF);
        JFootDes[RF] = gl.feet_jacobians_->getFootJacobian(des_q_,RF);
        JFootDes[LH] = gl.feet_jacobians_->getFootJacobian(des_q_,LH);
        JFootDes[RH] = gl.feet_jacobians_->getFootJacobian(des_q_,RH);

        double sigma_ws_limit;
        sigma_ws_limit = 0.009; //this is good also for hyqreal

        //verify if there is any endstop force and discart triggering with force in that case (do not consider it for knee joint! otherwise if the sigma is different for each robot it will not stop)
        bool jointLimitsHit = (gl.endStopViolation[toJointID(swing_leg_index, iit::dog::HAA)]|| gl.endStopViolation[toJointID(swing_leg_index, iit::dog::HFE)]);
        stop_condition = footSpliner[swing_leg_index].check_stop_condition(jointLimitsHit, JFootDes[swing_leg_index], sigma_ws_limit, gl.grForces[swing_leg_index], force_th);
    }

    if (stop_condition){
        //prt("swingstoped")
        swingTimer.resetTimer();
        return true;
    } else{

    }
    return false;
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
        gl.footPosDes = sample_footPosDes = gl.footPos;
    }
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


//void CrawlController::fill_ZMP_traj(LegID preview_start_swing_index, LegDataMap<Vector3d> sampleFootPosDes, double missingTime)//TODO input actual current swing leg and remaining time
//{
//    int count = 0;
//    LegID swing_index;
//    LegDataMap<Vector3d> footDesTraj;
//    Vector3d actualzmp;actualzmp.setZero(); //first value is done by the feet pos
//    Vector3d triangle_bary;
//    FootScheduler ZMPSchedule(RH, RF, LH, LF);
//    ZMPSchedule.setCurrentSwing(preview_start_swing_index);
//
//    int num_samples_swing = floor(swingTrajDuration*taskServoRate);
//    int num_samples_to_TD = floor(missingTime*taskServoRate);
//    double stab_margin = 0.15;
//    zmp_traj.resize(2, num_traj_samples);
//    footDesTraj = sampleFootPosDes;
//
//    //start filling the buffer with the uncomplete 1 st step
//    swing_index = ZMPSchedule.getCurrentSwing();
//
//    Matrix3d Rdes = commons::rpyToRot(Vector3d(0,0,0));
//    bodyTargetHandler->computeBaryNextTriangle(Rdes, swing_index, footDesTraj, Vector3d(0,0,1), stab_margin, triangle_bary);
//    actualzmp.segment(rbd::X,2) += triangle_bary.segment(rbd::X,2);
//    actualzmp(rbd::Z) = gl.des_height;
//
//
//
//    //the step has already been done
//    zmp_traj.block(rbd::X, 0, 1, num_samples_to_TD) = actualzmp(rbd::X)*MatrixXd::Ones(1, num_samples_to_TD);
//    zmp_traj.block(rbd::Y, 0, 1, num_samples_to_TD) = actualzmp(rbd::Y)*MatrixXd::Ones(1, num_samples_to_TD);
//    footDesTraj[swing_index] +=Vector3d(ZMPstep_x, ZMPstep_y,0.0);//TODO replace with stephandler compute steps
//    count = count +num_samples_to_TD;
//    //gl.print_swing_leg(swing_index);
//    //prt(triangle_bary)
//    //prt(footDesTraj)
//    ZMPSchedule.next();
//
//    while (count<=num_traj_samples)
//    {
//        swing_index = ZMPSchedule.getCurrentSwing();
//
//
//
//        bodyTargetHandler->computeBaryNextTriangle(Rdes, swing_index, footDesTraj, Vector3d(0,0,1), stab_margin, triangle_bary);
//        actualzmp.segment(rbd::X,2) += triangle_bary.segment(rbd::X,2);
//        actualzmp(rbd::Z) = gl.des_height;
//        if ((count+num_samples_swing)<=zmp_traj.cols()){
//            zmp_traj.block(rbd::X, count, 1, num_samples_swing) = actualzmp(rbd::X)*MatrixXd::Ones(1, num_samples_swing);
//            zmp_traj.block(rbd::Y, count, 1, num_samples_swing) = actualzmp(rbd::Y)*MatrixXd::Ones(1, num_samples_swing);
//        }else
//        {
//            zmp_traj.block(rbd::X, count, 1, zmp_traj.cols()-count) =  actualzmp(rbd::X)*MatrixXd::Ones(1, zmp_traj.size()-count);
//            zmp_traj.block(rbd::Y, count, 1, zmp_traj.cols()-count) = actualzmp(rbd::Y)*MatrixXd::Ones(1, zmp_traj.size()-count);
//        }
//        footDesTraj[swing_index] +=Vector3d(ZMPstep_x, ZMPstep_y,0.0);//TODO replace with stephandler compute steps
//        count += num_samples_swing;
//        //gl.print_swing_leg(swing_index);
//        //prt(triangle_bary)
//        //prt(footDesTraj)
//        ZMPSchedule.next();
//
//    }
//    zmp(rbd::X) = zmp_traj(0,0);
//    zmp(rbd::Y) = zmp_traj(1,0);
//    //zmp_150(rbd::X) = zmp_traj(0,150);
//    //zmp_150(rbd::Y) = zmp_traj(1,150);
//
//    //prt(zmp_traj)
//}
//
//void CrawlController::plotZMPtraj()
//{
//    fill_ZMP_traj(iit::dog::RH, gl.footPosDes, 1.0);
//    plotTimer.setDuration(dt*num_traj_samples);
//    plotTimer.startTimer(taskServoTime);
//    plotZMPFlag = true;
//}



//TODO implement client server to strt trunk contr


}//namespace
