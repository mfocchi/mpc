/*
 * loggingFeatures.cpp
 *
 *  Created on:
 *      Author: mfocchi
 */

#include <crawl_planner/CrawlPlanner.h>

using namespace rbd;
namespace dls_planner
{

void CrawlPlanner::addVarToCollect( double value, std::string var_name){
	debugPush(var_name,value);
}


void CrawlPlanner::updateVarsForDataLogging() {
debugClear();

addVarToCollect( taskServoTime       , "time"      );
addVarToCollect( gl.des_base_pos.x(rbd::X)        , "des_baseX"      );
addVarToCollect( gl.des_base_pos.x(rbd::Y)        , "des_baseY"      );
addVarToCollect( gl.des_base_pos.x(rbd::Z)        , "des_baseZ"      );
addVarToCollect( gl.actual_base.x(rbd::X)        , "actual_baseX"      );
addVarToCollect( gl.actual_base.x(rbd::Y)        , "actual_baseY"      );
addVarToCollect( gl.actual_base.x(rbd::Z)        , "actual_baseZ"      );
addVarToCollect( gl.des_base_posB.x(rbd::X)        , "des_baseBX"      );
addVarToCollect( gl.des_base_posB.x(rbd::Y)        , "des_baseBY"      );
addVarToCollect( gl.des_base_posB.x(rbd::Z)        , "des_baseBZ"      );
addVarToCollect( gl.actual_CoM.x(rbd::X)          , "actual_CoMX"    );
addVarToCollect( gl.actual_CoM.x(rbd::Y)          , "actual_CoMY"    );
addVarToCollect( gl.actual_CoMB.x(rbd::X)          , "actual_CoMBX"    );
addVarToCollect( gl.actual_CoMB.x(rbd::Y)          , "actual_CoMBY"    );
addVarToCollect( gl.actual_CoMB.x(rbd::Z)          , "actual_CoMBZ"    );
addVarToCollect( gl.stance_legs[iit::dog::LF]  , "stance_LF"    );
addVarToCollect( gl.stance_legs[iit::dog::RF]  , "stance_RF"    );
addVarToCollect( gl.stance_legs[iit::dog::LH]  , "stance_LH"    );
addVarToCollect( gl.stance_legs[iit::dog::RH]  , "stance_RH"    );
addVarToCollect( gl.des_base_posB.xd(rbd::X)        , "des_baseBXd"      );
addVarToCollect( gl.des_base_posB.xd(rbd::Y)        , "des_baseBYd"      );
addVarToCollect( gl.des_base_posB.xd(rbd::Z)        , "des_baseBZd"      );
addVarToCollect( gl.actual_CoMB.xd(rbd::X)          , "actual_CoMBXd"    );
addVarToCollect( gl.actual_CoMB.xd(rbd::Y)          , "actual_CoMBYd"    );
addVarToCollect( gl.actual_CoMB.xd(rbd::Z)          , "actual_CoMBZd"    );
addVarToCollect( gl.des_height, "des_height"       					 );
addVarToCollect( gl.actual_CoM_height_f, "actual_height"       	 );
addVarToCollect( gl.des_base_orient.x(rbd::X) , "des_base_roll" );
addVarToCollect( gl.des_base_orient.x(rbd::Y) , "des_base_pitch");
addVarToCollect( gl.des_base_orient.x(rbd::Z) , "des_base_yaw"  );
double roll_error = gl.des_base_orient.x(rbd::X) - gl.roll ;
double pitch_error = gl.des_base_orient.x(rbd::Y) - gl.pitch;
double yaw_error = gl.des_base_orient.x(rbd::Z) -gl.yaw;
addVarToCollect( roll_error, "roll_error" );
addVarToCollect(pitch_error , "pitch_error");
addVarToCollect( yaw_error , "yaw_error"  );
bool reachingMotion = ((state_machine == swingleg) && footSpliner[swing_leg_index].isTimeElapsed(taskServoTime));
addVarToCollect( reachingMotion , "reachingMotion"  );


addVarToCollect( gl.des_base_orient.xd(X),"des_base_rolld");
addVarToCollect( gl.des_base_orient.xd(Y),"des_base_pitchd");
addVarToCollect( gl.des_base_orient.xd(Z),"des_base_yawd");
addVarToCollect( gl.roll                      , "roll"               );
addVarToCollect( gl.pitch                     , "pitch"              );
addVarToCollect( gl.yaw                       , "yaw"                );
addVarToCollect( gl.roll_d,"roll_d");
addVarToCollect( gl.pitch_d,"pitch_d");
addVarToCollect( gl.yaw_d,"yaw_d");
addVarToCollect( gl.omega(X),"omegaX");
addVarToCollect( gl.omega(Y),"omegaY");
addVarToCollect( gl.omega(Z),"omegaZ");


addVarToCollect( gl.cycle_time                 , "cycle_time"        );
addVarToCollect( gl.steppingFrequency                 , "steppingFrequency"        );

addVarToCollect( headingSpeed        , "headingSpeed"          );
addVarToCollect( linearSpeedX        , "linearSpeedX"          );
addVarToCollect( linearSpeedY        , "linearSpeedY"          );

addVarToCollect( stab_margin, "stab_margin"       	 );

addVarToCollect( gl.feetForces(0)          , "feetForcesLFx"         );
addVarToCollect( gl.feetForces(1)          , "feetForcesLFy"         );
addVarToCollect( gl.feetForces(2)          , "feetForcesLFz"         );
addVarToCollect( gl.feetForces(3)          , "feetForcesRFx"         );
addVarToCollect( gl.feetForces(4)          , "feetForcesRFy"         );
addVarToCollect( gl.feetForces(5)          , "feetForcesRFz"         );
addVarToCollect( gl.feetForces(6)          , "feetForcesLHx"         );
addVarToCollect( gl.feetForces(7)          , "feetForcesLHy"         );
addVarToCollect( gl.feetForces(8)          , "feetForcesLHz"         );
addVarToCollect( gl.feetForces(9)          , "feetForcesRHx"         );
addVarToCollect( gl.feetForces(10)          , "feetForcesRHy"        );
addVarToCollect( gl.feetForces(11)          , "feetForcesRHz"        );

addVarToCollect( gl.feetForcesBase(0)          , "feetForcesBLFx"         );
addVarToCollect( gl.feetForcesBase(1)          , "feetForcesBLFy"         );
addVarToCollect( gl.feetForcesBase(2)          , "feetForcesBLFz"         );
addVarToCollect( gl.feetForcesBase(3)          , "feetForcesBRFx"         );
addVarToCollect( gl.feetForcesBase(4)          , "feetForcesBRFy"         );
addVarToCollect( gl.feetForcesBase(5)          , "feetForcesBRFz"         );
addVarToCollect( gl.feetForcesBase(6)          , "feetForcesBLHx"         );
addVarToCollect( gl.feetForcesBase(7)          , "feetForcesBLHy"         );
addVarToCollect( gl.feetForcesBase(8)          , "feetForcesBLHz"         );
addVarToCollect( gl.feetForcesBase(9)          , "feetForcesBRHx"         );
addVarToCollect( gl.feetForcesBase(10)          , "feetForcesBRHy"        );
addVarToCollect( gl.feetForcesBase(11)          , "feetForcesBRHz"        );

addVarToCollect( gl.grForces[iit::dog::LF](rbd::Y), "grForcesLFy"       );
addVarToCollect( gl.grForces[iit::dog::RF](rbd::Y), "grForcesRFy"       );
addVarToCollect( gl.grForces[iit::dog::LH](rbd::Y), "grForcesLHy"       );
addVarToCollect( gl.grForces[iit::dog::RH](rbd::Y), "grForcesRHy"       );
addVarToCollect( gl.grForces[iit::dog::LF](rbd::Z), "grForcesLFz"       );
addVarToCollect( gl.grForces[iit::dog::RF](rbd::Z), "grForcesRFz"       );
addVarToCollect( gl.grForces[iit::dog::LH](rbd::Z), "grForcesLHz"       );
addVarToCollect( gl.grForces[iit::dog::RH](rbd::Z), "grForcesRHz"       );
addVarToCollect( gl.grForces[iit::dog::LF](rbd::X), "grForcesLFx"       );
addVarToCollect( gl.grForces[iit::dog::RF](rbd::X), "grForcesRFx"       );
addVarToCollect( gl.grForces[iit::dog::LH](rbd::X), "grForcesLHx"       );
addVarToCollect( gl.grForces[iit::dog::RH](rbd::X), "grForcesRHx"       );
addVarToCollect( state_machine, "state_machine"       				 );
addVarToCollect( state_machine_leg[iit::dog::LF], "state_machineLF"       				 );
addVarToCollect( state_machine_leg[iit::dog::RF], "state_machineRF"       				 );
addVarToCollect( state_machine_leg[iit::dog::LH], "state_machineLH"       				 );
addVarToCollect( state_machine_leg[iit::dog::RH], "state_machineRH"       				 );


addVarToCollect(gl.terrRoll,"terrainRoll");
addVarToCollect(gl.terrPitch,"terrainPitch");

addVarToCollect(gl.max_torquesHAA[iit::dog::LF],"LF_HAAmax");
addVarToCollect(gl.max_torquesHAA[iit::dog::RF],"RF_HAAmax");
addVarToCollect(gl.max_torquesHAA[iit::dog::LH],"LH_HAAmax");
addVarToCollect(gl.max_torquesHAA[iit::dog::RH],"RH_HAAmax");
addVarToCollect(gl.max_torquesHFE[iit::dog::LF],"LF_HFEmax");
addVarToCollect(gl.max_torquesHFE[iit::dog::RF],"RF_HFEmax");
addVarToCollect(gl.max_torquesHFE[iit::dog::LH],"LH_HFEmax");
addVarToCollect(gl.max_torquesHFE[iit::dog::RH],"RH_HFEmax");
addVarToCollect(gl.max_torquesKFE[iit::dog::LF],"LF_KFEmax");
addVarToCollect(gl.max_torquesKFE[iit::dog::RF],"RF_KFEmax");
addVarToCollect(gl.max_torquesKFE[iit::dog::LH],"LH_KFEmax");
addVarToCollect(gl.max_torquesKFE[iit::dog::RH],"RH_KFEmax");

addVarToCollect(gl.min_torquesHAA[iit::dog::LF],"LF_HAAmin");
addVarToCollect(gl.min_torquesHAA[iit::dog::RF],"RF_HAAmin");
addVarToCollect(gl.min_torquesHAA[iit::dog::LH],"LH_HAAmin");
addVarToCollect(gl.min_torquesHAA[iit::dog::RH],"RH_HAAmin");
addVarToCollect(gl.min_torquesHFE[iit::dog::LF],"LF_HFEmin");
addVarToCollect(gl.min_torquesHFE[iit::dog::RF],"RF_HFEmin");
addVarToCollect(gl.min_torquesHFE[iit::dog::LH],"LH_HFEmin");
addVarToCollect(gl.min_torquesHFE[iit::dog::RH],"RH_HFEmin");
addVarToCollect(gl.min_torquesKFE[iit::dog::LF],"LF_KFEmin");
addVarToCollect(gl.min_torquesKFE[iit::dog::RF],"RF_KFEmin");
addVarToCollect(gl.min_torquesKFE[iit::dog::LH],"LH_KFEmin");
addVarToCollect(gl.min_torquesKFE[iit::dog::RH],"RH_KFEmin");



addVarToCollect( des_tau_(iit::dog::LF_HAA)     , "LF_HAA_uff"         );
addVarToCollect( des_tau_(iit::dog::LF_HFE)     , "LF_HFE_uff"         );
addVarToCollect( des_tau_(iit::dog::LF_KFE)     , "LF_KFE_uff"         );
addVarToCollect( des_tau_(iit::dog::RF_HAA)     , "RF_HAA_uff"         );
addVarToCollect( des_tau_(iit::dog::RF_HFE)     , "RF_HFE_uff"         );
addVarToCollect( des_tau_(iit::dog::RF_KFE)     , "RF_KFE_uff"         );
addVarToCollect( des_tau_(iit::dog::LH_HAA)     , "LH_HAA_uff"         );
addVarToCollect( des_tau_(iit::dog::LH_HFE)     , "LH_HFE_uff"         );
addVarToCollect( des_tau_(iit::dog::LH_KFE)     , "LH_KFE_uff"         );
addVarToCollect( des_tau_(iit::dog::RH_HAA)     , "RH_HAA_uff"         );
addVarToCollect( des_tau_(iit::dog::RH_HFE)     , "RH_HFE_uff"         );
addVarToCollect( des_tau_(iit::dog::RH_KFE)     , "RH_KFE_uff"         );

addVarToCollect(tau_(iit::dog::LF_HAA)     , "LF_HAA_load"         );
addVarToCollect(tau_(iit::dog::LF_HFE)     , "LF_HFE_load"         );
addVarToCollect(tau_(iit::dog::LF_KFE)     , "LF_KFE_load"         );
addVarToCollect(tau_(iit::dog::RF_HAA)     , "RF_HAA_load"         );
addVarToCollect(tau_(iit::dog::RF_HFE)     , "RF_HFE_load"         );
addVarToCollect(tau_(iit::dog::RF_KFE)     , "RF_KFE_load"         );
addVarToCollect(tau_(iit::dog::LH_HAA)     , "LH_HAA_load"         );
addVarToCollect(tau_(iit::dog::LH_HFE)     , "LH_HFE_load"         );
addVarToCollect(tau_(iit::dog::LH_KFE)     , "LH_KFE_load"         );
addVarToCollect(tau_(iit::dog::RH_HAA)     , "RH_HAA_load"         );
addVarToCollect(tau_(iit::dog::RH_HFE)     , "RH_HFE_load"         );
addVarToCollect(tau_(iit::dog::RH_KFE)     , "RH_KFE_load"         );

addVarToCollect( q_(iit::dog::LF_HAA)     , "LF_HAA_th"         );
addVarToCollect( q_(iit::dog::LF_HFE)     , "LF_HFE_th"         );
addVarToCollect( q_(iit::dog::LF_KFE)     , "LF_KFE_th"         );
addVarToCollect( q_(iit::dog::RF_HAA)     , "RF_HAA_th"         );
addVarToCollect( q_(iit::dog::RF_HFE)     , "RF_HFE_th"         );
addVarToCollect( q_(iit::dog::RF_KFE)     , "RF_KFE_th"         );
addVarToCollect( q_(iit::dog::LH_HAA)     , "LH_HAA_th"         );
addVarToCollect( q_(iit::dog::LH_HFE)     , "LH_HFE_th"         );
addVarToCollect( q_(iit::dog::LH_KFE)     , "LH_KFE_th"         );
addVarToCollect( q_(iit::dog::RH_HAA)     , "RH_HAA_th"         );
addVarToCollect( q_(iit::dog::RH_HFE)     , "RH_HFE_th"         );
addVarToCollect( q_(iit::dog::RH_KFE)     , "RH_KFE_th"         );

addVarToCollect(  gl.q_max(iit::dog::LF_HAA)     , "LF_HAA_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::LF_HFE)     , "LF_HFE_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::LF_KFE)     , "LF_KFE_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::RF_HAA)     , "RF_HAA_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::RF_HFE)     , "RF_HFE_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::RF_KFE)     , "RF_KFE_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::LH_HAA)     , "LH_HAA_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::LH_HFE)     , "LH_HFE_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::LH_KFE)     , "LH_KFE_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::RH_HAA)     , "RH_HAA_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::RH_HFE)     , "RH_HFE_th_max"         );
addVarToCollect(  gl.q_max(iit::dog::RH_KFE)     , "RH_KFE_th_max"         );

addVarToCollect( gl.q_min(iit::dog::LF_HAA)     , "LF_HAA_th_min"         );
addVarToCollect( gl.q_min(iit::dog::LF_HFE)     , "LF_HFE_th_min"         );
addVarToCollect( gl.q_min(iit::dog::LF_KFE)     , "LF_KFE_th_min"         );
addVarToCollect( gl.q_min(iit::dog::RF_HAA)     , "RF_HAA_th_min"         );
addVarToCollect( gl.q_min(iit::dog::RF_HFE)     , "RF_HFE_th_min"         );
addVarToCollect( gl.q_min(iit::dog::RF_KFE)     , "RF_KFE_th_min"         );
addVarToCollect( gl.q_min(iit::dog::LH_HAA)     , "LH_HAA_th_min"         );
addVarToCollect( gl.q_min(iit::dog::LH_HFE)     , "LH_HFE_th_min"         );
addVarToCollect( gl.q_min(iit::dog::LH_KFE)     , "LH_KFE_th_min"         );
addVarToCollect( gl.q_min(iit::dog::RH_HAA)     , "RH_HAA_th_min"         );
addVarToCollect( gl.q_min(iit::dog::RH_HFE)     , "RH_HFE_th_min"         );
addVarToCollect( gl.q_min(iit::dog::RH_KFE)     , "RH_KFE_th_min"         );


addVarToCollect( qd_(iit::dog::LF_HAA)     , "LF_HAA_thd"         );
addVarToCollect( qd_(iit::dog::LF_HFE)     , "LF_HFE_thd"         );
addVarToCollect( qd_(iit::dog::LF_KFE)     , "LF_KFE_thd"         );
addVarToCollect( qd_(iit::dog::RF_HAA)     , "RF_HAA_thd"         );
addVarToCollect( qd_(iit::dog::RF_HFE)     , "RF_HFE_thd"         );
addVarToCollect( qd_(iit::dog::RF_KFE)     , "RF_KFE_thd"         );
addVarToCollect( qd_(iit::dog::LH_HAA)     , "LH_HAA_thd"         );
addVarToCollect( qd_(iit::dog::LH_HFE)     , "LH_HFE_thd"         );
addVarToCollect( qd_(iit::dog::LH_KFE)     , "LH_KFE_thd"         );
addVarToCollect( qd_(iit::dog::RH_HAA)     , "RH_HAA_thd"         );
addVarToCollect( qd_(iit::dog::RH_HFE)     , "RH_HFE_thd"         );
addVarToCollect( qd_(iit::dog::RH_KFE)     , "RH_KFE_thd"         );

addVarToCollect( des_q_(iit::dog::LF_HAA)     , "LF_HAA_des_th"         );
addVarToCollect( des_q_(iit::dog::LF_HFE)     , "LF_HFE_des_th"         );
addVarToCollect( des_q_(iit::dog::LF_KFE)     , "LF_KFE_des_th"         );
addVarToCollect( des_q_(iit::dog::RF_HAA)     , "RF_HAA_des_th"         );
addVarToCollect( des_q_(iit::dog::RF_HFE)     , "RF_HFE_des_th"         );
addVarToCollect( des_q_(iit::dog::RF_KFE)     , "RF_KFE_des_th"         );
addVarToCollect( des_q_(iit::dog::LH_HAA)     , "LH_HAA_des_th"         );
addVarToCollect( des_q_(iit::dog::LH_HFE)     , "LH_HFE_des_th"         );
addVarToCollect( des_q_(iit::dog::LH_KFE)     , "LH_KFE_des_th"         );
addVarToCollect( des_q_(iit::dog::RH_HAA)     , "RH_HAA_des_th"         );
addVarToCollect( des_q_(iit::dog::RH_HFE)     , "RH_HFE_des_th"         );
addVarToCollect( des_q_(iit::dog::RH_KFE)     , "RH_KFE_des_th"         );

addVarToCollect(des_qd_(iit::dog::LF_HAA)     , "LF_HAA_des_thd"         );
addVarToCollect(des_qd_(iit::dog::LF_HFE)     , "LF_HFE_des_thd"         );
addVarToCollect(des_qd_(iit::dog::LF_KFE)     , "LF_KFE_des_thd"         );
addVarToCollect(des_qd_(iit::dog::RF_HAA)     , "RF_HAA_des_thd"         );
addVarToCollect(des_qd_(iit::dog::RF_HFE)     , "RF_HFE_des_thd"         );
addVarToCollect(des_qd_(iit::dog::RF_KFE)     , "RF_KFE_des_thd"         );
addVarToCollect(des_qd_(iit::dog::LH_HAA)     , "LH_HAA_des_thd"         );
addVarToCollect(des_qd_(iit::dog::LH_HFE)     , "LH_HFE_des_thd"         );
addVarToCollect(des_qd_(iit::dog::LH_KFE)     , "LH_KFE_des_thd"         );
addVarToCollect(des_qd_(iit::dog::RH_HAA)     , "RH_HAA_des_thd"         );
addVarToCollect(des_qd_(iit::dog::RH_HFE)     , "RH_HFE_des_thd"         );
addVarToCollect(des_qd_(iit::dog::RH_KFE)     , "RH_KFE_des_thd"         );

addVarToCollect(gl.footPosDes[iit::dog::LF](X),"footPosDesLFx");
addVarToCollect(gl.footPosDes[iit::dog::LF](Y),"footPosDesLFy");
addVarToCollect(gl.footPosDes[iit::dog::LF](Z),"footPosDesLFz");
addVarToCollect(gl.footPosDes[iit::dog::LH](X),"footPosDesLHx");
addVarToCollect(gl.footPosDes[iit::dog::LH](Y),"footPosDesLHy");
addVarToCollect(gl.footPosDes[iit::dog::LH](Z),"footPosDesLHz");
addVarToCollect(gl.footPosDes[iit::dog::RH](X),"footPosDesRHx");
addVarToCollect(gl.footPosDes[iit::dog::RH](Y),"footPosDesRHy");
addVarToCollect(gl.footPosDes[iit::dog::RH](Z),"footPosDesRHz");
addVarToCollect(gl.footPosDes[iit::dog::RF](X),"footPosDesRFx");
addVarToCollect(gl.footPosDes[iit::dog::RF](Y),"footPosDesRFy");
addVarToCollect(gl.footPosDes[iit::dog::RF](Z),"footPosDesRFz");

addVarToCollect(gl.footPosDesCorrected[iit::dog::LF](X),"footPosDesCorrLFx");
addVarToCollect(gl.footPosDesCorrected[iit::dog::LF](Y),"footPosDesCorrLFy");
addVarToCollect(gl.footPosDesCorrected[iit::dog::LF](Z),"footPosDesCorrLFz");
addVarToCollect(gl.footPosDesCorrected[iit::dog::LH](X),"footPosDesCorrLHx");
addVarToCollect(gl.footPosDesCorrected[iit::dog::LH](Y),"footPosDesCorrLHy");
addVarToCollect(gl.footPosDesCorrected[iit::dog::LH](Z),"footPosDesCorrLHz");
addVarToCollect(gl.footPosDesCorrected[iit::dog::RH](X),"footPosDesCorrRHx");
addVarToCollect(gl.footPosDesCorrected[iit::dog::RH](Y),"footPosDesCorrRHy");
addVarToCollect(gl.footPosDesCorrected[iit::dog::RH](Z),"footPosDesCorrRHz");
addVarToCollect(gl.footPosDesCorrected[iit::dog::RF](X),"footPosDesCorrRFx");
addVarToCollect(gl.footPosDesCorrected[iit::dog::RF](Y),"footPosDesCorrRFy");
addVarToCollect(gl.footPosDesCorrected[iit::dog::RF](Z),"footPosDesCorrRFz");

addVarToCollect(gl.footVelDes[iit::dog::LF](X),"footVelDesLFx");
addVarToCollect(gl.footVelDes[iit::dog::LF](Y),"footVelDesLFy");
addVarToCollect(gl.footVelDes[iit::dog::LF](Z),"footVelDesLFz");
addVarToCollect(gl.footVelDes[iit::dog::LH](X),"footVelDesLHx");
addVarToCollect(gl.footVelDes[iit::dog::LH](Y),"footVelDesLHy");
addVarToCollect(gl.footVelDes[iit::dog::LH](Z),"footVelDesLHz");
addVarToCollect(gl.footVelDes[iit::dog::RH](X),"footVelDesRHx");
addVarToCollect(gl.footVelDes[iit::dog::RH](Y),"footVelDesRHy");
addVarToCollect(gl.footVelDes[iit::dog::RH](Z),"footVelDesRHz");
addVarToCollect(gl.footVelDes[iit::dog::RF](X),"footVelDesRFx");
addVarToCollect(gl.footVelDes[iit::dog::RF](Y),"footVelDesRFy");
addVarToCollect(gl.footVelDes[iit::dog::RF](Z),"footVelDesRFz");

addVarToCollect(gl.footPos[iit::dog::LF](X),"footPosLFx");
addVarToCollect(gl.footPos[iit::dog::LF](Y),"footPosLFy");
addVarToCollect(gl.footPos[iit::dog::LF](Z),"footPosLFz");
addVarToCollect(gl.footPos[iit::dog::LH](X),"footPosLHx");
addVarToCollect(gl.footPos[iit::dog::LH](Y),"footPosLHy");
addVarToCollect(gl.footPos[iit::dog::LH](Z),"footPosLHz");
addVarToCollect(gl.footPos[iit::dog::RH](X),"footPosRHx");
addVarToCollect(gl.footPos[iit::dog::RH](Y),"footPosRHy");
addVarToCollect(gl.footPos[iit::dog::RH](Z),"footPosRHz");
addVarToCollect(gl.footPos[iit::dog::RF](X),"footPosRFx");
addVarToCollect(gl.footPos[iit::dog::RF](Y),"footPosRFy");
addVarToCollect(gl.footPos[iit::dog::RF](Z),"footPosRFz");


addVarToCollect(gl.baseTwist(rbd::LX),"baseTwistLx");
addVarToCollect(gl.baseTwist(rbd::LY),"baseTwistLy");
addVarToCollect(gl.baseTwist(rbd::LZ),"baseTwistLz");
addVarToCollect(gl.baseTwist(rbd::AX),"baseTwistAx");
addVarToCollect(gl.baseTwist(rbd::AY),"baseTwistAy");
addVarToCollect(gl.baseTwist(rbd::AZ),"baseTwistAz");

addVarToCollect(gl.wrenchError(rbd::LX),"wrenchErrorLx");
addVarToCollect(gl.wrenchError(rbd::LY),"wrenchErrorLy");
addVarToCollect(gl.wrenchError(rbd::LZ),"wrenchErrorLz");
addVarToCollect(gl.wrenchError(rbd::AX),"wrenchErrorAx");
addVarToCollect(gl.wrenchError(rbd::AY),"wrenchErrorAy");
addVarToCollect(gl.wrenchError(rbd::AZ),"wrenchErrorAz");

}

}
