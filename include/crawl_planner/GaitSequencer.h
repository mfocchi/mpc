/*
 * GaitSequencer.h
 *
 *  Created on: Nov 20, 2015
 *      Author: mfocchi
 */

#ifndef GAITSEQUENCER_H_
#define GAITSEQUENCER_H_


using namespace iit;
using namespace Eigen;
#include <crawl_planner/timer.h>
#include <iit/commons/dog/leg_data_map.h>

//TODO get generic sequence (e.g. acycle witrh only 3 legs)
class GaitSequencer{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    GaitSequencer(){
     waitForCycleReset[0] = false;
     waitForCycleReset[1] = false;
     waitForCycleReset[2] = false;
     waitForCycleReset[3] = false;



        swing_status = swing_status_old = false;
        speed = 1.0/cycleDuration;
    }
    GaitSequencer(iit::dog::LegID leg1, iit::dog::LegID leg2, iit::dog::LegID leg3, iit::dog::LegID leg4, int taskServoRate){
        foot_sequence[0] = leg1;
        foot_sequence[1] = leg2;
        foot_sequence[2] = leg3;
        foot_sequence[3] = leg4;
        dt = 1.0/(double) taskServoRate;
        dutyFactor[0] = 0.75; dutyFactor[1] = 0.75; dutyFactor[2] = 0.75; dutyFactor[3] = 0.75;
        swing_status = swing_status_old = false;
        speed = 1.0/cycleDuration;

    }
    ~GaitSequencer(){}



    //at startup

    inline void setTaskServoRate(int taskServoRate)
    {

            dt = 1.0/(double) taskServoRate;
//            strideParametrization_notReset = speed*dt;
//            strideParametrization_notReset = speed*dt;
    }
    inline void setSequence(iit::dog::LegID leg1, iit::dog::LegID leg2, iit::dog::LegID leg3, iit::dog::LegID leg4){
            foot_sequence[0] = leg1;
            foot_sequence[1] = leg2;
            foot_sequence[2] = leg3;
            foot_sequence[3] = leg4;
    }
    //at startup
    inline void setOffsetAboutCycleStart(const double offset0, const double offset1, const double offset2, const double offset3){
        offset[0] = offset0;
        offset[1] = offset1;
        offset[2] = offset2;
        offset[3] = offset3;
    }


    //on run
    inline void setTotalCycleDuration(const double totalCycleTime){
        this->cycleDuration = totalCycleTime;
        //set the velocity of the time parametrization
        if (fabs(cycleDuration) < 0.001) {
            this->speed = 0.0;
        } else {
            this->speed = 1.0/cycleDuration;
        }
        //remember to update the cycle durations!
        //TODO check the timer is not active before changing the duration!
        timer[0].setDuration(1.0-dutyFactor[0]);
        timer[1].setDuration(1.0-dutyFactor[1]);
        timer[2].setDuration(1.0-dutyFactor[2]);
        timer[3].setDuration(1.0-dutyFactor[3]);
        //update swing duration
        computeSwingDuration(cycleDuration);
    }


    //dutyFactor <1.0
    inline void setDutyFactor(const double duty0, const double duty1, const double duty2, const double duty3){
        this->dutyFactor[0] = duty0;
        this->dutyFactor[1] = duty1;
        this->dutyFactor[2] = duty2;
        this->dutyFactor[3] = duty3;

        timer[0].setDuration(1.0-dutyFactor[0]);
        timer[1].setDuration(1.0-dutyFactor[1]);
        timer[2].setDuration(1.0-dutyFactor[2]);
        timer[3].setDuration(1.0-dutyFactor[3]);
        timer[0].resetTimer();
        timer[1].resetTimer();
        timer[2].resetTimer();
        timer[3].resetTimer();
        //update swing duration
        computeSwingDuration(cycleDuration);
    }
    //totalCycleTime = 4 legs
    inline void computeSwingDuration(double totalCycleTime)
    {
        swingDuration[0] = totalCycleTime*(1.0-dutyFactor[0]);
        swingDuration[1] = totalCycleTime*(1.0-dutyFactor[1]);
        swingDuration[2] = totalCycleTime*(1.0-dutyFactor[2]);
        swingDuration[3] = totalCycleTime*(1.0-dutyFactor[3]);
    }

    //getters (it should be done every loop)
    inline void updateGaitScheduler(iit::dog::LegDataMap<bool>  & prepareSwinging, bool & detected_switch)
    {
        //updates swing_status var


        strideParametrization += speed*dt; //ring buffer
        strideParametrization_notReset += speed*dt;


        updateSwingStatus(strideParametrization_notReset, strideParametrization);
        //it is fundamental to add a dt to 1.0 because we add it and it would reset too early (e.g. the last leg is not able to set the waitForResetCycle is set too late to true and
        // it skips the last leg swing every two cycles
        if (strideParametrization >=(1.0+speed*dt)){
            strideParametrization = 0.0;
            waitForCycleReset[0]=false;
            waitForCycleReset[1]=false;
            waitForCycleReset[2]=false;
            waitForCycleReset[3]=false;

        }
//        std::cout <<std::setprecision(4) <<strideParametrization<<std::endl;
//        std::cout <<std::setprecision(4) <<strideParametrization_notReset<<std::endl;

        //detect change on lift of, silent this check for swinging legs
        for (int i = 0; i<4; i++){
            stance_switch[i] = (swing_status[i] != swing_status_old[i]) && swing_status[i]; //detect only rising edge
            if (stance_switch[i]) { //it will enter here only once at each trigger
                prepareSwinging[foot_sequence[i]] = true; //start move body in the opposite direction of swing, the swing will be triggered after delay time
                detected_switch = true;
//				if (swingDuration[i]>=2.0)
//					swingDuration[i] = 2.0;
            }
        }
        swing_status_old = swing_status;
    }
    //TODO check the output of this function im not sure
    inline iit::dog::LegDataMap<bool> getSwingLegState(){
        iit::dog::LegDataMap<bool> status;
        for (int i = 0; i<4; i++){
            status[foot_sequence[i]] = swing_status[i];

        }

        return status;
    }

    inline double getSwingDuration(iit::dog::LegID leg){
        for (int i = 0; i<4; i++){
            if (foot_sequence[i] == leg){
                return swingDuration[i];
            }
        }
        return swingDuration[0];
    }

    inline double getStrideParametrization(){
        return strideParametrization;
    }

private:

void updateSwingStatus(double s_notReset, double s);

double speed, strideParametrization = 0.0;
double strideParametrization_notReset = 0.0;
double cycleDuration = 4.0;
double dt = 1.0/250.0;
iit::dog::LegDataMap<double> dutyFactor, offset, swingDuration;
iit::dog::LegDataMap<iit::dog::LegID> foot_sequence;
iit::dog::LegDataMap<Timer> timer;
iit::dog::LegDataMap<bool> swing_status, swing_status_old,  stance_switch, waitForCycleReset; //is 1 when a leg is in swing
};

inline void GaitSequencer::updateSwingStatus(double s_notReset, double s)
{


        for (int i = 0; i<4; i++){
            //set to One this sets the flag timer on when conditions are met
            if (timer[i].resetFlag)
            {
                if (((s > (offset[i]))) && !waitForCycleReset[i]){
                    timer[i].startTimer(s_notReset);
                    swing_status[i] = true;
                }
            } else {

                if (timer[i].isTimeElapsed(s_notReset))
                {
                    swing_status[i] = false;
                    waitForCycleReset[i] = true;
                    timer[i].resetTimer();
//					prt(waitForCycleReset[0])
//					prt(waitForCycleReset[1])
//					prt(waitForCycleReset[2])
//					prt(waitForCycleReset[3])
//					std::cout<<std::endl<<std::endl;
                }
            }
        }


}


#endif /* GAITSEQUENCER_H_ */
