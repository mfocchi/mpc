/*
 * FootScheduler.h
 *
 *  Created on: Nov 19, 2015
 *      Author: mfocchi
 */

#ifndef FOOTSCHEDULER_H_
#define FOOTSCHEDULER_H_


using namespace iit;
using namespace Eigen;

class FootScheduler{

public:
	typedef iit::dog::LegID FootSequence[4];
	FootScheduler(){}
	FootScheduler(iit::dog::LegID leg1, iit::dog::LegID leg2, iit::dog::LegID leg3, iit::dog::LegID leg4){
		foot_sequence[0] = leg1;
		foot_sequence[1] = leg2;
		foot_sequence[2] = leg3;
		foot_sequence[3] = leg4;
		current_swing = 0;
	}

	iit::dog::LegID getCurrentSwing()
	{
		return foot_sequence[current_swing];
	}

	void setCurrentSwing(iit::dog::LegID start_swing_index)
	{
		for(int i=0; i<4;i++){
			if (foot_sequence[i] == start_swing_index)
			{
				current_swing = i;
				break;
			}
		}

	}

	void setSequence(iit::dog::LegID leg1, iit::dog::LegID leg2, iit::dog::LegID leg3, iit::dog::LegID leg4)
	{
		foot_sequence[0] = leg1;
		foot_sequence[1] = leg2;
		foot_sequence[2] = leg3;
		foot_sequence[3] = leg4;
		current_swing = 0;
	}
	void setSequence(FootSequence new_sequence)
	{
		foot_sequence[0] = new_sequence[0];
		foot_sequence[1] = new_sequence[1];
		foot_sequence[2] = new_sequence[2];
		foot_sequence[3] = new_sequence[3];
		current_swing = 0;
	}
	iit::dog::LegID getOppositeSwing()
	{
		switch(getCurrentSwing()){
			case(iit::dog::LF): return iit::dog::RH;break;
			case(iit::dog::RF): return iit::dog::LH;break;
			case(iit::dog::LH): return iit::dog::RF;break;
			case(iit::dog::RH): return iit::dog::LF;break;
			default: return iit::dog::LF;break;
		}
		return iit::dog::LF;
	}

	iit::dog::LegID getNextSwing()
	{
		int next_swing = current_swing+1;
		if (next_swing == 4){
			return foot_sequence[0];
		}
		else return foot_sequence[next_swing];
	}
    iit::dog::LegID getPreviousSwing()
    {
        int previous_swing = current_swing-1;
        if (previous_swing == -1){
            return foot_sequence[3];
        }
        else return foot_sequence[previous_swing];
    }
	void next()
	{
		int next_swing = current_swing+1;
		if (next_swing == 4) current_swing = 0;
		else  current_swing = next_swing;
	}

private:
	FootSequence foot_sequence;
	int current_swing;
};


#endif /* FOOTSCHEDULER_H_ */
