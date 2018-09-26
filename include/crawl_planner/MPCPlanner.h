/*
 * MPCPlanner.h
 *
 *  Created on: Nov 14, 2017
 *      Author: mfocchi
 */

#ifndef MPCPLANNER_H_
#define MPCPLANNER_H_
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <Eigen/Dense>

#include <crawl_planner/eiquadprog.hpp>
#include <iit/commons/dog/leg_data_map.h>
#include <iit/commons/planning/planning.h> //this is for linecoeffd
#include <iit/rbd/rbd.h>



class MPCPlanner
{

    public:

    struct BoxLimits{
            Eigen::VectorXd max;
            Eigen::VectorXd min;
            void resize(int size) { max.resize(size);
                                       min.resize(size); }
    };
    struct footState{
         Eigen::VectorXd x;
         Eigen::VectorXd y;
         Eigen::VectorXd swing;
         void resize(int size) {
             x.resize(size);
             y.resize(size);
             swing.resize(size);
         }
    };

        enum state_type{POSITION=0, VELOCITY, ACCELERATION};
        MPCPlanner(const double horizon_size, const double Ts, const double gravity);
        ~MPCPlanner();

        void setWeights(double weight_R, double weight_Q, double weight_Qs = 1);
        void debug();

        void saveTraj(const std::string finename, const Eigen::VectorXd & var, bool verbose = true);

        void saveTraj(const std::string finename, const Eigen::VectorXd & var_x, const Eigen::VectorXd & var_y, bool verbose = true);

        void computeZMPtrajectory(const Eigen::Vector3d & initial_state_x, const Eigen::Vector3d & initial_state_y,
                                 const Eigen::VectorXd & jerk_x, const Eigen::VectorXd & jerk_y,
                                 Eigen::VectorXd & zmp_x, Eigen::VectorXd &zmp_y);
        void computeZMPtrajectory(const Eigen::Vector3d & initial_state_, const Eigen::VectorXd & jerk, Eigen::VectorXd & zmp);


        Eigen::Vector3d computeCOMtrajectory( const  Eigen::Vector3d & initial_state, const  Eigen::VectorXd & jerk);

        //compute x_k+1 from x_k
        void  computeCOMupdate(Eigen::Vector3d & actualCom, const double jerk_sample);

        void computeCOMtrajectory(const Eigen::Vector3d & initial_state_,
                                  const Eigen::VectorXd & jerk,
                                  Eigen::VectorXd & traj,
                                  const state_type state = POSITION);
        void computeCOMtrajectory( const Eigen::Vector3d & initial_state_x,  const Eigen::Vector3d & initial_state_y,
                                   const Eigen::VectorXd & jerk_x, const Eigen::VectorXd & jerk_y,
                                   Eigen::VectorXd & traj_x, Eigen::VectorXd &traj_y, const state_type state = POSITION);
        void buildMatrix(const Eigen::Matrix<double, 1,3> C_in, Eigen::MatrixXd & state_matrix, Eigen::MatrixXd & input_matrix, int  size = 1000);
        void solveQP(const double actual_height, const Eigen::Vector3d & initial_state,const  Eigen::VectorXd & zmp_ref,  Eigen::VectorXd & jerk_vector);
        void solveQPconstraint(const double actual_height, const Eigen::Vector3d & initial_state,
                               const  BoxLimits & zmpLim,  Eigen::VectorXd & jerk_vector);
        void solveQPconstraintSlack(const double actual_height, const Eigen::Vector3d & initial_state,
                                    const  BoxLimits & zmpLim,  Eigen::VectorXd & jerk_vector);

        void solveQPConstraintCoupled(const double actual_height, const Eigen::Vector3d & initial_state_x, const Eigen::Vector3d & initial_state_y,
                                      const  Eigen::MatrixXd & A,  const  Eigen::VectorXd & b,  Eigen::VectorXd & jerk_vector_x, Eigen::VectorXd & jerk_vector_y);

        void solveQPConstraintCoupled(const double actual_height,
                                      const Eigen::Vector3d & initial_state_x,
                                      const Eigen::Vector3d & initial_state_y,
                                      const  Eigen::MatrixXd & A,  const  Eigen::VectorXd & b,
                                      const Eigen::Vector2d &targetSpeed,
                                      Eigen::VectorXd & jerk_vector_x,
                                      Eigen::VectorXd & jerk_vector_y,
                                      int replanningWindow = 1000);

        void solveQPConstraintCoupledSlacks(const double actual_height,
                                                  const Eigen::Vector3d & initial_state_x,
                                                  const Eigen::Vector3d & initial_state_y,
                                                  const  Eigen::MatrixXd & A,  const  Eigen::VectorXd & b,
                                                  const Eigen::Vector2d &targetSpeed,
                                                  Eigen::VectorXd & jerk_vector_x,
                                                  Eigen::VectorXd & jerk_vector_y,
                                            int replanningWindow = 1000);


        void  buildPolygonMatrix(const iit::dog::LegDataMap<footState> feetStates, const int start_phase_index,
                                             const int phase_duration, const int horizon_size,
                                             Eigen::MatrixXd & A, Eigen::VectorXd & b, int & number_of_constraints );
        void setHorizonSize(int horizon);
        Eigen::VectorXd makeGaussian(const int length, const int mean, const int stddev);
        Eigen::VectorXd  getConstraintViolation(const iit::dog::LegDataMap<footState> feetStates);
        void  getSlacks(const iit::dog::LegDataMap<footState> feetStates,Eigen::VectorXd & min_slacks, Eigen::VectorXd & avg_slacks);

        void computeSteps(const iit::dog::LegDataMap<double> & initial_feet_x,
                          const iit::dog::LegDataMap<double> & initial_feet_y,
                          const double distance, const int number_of_steps, const int horizon_size,
                          iit::dog::LegDataMap<MPCPlanner::footState> & feetStates,
                          iit::dog::LegDataMap<MPCPlanner::footState> & footHolds,
                          Eigen::MatrixXd & A,  Eigen::VectorXd & b, MPCPlanner & myPlanner);


        void computeSteps(const Eigen::Vector2d & userSpeed,
                          const iit::dog::LegDataMap<double> & initial_feet_x,
                          const iit::dog::LegDataMap<double> & initial_feet_y,
                          const int number_of_steps, const int horizon_size,
                          iit::dog::LegDataMap<MPCPlanner::footState> & feetStates,
                          iit::dog::LegDataMap<MPCPlanner::footState> & footHolds,
                          Eigen::MatrixXd & A,  Eigen::VectorXd & b, MPCPlanner & myPlanner,
                          iit::dog::LegID swing_leg_index,  Eigen::Vector2d initialCoM = Eigen::Vector2d::Zero());
        void printSwing(iit::dog::LegID swing);

        void setHipOffsets(iit::dog::LegDataMap<Eigen::Vector2d> hip_offsets);

        iit::dog::LegDataMap<Eigen::Vector3d> getDummyVars(int number)
        {
            iit::dog::LegDataMap<Eigen::Vector3d> dummy3d;
            switch(number)
            {
            case 1:{

                dummy3d[iit::dog::LF] <<dummy1[iit::dog::LF],0.02;
                dummy3d[iit::dog::RF] <<dummy1[iit::dog::RF],0.02;
                dummy3d[iit::dog::LH] <<dummy1[iit::dog::LH],0.02;
                dummy3d[iit::dog::RH] <<dummy1[iit::dog::RH],0.02;
                }
                break;
            case 2:{
                dummy3d[iit::dog::LF] <<dummy2[iit::dog::LF],0.0;
                dummy3d[iit::dog::RF] <<dummy2[iit::dog::RF],0.0;
                dummy3d[iit::dog::LH] <<dummy2[iit::dog::LH],0.0;
                dummy3d[iit::dog::RH] <<dummy2[iit::dog::RH],0.0;
                }
                break;
            default:
                break;
            }
            return dummy3d;

        }

    private:

        bool debug_mode = false;

        double Ts;
        double D2;
        Eigen::Matrix<double, 1, 3> D1;
        int horizon_size_;
        Eigen::VectorXd initial_state_;
        Eigen::VectorXd all_violations_;
        Eigen::VectorXd slacks;

        double weight_R, weight_Q, weight_Qa, weight_Qs;
        Eigen::Matrix<double,3,3> A;
        Eigen::Matrix<double,3,1> B;
        Eigen::Matrix<double,1,3> Cz;
        Eigen::Matrix<double,1,3> Cx;
        Eigen::Matrix<double,1,3> Cv;
        Eigen::Matrix<double,1,3> Ca;
        Eigen::MatrixXd Zx, Zu, Xpx, Xpu, Xvx, Xvu, Xax, Xau;
        double gravity_,height_;
        std::map<int, std::string> legmap;
        iit::dog::LegDataMap<Eigen::Vector2d> hip_offsets,dummy1, dummy2;

};




#endif /* MPCPLANNER_H_ */
