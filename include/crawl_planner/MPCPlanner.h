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


class MPCPlanner
{

    public:

    struct BoxLimits{
            Eigen::VectorXd max;
            Eigen::VectorXd min;
            void resize(double size) { max.resize(size);
                                       min.resize(size); }
    };

        enum state_type{POSITION=0, VELOCITY, ACCELERATION};
        MPCPlanner(const double horizon_size, const double Ts, const double gravity);
        ~MPCPlanner();

        void setWeights(double weight_R, double weight_Q);
        void debug();

        void saveTraj(const std::string finename, const Eigen::VectorXd & zmp);

        void computeZMPtrajectory(const Eigen::Vector3d & initial_state_x, const Eigen::Vector3d & initial_state_y,
                                 const Eigen::VectorXd & jerk_x, const Eigen::VectorXd & jerk_y,
                                 Eigen::VectorXd & zmp_x, Eigen::VectorXd &zmp_y);
        void computeZMPtrajectory(const Eigen::Vector3d & initial_state_, const Eigen::VectorXd & jerk, Eigen::VectorXd & zmp);

        void computeCOMtrajectory(const Eigen::Vector3d & initial_state_,
                                  const Eigen::VectorXd & jerk,
                                  Eigen::VectorXd & traj,
                                  const state_type state = POSITION);
        void computeCOMtrajectory( const Eigen::Vector3d & initial_state_x,  const Eigen::Vector3d & initial_state_y,
                                   const Eigen::VectorXd & jerk_x, const Eigen::VectorXd & jerk_y,
                                   Eigen::VectorXd & traj_x, Eigen::VectorXd &traj_y, const state_type state = POSITION);
        void buildMatrix(const Eigen::Matrix<double, 1,3> C_in, Eigen::MatrixXd & state_matrix, Eigen::MatrixXd & input_matrix);
        void solveQP(const double actual_height, const Eigen::Vector3d & initial_state,const  Eigen::VectorXd & zmp_ref,  Eigen::VectorXd & jerk_vector);
        void solveQPconstraint(const double actual_height, const Eigen::Vector3d & initial_state,
                               const  BoxLimits & zmpLim,  Eigen::VectorXd & jerk_vector, bool robustnessFlag = false);
        void solveQPconstraint(const double weight_R,const double weight_Q, const double actual_height,
                               const Eigen::Vector3d & initial_state,const  BoxLimits & zmpLim,
                               Eigen::VectorXd & jerk_vector, bool robustnessFlag = false);

        void setHorizonSize(int horizon);
    private:



        double Ts;
        double D2;
        Eigen::Matrix<double, 1, 3> D1;
        int horizon_size_;
        Eigen::VectorXd initial_state_;
        double weight_R, weight_Q;
        Eigen::Matrix<double,3,3> A;
        Eigen::Matrix<double,3,1> B;
        Eigen::Matrix<double,1,3> Cz;
        Eigen::Matrix<double,1,3> Cx;
        Eigen::Matrix<double,1,3> Cv;
        Eigen::Matrix<double,1,3> Ca;
        Eigen::MatrixXd Zx, Zu, Xpx, Xpu, Xvx, Xvu, Xax, Xau;
        double gravity_,height_;
};




#endif /* MPCPLANNER_H_ */
