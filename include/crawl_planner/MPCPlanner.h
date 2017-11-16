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
        MPCPlanner(const double horizon_size, const double Ts, const double gravity);
        ~MPCPlanner();

        void setWeights(double weight_R, double weight_Q);
        void debug();

        void saveTraj(const std::string finename, const Eigen::VectorXd & zmp);

        void computeZMPtrajectory(const Eigen::Vector3d & initial_state_, const Eigen::VectorXd & jerk, Eigen::VectorXd & zmp_x, Eigen::VectorXd &zmp_y);
        void computeCoMtrajectory(const Eigen::Vector3d & initial_state_, const Eigen::VectorXd & jerk, Eigen::VectorXd & com_x, Eigen::VectorXd &com_y);
        void buildZMPMatrix(const double height);
        void solveQP(const Eigen::Vector3d & initial_state,const  Eigen::VectorXd & zmp_ref,  Eigen::VectorXd & jerk_vector);

    private:



        double Ts;
        double D2;
        Eigen::Matrix<double, 1, 3> D1;
        int horizon_size_;
        Eigen::VectorXd initial_state_;
        double weight_R, weight_Q;
        Eigen::Matrix<double,3,3> A;
        Eigen::Matrix<double,3,1> B;
        Eigen::Matrix<double,1,3> C;
        Eigen::MatrixXd Px, Pu, Xx, Xu;
        double gravity_,height_;
};




#endif /* MPCPLANNER_H_ */
