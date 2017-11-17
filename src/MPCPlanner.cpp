/*
 * MPCPlanner.cpp
 *
 *  Created on: Nov 14, 2017
 *      Author: mfocchi
 */
#include <crawl_planner/MPCPlanner.h>

using namespace std;      using namespace Eigen;

#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;


MPCPlanner::MPCPlanner(const double horizon_size, const double Ts, const double gravity){
     this->horizon_size_= horizon_size;
     this->Zu.resize(horizon_size_, horizon_size_);
     this->Zx.resize(horizon_size_, 3);

     this->Xpu.resize(horizon_size_, horizon_size_);
     this->Xpx.resize(horizon_size_, 3);

     this->Xvu.resize(horizon_size_, horizon_size_);
     this->Xvx.resize(horizon_size_, 3);

     this->Xau.resize(horizon_size_, horizon_size_);
     this->Xax.resize(horizon_size_, 3);

     this->Ts  = Ts;
     this->gravity_  = gravity;
//default weights
     weight_R = 1e-06;
     weight_Q = 1.0;
     height_ = 0.5;


    A <<  1,     Ts,  pow(Ts,2)/2,
          0,     1 ,  Ts,
          0,     0 ,  1;

    B << pow(Ts,3)/6,  pow(Ts,2)/2,  Ts;

    Cx << 1 , 0  , 0;

    Cv << 0 , 1  , 0;

    Ca << 0 , 0  , 1;

    Cz << 1 , 0  , -height_/gravity_;

//    //string filename = ros::package::getPath(std::string("crawl_planner")) + "/config/Gd.txt";
//    loadGains(filename);
//
//    initial_state_.resize();
}

MPCPlanner::~MPCPlanner()
{

}

void MPCPlanner::setWeights(double weight_R, double weight_Q){
    this->weight_R = weight_R;
    this->weight_Q = weight_Q;

}

void MPCPlanner::buildMatrix(const Matrix<double, 1,3> C_in, MatrixXd & state_matrix, MatrixXd & input_matrix)
{


    D1 = C_in*A;
    D2 = C_in*B;


    //first fill in Px matrix
    MatrixXd element;
    element = D1;
    //set the first element D1
    state_matrix.row(0) = D1;

    for (int i=1; i<horizon_size_; i++)
    {
        element = element* A; //update
        state_matrix.row(i) = element;
    }
    //prt(Px)

    //second fill in Pu matrix
    //set the diagonal to D2
    input_matrix.diagonal().setConstant(D2);
    //set the first subdiagonal to D1*B
    input_matrix.diagonal(-1).setConstant(D1*B);

    //set the second subdiagonal to D1*A*B
    Matrix<double,1,3> dummy_vec;
    dummy_vec = D1*A;
    input_matrix.diagonal(-2).setConstant(dummy_vec*B);


    //for the other subdiagonals of the toeplitz matrix
    for (int i=3; i<horizon_size_; i++)
    {
        dummy_vec = dummy_vec* A;
        input_matrix.diagonal(-i).setConstant(dummy_vec * B);
    }
//    prt(Pu)
}


//this is the zmp in the next state k+1 with respect to u
//the input u applyes immediately at step k as the initial state but has an influence on k+1
void  MPCPlanner::computeZMPtrajectory(const Vector3d & initial_state_x, const Vector3d & initial_state_y,
                                       const VectorXd & jerk_x, const VectorXd & jerk_y,
                                       VectorXd & zmp_x, VectorXd &zmp_y)
{
    computeZMPtrajectory(initial_state_x, jerk_x, zmp_x);
    computeZMPtrajectory(initial_state_y, jerk_y, zmp_y);
}

void  MPCPlanner::computeZMPtrajectory(const Vector3d & initial_state, const VectorXd & jerk, VectorXd & zmp)
{
    zmp.resize(horizon_size_);
    buildMatrix(Cz,Zx,Zu);
    zmp = Zx*initial_state + Zu*jerk;
}


void MPCPlanner::computeCOMtrajectory( const Vector3d & initial_state_x,  const Vector3d & initial_state_y,
                                       const VectorXd & jerk_x, const VectorXd & jerk_y,
                                       VectorXd & traj_x, VectorXd &traj_y, const state_type state)
{
    computeCOMtrajectory(initial_state_x, jerk_x, traj_x, state);
    computeCOMtrajectory(initial_state_y, jerk_y, traj_y, state);
}

void MPCPlanner::computeCOMtrajectory( const Vector3d & initial_state, const VectorXd & jerk, VectorXd & traj, const state_type state)
{
    traj.resize(horizon_size_);

    switch(state)
    {
        case POSITION:
            buildMatrix(Cx,Xpx,Xpu);
            traj = Xpx*initial_state + Xpu*jerk;

        break;
        case VELOCITY:
            buildMatrix(Cv,Xvx,Xvu);
            traj = Xvx*initial_state + Xvu*jerk;

        break;
        case ACCELERATION:
            buildMatrix(Ca,Xax,Xau);
            traj = Xax*initial_state + Xau*jerk;

        break;
        default:
            break;
    }

}


void MPCPlanner::solveQP(const double actual_height, const Vector3d & initial_state,const  VectorXd & zmp_ref,  VectorXd & jerk_vector)
{
    this->height_ = actual_height;
    //build matrix
    MatrixXd G;G.resize(horizon_size_,horizon_size_);
    VectorXd b;b.resize(horizon_size_,1);
    //update with height
    Cz << 1 , 0  , -height_/gravity_;
    buildMatrix(Cz,Zx,Zu);
    G = Zu.transpose()*Zu + weight_R/weight_Q*MatrixXd::Identity(horizon_size_,horizon_size_);
    b = Zu.transpose()*(Zx*initial_state - zmp_ref);
    jerk_vector = -G.inverse()*b;

}



void MPCPlanner::saveTraj(const std::string finename, const VectorXd & zmp)
{
    double time = 0.0;
    std::ofstream file;
    file.open(finename);
    for (int i=0; i<horizon_size_;i++)
    {
        file<<time <<" ";
        file<< zmp(i) <<" ";
        file <<  std::endl;
        time+=Ts;
    }


    printf("done saving\n");
    file.close();

}


void MPCPlanner::debug()
{

    double size = 6;

    MatrixXd  m;
    m.resize(size,size);m.setZero();

    VectorXd Vdiag;
    //first set the diagonal
    Vdiag.resize(size);
    Vdiag.setConstant(100);
    m.diagonal() =Vdiag;

    //then the subdiagonals of the toeplitz matrix
    for (int i=1; i<size; i++)
    {
        Vdiag.resize(size - i);
        Vdiag.setConstant(i*10);
        prt(Vdiag)
        m.diagonal(-i) =Vdiag;
    }

    cout << "Here is the matrix m:" << endl << m << endl;
    cout << "Here are the coefficients on the 1st super-diagonal and 2nd sub-diagonal of m:" << endl
         << m.diagonal(1).transpose() << endl
         << m.diagonal(-2).transpose() << endl;

}
