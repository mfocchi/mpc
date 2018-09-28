/*
 * MPCPlanner.cpp
 *
 *  Created on: Nov 14, 2017
 *      Author: mfocchi
 */
#include <crawl_planner/MPCPlanner.h>
#include <crawl_planner/FootScheduler.h>


using namespace std;
using namespace Eigen;
using namespace iit::dog;
using namespace iit;

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
     weight_R = 1e-06; //used for jerk
     weight_Q = 1.0; //used for velocity and zmp ref
     weight_Qa = 1e-06;
     weight_Qs = 100.0; //used for slacks
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
    slacks.resize(0);

    legmap[iit::dog::LF] = "LF";
    legmap[iit::dog::RF] = "RF";
    legmap[iit::dog::LH] = "LH";
    legmap[iit::dog::RH] = "RH";
}


MPCPlanner::~MPCPlanner()
{

}

void MPCPlanner::setHorizonSize(int horizon_size_)
{
    this->horizon_size_= horizon_size_;
    this->Zu.resize(horizon_size_, horizon_size_);
    this->Zx.resize(horizon_size_, 3);

    this->Xpu.resize(horizon_size_, horizon_size_);
    this->Xpx.resize(horizon_size_, 3);

    this->Xvu.resize(horizon_size_, horizon_size_);
    this->Xvx.resize(horizon_size_, 3);

    this->Xau.resize(horizon_size_, horizon_size_);
    this->Xax.resize(horizon_size_, 3);
}

void MPCPlanner::setWeights(double weight_R, double weight_Q, double weight_Qs){
    this->weight_R = weight_R;
    this->weight_Q = weight_Q;
    this->weight_Qs = weight_Qs;

}

void MPCPlanner::buildMatrix(const Matrix<double, 1,3> C_in, MatrixXd & state_matrix, MatrixXd & input_matrix, int  size)
{
    if (size == 1000)
    {
        size = horizon_size_;
    }
    state_matrix.resize(size,3);
    input_matrix.resize(size,size);

    state_matrix.setZero();
    input_matrix.setZero();

    D1 = C_in*A;
    D2 = C_in*B;


    //first fill in #x matrix
    MatrixXd element;
    element = D1;
    //set the first element D1
    state_matrix.row(0) = D1;

    if (size >1) //fill in only if the size is bigger than 1
    {
        for (int i=1; i<size; i++)
        {
            element = element* A; //update
            state_matrix.row(i) = element;
        }
    }
    //prt(Px)

    //second fill in #u matrix
    //set the diagonal to D2
    input_matrix.diagonal().setConstant(D2);

    if (size >1) //fill in only if the size is bigger than 1
    {
        //set the first subdiagonal to D1*B
        input_matrix.diagonal(-1).setConstant(D1*B);

        //set the second subdiagonal to D1*A*B
        Matrix<double,1,3> dummy_vec;
        dummy_vec = D1*A;
        input_matrix.diagonal(-2).setConstant(dummy_vec*B);


        //for the other subdiagonals of the toeplitz matrix
        for (int i=3; i<size; i++)
        {
            dummy_vec = dummy_vec* A;
            input_matrix.diagonal(-i).setConstant(dummy_vec * B);
        }
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
    zmp.resize(jerk.size());
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
//this outputs just the last element
Vector3d MPCPlanner::computeCOMtrajectory( const Vector3d & initial_state, const VectorXd & jerk)
{
    VectorXd traj_x, traj_xd, traj_xdd;
    computeCOMtrajectory(initial_state, jerk, traj_x, POSITION);
    computeCOMtrajectory(initial_state, jerk, traj_xd, VELOCITY);
    computeCOMtrajectory(initial_state, jerk, traj_xdd, ACCELERATION);
    Vector3d last_state;
    last_state << traj_x.tail(1), traj_xd.tail(1), traj_xdd.tail(1);
    return last_state;
}

void MPCPlanner::computeCOMtrajectory( const Vector3d & initial_state, const VectorXd & jerk, VectorXd & traj, const state_type state)
{
    traj.resize(jerk.size());
    switch(state)
    {
        case POSITION:
            buildMatrix(Cx,Xpx,Xpu, jerk.size());
            traj = Xpx*initial_state + Xpu*jerk;

        break;
        case VELOCITY:
            buildMatrix(Cv,Xvx,Xvu, jerk.size());
            traj = Xvx*initial_state + Xvu*jerk;

        break;
        case ACCELERATION:
            buildMatrix(Ca,Xax,Xau, jerk.size());
            traj = Xax*initial_state + Xau*jerk;

        break;
        default:
            break;
    }

}

//1 D - no constraints, tries to track a zmp ref
void MPCPlanner::solveQP(const double actual_height, const Vector3d & initial_state,const  VectorXd & zmp_ref,  VectorXd & jerk_vector)
{
    this->height_ = actual_height;
    //build matrix
    MatrixXd G;G.resize(horizon_size_,horizon_size_);
    VectorXd b;b.resize(horizon_size_,1);
    jerk_vector.resize(horizon_size_);
    //update with height
    Cz << 1 , 0  , -height_/gravity_;
    buildMatrix(Cz,Zx,Zu);
    G = Zu.transpose()*Zu + weight_R/weight_Q*MatrixXd::Identity(horizon_size_,horizon_size_);
    b = Zu.transpose()*(Zx*initial_state - zmp_ref);
    jerk_vector = -G.inverse()*b;

}

//1 D - this enables to set only BOX constraints, no zmp reference so you need to set only R
void MPCPlanner::solveQPconstraint(const double actual_height, const Vector3d & initial_state,const  BoxLimits & zmpLim,  VectorXd & jerk_vector)
{
    this->height_ = actual_height;

    Eigen::MatrixXd GQ, CI, CE; //A is used for computing wrencherror
    Eigen::VectorXd g0, ce0, ci0;
    //build matrix
    GQ.resize(horizon_size_,horizon_size_);
    g0.resize(horizon_size_,1);
    CI.resize(2*horizon_size_,horizon_size_); //max /min for all horizon
    ci0.resize(2*horizon_size_);
    jerk_vector.resize(horizon_size_);
    //update with height
    Cz << 1 , 0  , -height_/gravity_;
    buildMatrix(Cz,Zx,Zu);


    //Finds x that minimizes xddd^T * R * xddd only the jerk is optimized
    GQ.setIdentity(); GQ*=weight_R;
    g0.setZero();


    //no equality constraints
    CE.resize(0,0);ce0.resize(0);

    //inequality 2*N min <zmp < max


    //first N min constraint -- zmp  >= min => zmp - min >0 => Z_x x0 + Zu *xddd - min >0 =>   Zu *xddd + (Z_x*x0  - min) >0
    CI.block(0, 0, horizon_size_,horizon_size_) = Zu;
    ci0.segment(0,horizon_size_) = Zx*initial_state - zmpLim.min;
    //N max constraint -- zmp  <= max => -zmp > -max => -zmp + max >0 => -Z_x x0 - Zu *xddd +max >0 => - Zu *xddd  + (max -Z_x x0)    >0
    CI.block(horizon_size_, 0, horizon_size_,horizon_size_) = -Zu;
    ci0.segment(horizon_size_,horizon_size_) = zmpLim.max-Zx*initial_state;

//
//    prt(GQ)
//    prt(CI)
//    prt(ci0.transpose())

//    min 0.5 * x G x + g0 x
//    s.t.
//        CE^T x + ce0 = 0
//        CI^T x + ci0 >= 0
//
//     The matrix and vectors dimensions are as follows:
//         G: n * n
//            g0: n
//
//            CE: n * p
//         ce0: p
//
//          CI: n * m
//       ci0: m
//
//         x: n
    double result = Eigen::solve_quadprog(GQ, g0, CE.transpose(), ce0, CI.transpose(), ci0, jerk_vector);
    if(result == std::numeric_limits<double>::infinity())
        {cout<<"couldn't find a feasible solution"<<endl;}

}

//1 D - box constraints tries to stay away from box constraints thanks to slacks (no zmp ref given)
void MPCPlanner::solveQPconstraintSlack(const double actual_height, const Vector3d & initial_state,const  BoxLimits & zmpLim,  VectorXd & jerk_vector)
{
    this->height_ = actual_height;

    Eigen::MatrixXd GQ, CI, CE; //A is used for computing wrencherror
    Eigen::VectorXd g0, ce0, ci0, solution;
    //build matrix

    //1 -old using 1 slack for both  max min constraints - gives a tradeoff
//    //we have both jerk (N) and slack vars (N)
//    GQ.resize(2*horizon_size_,2*horizon_size_);
//    g0.resize(2*horizon_size_,1);g0.setZero();
//    CI.resize(3*horizon_size_,2*horizon_size_); CI.setZero();//max /min for all horizon
//    ci0.resize(3*horizon_size_);
//    jerk_vector.resize(horizon_size_);
//    solution.resize(2*horizon_size_); //jerk in \R^N w in R^{N}
//    //update with height
//    Cz << 1 , 0  , -height_/gravity_;
//    buildMatrix(Cz,Zx,Zu);

//    //GQ shoud be positive definite
//    GQ.setIdentity();
//    GQ.block(0,0, horizon_size_,horizon_size_)*=weight_R; //jerk part
//    GQ.block(horizon_size_,horizon_size_, horizon_size_,horizon_size_)*=weight_Qs; //slack part

//    //the lienar part comes from slacks   ones(1000)^T*Q*w
//    g0.segment(horizon_size_,horizon_size_).setConstant(1000*weight_Qs);
//    //no equality constraints
//    CE.resize(0,0);ce0.resize(0);

//    //A*x+b + w >=0 / w<0 for robustness
//    //inequality (2*N) min <zmp < max

//    //first N min constraint -- zmp  >= min => zmp - min + w > 0  => Z_x x0 + Zu *xddd - min + w>0 =>   Zu *xddd + (Z_x*x0  - min) + w  >0
//    // [Zu | Inxn] *[xddd/w]  + (Z_x*x0  - min) >0
//    CI.block(0, 0, horizon_size_,horizon_size_) = Zu;
//    CI.block(0, horizon_size_, horizon_size_,horizon_size_) = MatrixXd::Identity(horizon_size_,horizon_size_);
//    ci0.segment(0,horizon_size_) = Zx*initial_state - zmpLim.min;
//    //N max constraint -- zmp  <= max => -zmp > -max => -zmp + max + w>0 => -Z_x x0 - Zu *xddd +max + w>0 => - Zu *xddd  + (max -Z_x x0)  +w >0
//    //[-Zu | Inxn] *[xddd/w]   + (max -Z_x x0)    >0
//    CI.block(horizon_size_, 0, horizon_size_,horizon_size_) = -Zu;
//    CI.block(horizon_size_, horizon_size_, horizon_size_,horizon_size_) = MatrixXd::Identity(horizon_size_,horizon_size_);
//    ci0.segment(horizon_size_,horizon_size_) = zmpLim.max-Zx*initial_state;

//    //inequality (1*N) w<=0 => -w>0
//    CI.block(2*horizon_size_, horizon_size_, horizon_size_,horizon_size_)= -MatrixXd::Identity(horizon_size_,horizon_size_);
//    ci0.segment(2*horizon_size_,horizon_size_).setZero();

    //2- new using 1 slack for each constraints
    //we have both jerk (N) and slack vars (2N)
    int number_of_slacks = 2*horizon_size_;
    slacks.resize(number_of_slacks);
    GQ.resize(horizon_size_+number_of_slacks,horizon_size_+number_of_slacks);
    g0.resize(horizon_size_+number_of_slacks,1);g0.setZero();
    CI.resize(2*horizon_size_+number_of_slacks,horizon_size_+number_of_slacks);
    CI.setZero();//max /min for all horizon
    ci0.resize(2*horizon_size_+number_of_slacks);
    ci0.setZero();
    jerk_vector.resize(horizon_size_);
    solution.resize(horizon_size_+number_of_slacks); //jerk in \R^N w in R^{2N}
    //update with height
    Cz << 1 , 0  , -height_/gravity_;
    buildMatrix(Cz,Zx,Zu);

    //GQ shoud be positive definite
    GQ.setIdentity();
    GQ.block(0,0, horizon_size_,horizon_size_)*=weight_R; //jerk part
    GQ.block(horizon_size_,horizon_size_,number_of_slacks,number_of_slacks)*=weight_Qs; //slack part

    //the lienar part comes from slacks   ones(1000)^T*Q*w
    g0.segment(horizon_size_,number_of_slacks).setConstant(1000*weight_Qs);
    //no equality constraints
    CE.resize(0,0);ce0.resize(0);

    //A*x+b + w >=0 / w<0 for robustness
    //inequality (2*N) min <zmp < max

    //first N min constraint -- zmp  >= min => zmp - min + w > 0  => Z_x x0 + Zu *xddd - min + w>0 =>   Zu *xddd + (Z_x*x0  - min) + w  >0
    // [Zu | Inxn] *[xddd/w]  + (Z_x*x0  - min) >0
    CI.block(0, 0, horizon_size_,horizon_size_) = Zu;
    ci0.segment(0,horizon_size_) = Zx*initial_state - zmpLim.min;

    //N max constraint -- zmp  <= max => -zmp > -max => -zmp + max + w>0 => -Z_x x0 - Zu *xddd +max + w>0 => - Zu *xddd  + (max -Z_x x0)  +w >0
    //[-Zu | Inxn] *[xddd/w]   + (max -Z_x x0)    >0
    CI.block(horizon_size_, 0, horizon_size_,horizon_size_) = -Zu;
    ci0.segment(horizon_size_,horizon_size_) = zmpLim.max-Zx*initial_state;

    //slack part of the inequalities
    CI.block(0, horizon_size_, number_of_slacks,number_of_slacks) = MatrixXd::Identity(number_of_slacks,number_of_slacks);
    //inequality (1*N) w<=0 => -w>0
    CI.block(2*horizon_size_, horizon_size_, number_of_slacks,number_of_slacks)= -MatrixXd::Identity(number_of_slacks,number_of_slacks);
    ci0.segment(2*horizon_size_,number_of_slacks).setZero();

//
//    prt(GQ)
//    prt(CI)
//    prt(ci0.transpose())

//    min 0.5 * x G x + g0 x
//    s.t.
//        CE^T x + ce0 = 0
//        CI^T x + ci0 >= 0
//
//     The matrix and vectors dimensions are as follows:
//         G: n * n
//            g0: n
//
//            CE: n * p
//         ce0: p
//
//          CI: n * m
//       ci0: m
//
//         x: n
    double result = Eigen::solve_quadprog(GQ, g0, CE.transpose(), ce0, CI.transpose(), ci0, solution);
    if(result == std::numeric_limits<double>::infinity())
        {cout<<"couldn't find a feasible solution"<<endl;}
    else {
        prt(solution.segment(horizon_size_,horizon_size_).transpose()) //slacks
        jerk_vector = solution.segment(0,horizon_size_);
    }
}

//2 D - using polygon constraints without velocity optimization, set only R
void MPCPlanner::solveQPConstraintCoupled(const double actual_height,
                                          const Eigen::Vector3d & initial_state_x,
                                          const Eigen::Vector3d & initial_state_y,
                                          const  MatrixXd & A,  const  VectorXd & b,
                                          Eigen::VectorXd & jerk_vector_x, Eigen::VectorXd & jerk_vector_y)
{

    this->height_ = actual_height;
    Eigen::MatrixXd GQ, CI, CE, Zuc, Zxc, Zyc; //A is used for computing wrencherror
    Eigen::VectorXd g0, ce0, ci0, solution;
    //build matrix
    GQ.resize(2*horizon_size_,2*horizon_size_);
    g0.resize(2*horizon_size_,1);
    solution.resize(2*horizon_size_);

    CI.resize(b.size(),2*horizon_size_); //max /min for all horizon
    ci0.resize(b.size());

    jerk_vector_x.resize(horizon_size_);
    jerk_vector_y.resize(horizon_size_);
    //update with height
    Cz << 1 , 0  , -height_/gravity_;
    buildMatrix(Cz,Zx,Zu);


    //Finds x that minimizes xddd^T * Q * xddd where xddd = [xddd_x, xddd_y]
    GQ.setIdentity(); GQ*=weight_R;
    g0.setZero();

    //no equality constraints
    CE.resize(0,0);ce0.resize(0);

    //inequalities
    //Zuc is block diagonal 2Nx2N
    Zuc.resize(2*horizon_size_,2*horizon_size_); Zuc.setZero();
    Zuc.block(0,0,horizon_size_,horizon_size_) = Zu;
    Zuc.block(horizon_size_,horizon_size_,horizon_size_,horizon_size_) = Zu;

//    //Zxc, Zyc are column matrix
    Zxc.resize(2*horizon_size_,3);Zxc.setZero();
    Zxc.block(0,0,horizon_size_,3) = Zx;
    Zyc.resize(2*horizon_size_,3);Zyc.setZero();
    Zyc.block(horizon_size_,0,horizon_size_,3) = Zx;
//

    CI = A*Zuc;
    ci0 = b + A*(Zxc*initial_state_x + Zyc*initial_state_y);


    double result = Eigen::solve_quadprog(GQ, g0, CE.transpose(), ce0, CI.transpose(), ci0, solution);
    if(result == std::numeric_limits<double>::infinity())
        {cout<<"couldn't find a feasible solution"<<endl;}
    else {
        //prt(solution.transpose())
        jerk_vector_x = solution.segment(0,horizon_size_);
        jerk_vector_y = solution.segment(horizon_size_,horizon_size_);
    }

    //compute violation is a vector of size number_of_constraints, if I evaulate the column for each time sample I can get an idea of which constraint is getting close to zero

    all_violations_ =CI*solution + ci0;
    //prt((CI*solution + ci0).transpose())

}

//2 D - using polygon constraints with velocity optimization, set R for jerk and Q for velocity
void MPCPlanner::solveQPConstraintCoupled(const double actual_height,
                                          const Eigen::Vector3d & initial_state_x,
                                          const Eigen::Vector3d & initial_state_y,
                                          const  MatrixXd & A,  const  VectorXd & b,
                                          const Eigen::Vector2d &targetSpeed,
                                          Eigen::VectorXd & jerk_vector_x, Eigen::VectorXd & jerk_vector_y, int replanningWindow)
{


    this->height_ = actual_height;
    Eigen::MatrixXd GQ, CI, CE, Zuc, Zxc, Zyc; //A is used for computing wrencherror
    Eigen::VectorXd g0, ce0, ci0, solution;
    //build matrix
    GQ.resize(2*horizon_size_,2*horizon_size_);
    g0.resize(2*horizon_size_,1);
    solution.resize(2*horizon_size_);

    CI.resize(b.size(),2*horizon_size_); //max /min for all horizon
    ci0.resize(b.size());

    jerk_vector_x.resize(horizon_size_);
    jerk_vector_y.resize(horizon_size_);
    //update with height
    Cz << 1 , 0  , -height_/gravity_;
    buildMatrix(Cz,Zx,Zu);


    //Finds x that minimizes xddd^T * Q * xddd where xddd = [xddd_x, xddd_y]
    g0.setZero();

    //add the jerk term
    Eigen::MatrixXd Gjerk;
    Gjerk.resize(2*horizon_size_,2*horizon_size_);
    Gjerk.setIdentity();
    Gjerk*=weight_R;


    //add the velocity term (set a constraint only on the last value of velocity
    int window = horizon_size_;//horizon_size_ //is the window where you enforce the velocity is good to set the whole horizon otherwise you have discontinuities
    buildMatrix(Cv,Xvx,Xvu);
    MatrixXd weightQv, Gv;
    weightQv.resize(horizon_size_,horizon_size_);
    if (replanningWindow ==1000)
    {
        //this tries to achieve the desired speed at the end of the horizon
        //prt("usig constant  weighting")
        weightQv.setIdentity();
        weightQv  *= weight_Q*horizon_size_/window;//scale the importance with window size to make trhe costs comparable
    } else{
        //this tries to achieve the desired speed at the end of the replanning window
        //prt("usig importance weighting")
        //importance weight, make gaussian wise most important the weights at the end of the replanning window
        weightQv = makeGaussian(horizon_size_, replanningWindow, replanningWindow).asDiagonal();
        weightQv  *= weight_Q*horizon_size_; //to use the same weights cause the gaussian  integral is 1 and not horizon_size (e.g. as it would be as we have all ones)
    }

    VectorXd selectionVector;
    MatrixXd selectionMatrix;
    selectionVector.resize(horizon_size_);
    selectionVector.setZero();
    selectionVector.segment(horizon_size_-window, window) = VectorXd::Ones(window);
    selectionMatrix.resize(horizon_size_,horizon_size_);
    selectionMatrix = selectionVector.asDiagonal();
    Gv.resize(2*horizon_size_,2*horizon_size_);Gv.setZero();
    //to select from a matrix I need to left and right multiply for the selection matrix
    Gv.block(0,0,horizon_size_,horizon_size_) = selectionMatrix* Xvu.transpose()*weightQv*Xvu *selectionMatrix;
    Gv.block(horizon_size_,horizon_size_,horizon_size_,horizon_size_) = selectionMatrix * Xvu.transpose()*weightQv*Xvu *selectionMatrix;
    //prt(Gv)

    Eigen::VectorXd bx, by, desiredSpeedVector,gv;
    desiredSpeedVector.resize(horizon_size_);
    bx.resize(horizon_size_);
    by.resize(horizon_size_);
    bx = Xvx*initial_state_x - desiredSpeedVector.setConstant(targetSpeed(rbd::X));
    by = Xvx*initial_state_y - desiredSpeedVector.setConstant(targetSpeed(rbd::Y));
    gv.resize(2*horizon_size_,1);gv.setZero();
    //to select from a row vector I need to right multiply for the selection matrix
    gv.segment(0,horizon_size_) = bx.transpose()*weightQv*Xvu*selectionMatrix;
    gv.segment(horizon_size_,horizon_size_) = by.transpose()*weightQv*Xvu*selectionMatrix;
    //prt(gv.transpose())

    //add accel term (the weight is the same as R)
    //add the velocity term (set a constraint only on the last value of velocity
    buildMatrix(Ca,Xax,Xau);
    MatrixXd  Ga;
    Ga.resize(2*horizon_size_,2*horizon_size_);Ga.setZero();
    Ga.block(0,0,horizon_size_,horizon_size_) = Xau.transpose()*weight_Qa*Xau;
    Ga.block(horizon_size_,horizon_size_,horizon_size_,horizon_size_) = Xau.transpose()*weight_Qa*Xau;

////////////

    GQ = Gjerk + Gv;// + Ga; //Ga does not make difference
    g0 = gv; //gjerk ga are zero cause we want to just miniminze their norm

    ///////////////////////////////////////////////////
    //no equality constraints
    CE.resize(0,0);ce0.resize(0);

    //inequalities
    //Zuc is block diagonal
    Zuc.resize(2*horizon_size_,2*horizon_size_); Zuc.setZero();
    Zuc.block(0,0,horizon_size_,horizon_size_) = Zu;
    Zuc.block(horizon_size_,horizon_size_,horizon_size_,horizon_size_) = Zu;

//    //Zxc, Zyc are columns
    Zxc.resize(2*horizon_size_,3);Zxc.setZero();
    Zxc.block(0,0,horizon_size_,3) = Zx;
    Zyc.resize(2*horizon_size_,3);Zyc.setZero();
    Zyc.block(horizon_size_,0,horizon_size_,3) = Zx;
//

    CI = A*Zuc;
    ci0 = b + A*(Zxc*initial_state_x + Zyc*initial_state_y);


    double result = Eigen::solve_quadprog(GQ, g0, CE.transpose(), ce0, CI.transpose(), ci0, solution);
    if(result == std::numeric_limits<double>::infinity())
        {cout<<"couldn't find a feasible solution"<<endl;}
    else {
        //prt(solution.transpose())
        jerk_vector_x = solution.segment(0,horizon_size_);
        jerk_vector_y = solution.segment(horizon_size_,horizon_size_);
    }
    //costs
    if (debug_mode)
    {
        std::cout<<"jerk cost Cj: "<<0.5*solution.transpose()*Gjerk*solution <<std::endl;
        std::cout<<"acc cost Ca: "<<0.5*solution.transpose()*Ga*solution <<std::endl ;
        std::cout<<"vel cost Cv: "<<0.5*solution.transpose()*Gv*solution + gv.transpose()*solution<<std::endl;
    }
    //compute violation is a vector of size number_of_constraints, if I evaulate the column for each time sample I can get an idea of which constraint is getting close to zero

    all_violations_ =CI*solution + ci0;
    //prt((CI*solution + ci0).transpose())

}


//2 D - using polygon constraints with velocity optimization, set R for jerk and Q for velocity, with slacks for zmp robustness (e.g. stay away from the polygon)
void MPCPlanner::solveQPConstraintCoupledSlacks(const double actual_height,
                                          const Eigen::Vector3d & initial_state_x,
                                          const Eigen::Vector3d & initial_state_y,
                                          const  MatrixXd & A,  const  VectorXd & b,
                                          const Eigen::Vector2d &targetSpeed,
                                          Eigen::VectorXd & jerk_vector_x,
                                          Eigen::VectorXd & jerk_vector_y,
                                          int replanningWindow)
{


    this->height_ = actual_height;
    int number_of_poly_constraints = b.size();
    Eigen::MatrixXd GQ, CI, CE, Zuc, Zxc, Zyc; //A is used for computing wrencherror
    Eigen::VectorXd g0, ce0, ci0, solution;
    //build matrix
    //state is jexk X, Y and a number of slack equal to the constraints
    GQ.resize(2*horizon_size_+number_of_poly_constraints,2*horizon_size_+number_of_poly_constraints); GQ.setZero();
    g0.resize(2*horizon_size_+number_of_poly_constraints,1); g0.setZero();

    //with slacks
    CI.resize(2*number_of_poly_constraints,2*horizon_size_+number_of_poly_constraints);CI.setZero(); //max /min for all horizon
    ci0.resize(2*number_of_poly_constraints);    ci0.setZero();
    //outputs
    solution.resize(2*horizon_size_+number_of_poly_constraints);
    jerk_vector_x.resize(horizon_size_);
    jerk_vector_y.resize(horizon_size_);
        slacks.resize(number_of_poly_constraints);
    //update with height
    Cz << 1 , 0  , -height_/gravity_;
    buildMatrix(Cz,Zx,Zu);


    //Finds x that minimizes xddd^T * Q * xddd where xddd + (-1000 - w)^T Qs(-1000 - w) = [xddd_x, xddd_y, w]
    //1 - add the jerk term
    Eigen::MatrixXd Gjerk;
    Gjerk.resize(2*horizon_size_,2*horizon_size_);
    Gjerk.setIdentity();
    Gjerk*=weight_R;

    //2 - add the velocity term (set a constraint only on the last value of velocity
    int window = horizon_size_;//horizon_size_ //is the window where you enforce the velocity is good to set the whole horizon otherwise you have discontinuities
    buildMatrix(Cv,Xvx,Xvu);
    MatrixXd weightQv, Gv;
    weightQv.resize(horizon_size_,horizon_size_);
    if (replanningWindow ==1000)
    {
        //this tries to achieve the desired speed at the end of the horizon
        //prt("usig constant  weighting")
        weightQv.setIdentity();
        weightQv  *= weight_Q*horizon_size_/window;//scale the importance with window size to make trhe costs comparable
    } else{
        //this tries to achieve the desired speed at the end of the replanning window
        //prt("usig importance weighting")
        //importance weight, make gaussian wise most important the weights at the end of the replanning window
        weightQv = makeGaussian(horizon_size_, replanningWindow, replanningWindow).asDiagonal();
        weightQv  *= weight_Q*horizon_size_; //to use the same weights cause the gaussian  integral is 1 and not horizon_size (e.g. as it would be as we have all ones)
    }

    VectorXd selectionVector;
    MatrixXd selectionMatrix;
    selectionVector.resize(horizon_size_);
    selectionVector.setZero();
    selectionVector.segment(horizon_size_-window, window) = VectorXd::Ones(window);
    selectionMatrix.resize(horizon_size_,horizon_size_);
    selectionMatrix = selectionVector.asDiagonal();
    Gv.resize(2*horizon_size_,2*horizon_size_);Gv.setZero();
    //to select from a matrix I need to left and right multiply for the selection matrix
    Gv.block(0,0,horizon_size_,horizon_size_) = selectionMatrix* Xvu.transpose()*weightQv*Xvu *selectionMatrix;
    Gv.block(horizon_size_,horizon_size_,horizon_size_,horizon_size_) = selectionMatrix * Xvu.transpose()*weightQv*Xvu *selectionMatrix;
    //prt(Gv)

    Eigen::VectorXd bx, by, desiredSpeedVector,gv;
    desiredSpeedVector.resize(horizon_size_);
    bx.resize(horizon_size_);
    by.resize(horizon_size_);
    bx = Xvx*initial_state_x - desiredSpeedVector.setConstant(targetSpeed(rbd::X));
    by = Xvx*initial_state_y - desiredSpeedVector.setConstant(targetSpeed(rbd::Y));
    gv.resize(2*horizon_size_,1);gv.setZero();
    //to select from a row vector I need to right multiply for the selection matrix
    gv.segment(0,horizon_size_) = bx.transpose()*weightQv*Xvu*selectionMatrix;
    gv.segment(horizon_size_,horizon_size_) = by.transpose()*weightQv*Xvu*selectionMatrix;
    //prt(gv.transpose())

    //3 - add accel term
    //add the velocity term (set a constraint only on the last value of velocity
    buildMatrix(Ca,Xax,Xau);
    MatrixXd  Ga;
    Ga.resize(2*horizon_size_,2*horizon_size_);Ga.setZero();
    Ga.block(0,0,horizon_size_,horizon_size_) = Xau.transpose()*weight_Qa*Xau;
    Ga.block(horizon_size_,horizon_size_,horizon_size_,horizon_size_) = Xau.transpose()*weight_Qa*Xau;


////////////

    GQ.block(0,0,2*horizon_size_,2*horizon_size_) = Gjerk+ Gv;// + Ga; //Ga does not make difference
    g0.segment(0,2*horizon_size_) = gv; //gjerk ga are zero cause we want to just miniminze their norm

    //add slack part to the cost function that starts at 2N,2N
    //GQ shoud be positive definite

    GQ.block(2*horizon_size_,2*horizon_size_, number_of_poly_constraints,number_of_poly_constraints) = weight_Qs*MatrixXd::Identity(number_of_poly_constraints,number_of_poly_constraints);
    //the linear part comes from slacks   ones(1000)^T*Qs*w
    g0.segment(2*horizon_size_,number_of_poly_constraints) = MatrixXd::Ones(1,number_of_poly_constraints) *1000*weight_Qs;

    ///////////////////////////////////////////////////
    //no equality constraints
    CE.resize(0,0);ce0.resize(0);

    //inequalities
    //Zuc is block diagonal
    Zuc.resize(2*horizon_size_,2*horizon_size_); Zuc.setZero();
    Zuc.block(0,0,horizon_size_,horizon_size_) = Zu;
    Zuc.block(horizon_size_,horizon_size_,horizon_size_,horizon_size_) = Zu;

//    //Zxc, Zyc are columns
    Zxc.resize(2*horizon_size_,3);Zxc.setZero();
    Zxc.block(0,0,horizon_size_,3) = Zx;
    Zyc.resize(2*horizon_size_,3);Zyc.setZero();
    Zyc.block(horizon_size_,0,horizon_size_,3) = Zx;

    CI.block(0,0, number_of_poly_constraints, 2*horizon_size_) = A*Zuc;
    ci0.segment(0,number_of_poly_constraints) = b + A*(Zxc*initial_state_x + Zyc*initial_state_y);

    //ad slack part to the inequalities
    //A*x+b + w >=0 / w<0 for robustness
    //robustness
    CI.block(0, 2*horizon_size_, number_of_poly_constraints, number_of_poly_constraints) = MatrixXd::Identity(number_of_poly_constraints,number_of_poly_constraints);
    //negativity of slacks
    CI.block(number_of_poly_constraints, 2*horizon_size_, number_of_poly_constraints, number_of_poly_constraints) = -MatrixXd::Identity(number_of_poly_constraints,number_of_poly_constraints);


    double result = Eigen::solve_quadprog(GQ, g0, CE.transpose(), ce0, CI.transpose(), ci0, solution);
    if(result == std::numeric_limits<double>::infinity())
        {cout<<"couldn't find a feasible solution"<<endl;}
    else {
        //prt(solution.transpose())
        slacks = solution.segment(2*horizon_size_,number_of_poly_constraints);
        std::cout<<"Slacks: " << slacks.transpose() << std::endl;//slacks
        jerk_vector_x = solution.segment(0,horizon_size_);
        jerk_vector_y = solution.segment(horizon_size_,horizon_size_);
    }
    //costs
    if (debug_mode)
    {

        //std::cout<<"number of ineq constraints: "<< number_of_poly_constraints<<std::endl;
        //std::cout<<"GQ"<<std::endl;
        //std::cout<<GQ<<std::endl;
        //std::cout<<"g0.transpose"<<std::endl;
        //std::cout<<g0<<std::endl;
        //std::cout<<"CI"<<std::endl;
        //std::cout<<CI<<std::endl;
        //std::cout<<"ci0.transpose"<<std::endl;
        //std::cout<<ci0.transpose()<<std::endl;
        //std::cout<<"ci0.transpose"<<std::endl;

        VectorXd jerkVectors = solution.segment(0, 2*horizon_size_);
        std::cout<<"jerk cost Cj: "<<0.5*jerkVectors.transpose()*Gjerk*jerkVectors <<std::endl;
        std::cout<<"acc cost Ca: "<<0.5*jerkVectors.transpose()*Ga*jerkVectors <<std::endl ;
        std::cout<<"vel cost Cv: "<<0.5*jerkVectors.transpose()*Gv*jerkVectors + gv.transpose()*jerkVectors<<std::endl;
        std::cout<<"slack cost: "<<0.5*(-MatrixXd::Ones(number_of_poly_constraints, 1) - slacks).transpose()*
                                       weight_Qs*
                                        (-MatrixXd::Ones(number_of_poly_constraints, 1) - slacks)<<std::endl;
    }
    //compute violation is a vector of size number_of_constraints, if I evaulate the column for each time sample I can get an idea of which constraint is getting close to zero

    all_violations_ =CI.block(0,0,number_of_poly_constraints,2*horizon_size_) * solution.segment(0, 2*horizon_size_) + ci0.segment(0, number_of_poly_constraints);
    //prt((CI*solution + ci0).transpose())

}




void MPCPlanner::saveTraj(const std::string finename, const VectorXd & var, bool verbose)
{
    double time = 0.0;


    std::ofstream file;
    file.open(finename.c_str());

    for (int i=0; i<var.size();i++)
    {
        file<<time <<" ";
        file<< var(i) <<" ";
        file <<  std::endl;
        time+=Ts;
    }
    if (verbose)
        printf("done saving\n");
    file.close();

}

void MPCPlanner::saveTraj(const std::string finename, const VectorXd & var_x, const VectorXd & var_y, bool verbose)
{
    double time = 0.0;

    std::ofstream file;
    file.open(finename.c_str());

    for (int i=0; i<var_x.size();i++)
    {
        file<<time <<" ";
        file<< var_x(i) <<" ";
        file<< var_y(i) <<" ";
        file <<  std::endl;
        time+=Ts;
    }
    if (verbose)
        printf("done saving x and y vars\n");
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

void  MPCPlanner::buildPolygonMatrix(const iit::dog::LegDataMap<footState> feetStates, const int start_phase_index,
                                     const int phase_duration, const int horizon_size,
                                     Eigen::MatrixXd & A, Eigen::VectorXd & b, int & number_of_constraints )
{
    std::vector<Vector3d> footCCwiseSorted;
    std::vector<planning::LineCoeff2d> lineCoeff;
    int edgeCounter;
    if (phase_duration !=0)
    {
        //comopute the number of stances
        edgeCounter = 0;
        footCCwiseSorted.resize(0);
        for (int leg = 0; leg<4; leg++){
            if (!feetStates[dog::LegID(leg)].swing(start_phase_index)){
                Vector3d footPosXY = Vector3d( feetStates[dog::LegID(leg)].x(start_phase_index) , feetStates[dog::LegID(leg)].y(start_phase_index) , 0.0);
                footCCwiseSorted.push_back(footPosXY);
                edgeCounter++;
                //prt(footPosXY.transpose())
            }
        }
        lineCoeff.resize(edgeCounter);
        //sort the positions
        planning::CounterClockwiseSort(footCCwiseSorted);
        //cycle along the ordered feet to compute the line coeff p*xcp + q*ycp  +r  > + stability_margin
        for(int edgeCounter = 0; edgeCounter<lineCoeff.size(); edgeCounter++){
            //compute the coeffs of the line between two feet
            lineCoeff[edgeCounter] = planning::LineCoeff(footCCwiseSorted[edgeCounter],  footCCwiseSorted[(edgeCounter + 1) % lineCoeff.size()], true); //I set true to normalize and use stab margin
            //prt(footCCwiseSorted[edgeCounter].transpose())
        }


        //apply the coeff to the A matrix that is (horizon_size*4/3 )X( 2*horizon_size), fill in the same value for all the phase duration
        for(int i=0; i < phase_duration;i++){
            for(int edgeCounter = 0; edgeCounter<lineCoeff.size(); edgeCounter++){
                A(number_of_constraints + edgeCounter, start_phase_index + i) = lineCoeff[edgeCounter].p;
                A(number_of_constraints + edgeCounter, start_phase_index + i + horizon_size) = lineCoeff[edgeCounter].q;
                b(number_of_constraints + edgeCounter) = lineCoeff[edgeCounter].r;
            }
            number_of_constraints+=lineCoeff.size();
        }
    }

}

void   MPCPlanner::getSlacks(const iit::dog::LegDataMap<footState> feetStates,Eigen::VectorXd & min_slacks, Eigen::VectorXd & avg_slacks)
{
    avg_slacks.resize(horizon_size_);
    min_slacks.resize(horizon_size_);
    int number_of_constraints = 0;
     for (int i =0; i<horizon_size_;i++)
     {
         //count numner of edges
         int number_of_edges = 0;
         for (int leg =0; leg<4; leg++)
         {
             if (!feetStates[leg].swing(i))
                  number_of_edges++;
         }

         number_of_constraints +=number_of_edges;
         avg_slacks(i) = slacks.segment(number_of_constraints, number_of_edges).mean();
         min_slacks(i) = slacks.segment(number_of_constraints, number_of_edges).maxCoeff();
     }

}

void   MPCPlanner::computeCentroid(const iit::dog::LegDataMap<footState> feetStates,Eigen::VectorXd & centroidX, Eigen::VectorXd & centroidY)
{
    centroidX.resize(horizon_size_);
    centroidY.resize(horizon_size_);

     for (int i =0; i<horizon_size_;i++)
     {
         //count numner of edges
         int number_of_stances = 0;
         double sumfeetX = 0.0;
         double sumfeetY = 0.0;

         for (int leg =0; leg<4; leg++)
         {
             if (!feetStates[leg].swing(i))
             {
                 sumfeetX+=feetStates[leg].x(i);
                 sumfeetY+=feetStates[leg].y(i);
                 number_of_stances++;
             }
         }
         centroidX(i) = sumfeetX/number_of_stances;
         centroidY(i) = sumfeetY/number_of_stances;
     }
}

Eigen::VectorXd   MPCPlanner::getConstraintViolation(const iit::dog::LegDataMap<footState> feetStates)
{
    Eigen::VectorXd constraint_violation_;
    constraint_violation_.resize(horizon_size_);
    Eigen::VectorXd col;
    int number_of_constraints = 0;
     for (int i =0; i<horizon_size_;i++)
     {
         //count numner of edges
         int number_of_edges = 0;
         for (int leg =0; leg<4; leg++)
         {
             if (!feetStates[leg].swing(i))
                  number_of_edges++;
         }
         col = all_violations_.segment(number_of_constraints,number_of_edges);
         number_of_constraints +=number_of_edges;
         constraint_violation_(i) = col.minCoeff();
     }
    return constraint_violation_;

}

void MPCPlanner::computeCOMupdate(Eigen::Vector3d & actualCom, const double jerk_sample)
{
    actualCom = A*actualCom + B*jerk_sample;
}


VectorXd MPCPlanner::makeGaussian(const int length, const int mean, const int stddev)
{
    VectorXd x, weight, arg;
    x.resize(length);arg.resize(length);weight.resize(length);
    x = VectorXd::LinSpaced(length, 1, length);

    arg = -0.5 * ((x.array() - mean)/stddev).array().pow(2);

    weight = arg.array().exp() / (std::sqrt(2*M_PI) * stddev);
    return weight;
}
void MPCPlanner::computeSteps(const LegDataMap<double> & initial_feet_x,
                  const LegDataMap<double> & initial_feet_y,
                  double distance,
                  const int number_of_steps,
                  const int horizon_size,
                  LegDataMap<MPCPlanner::footState> & feetStates, LegDataMap<MPCPlanner::footState> & footHolds,
                  MatrixXd & A,  VectorXd & b, MPCPlanner & myPlanner)
{
    computeSteps(Vector2d(distance/number_of_steps, 0.0), initial_feet_x, initial_feet_y,
            number_of_steps, horizon_size,
            feetStates,  footHolds,
            A,  b,  myPlanner, iit::dog::LF);

}


void MPCPlanner::computeSteps(const Vector2d & userSpeed,  const LegDataMap<double> & initial_feet_x,
                  const LegDataMap<double> & initial_feet_y,
                  const int number_of_steps,
                  const int horizon_size,
                  LegDataMap<MPCPlanner::footState> & feetStates, LegDataMap<MPCPlanner::footState> & footHolds,
                  MatrixXd & A,  VectorXd & b, MPCPlanner & myPlanner, dog::LegID swing_leg_index, Vector2d  initialCoM)
{

    //use internal schedule to predict the steps (assume default schedule sequence TODO)
    FootScheduler schedule; schedule.setSequence(LF, RH,RF,LH);
    schedule.setCurrentSwing(swing_leg_index);
    int start_phase_index,  phase_duration, number_of_constraints;
    int step_knots = floor(horizon_size/number_of_steps);
    LegDataMap<double> feetValuesX, feetValuesY;

    feetValuesX = initial_feet_x;
    feetValuesY = initial_feet_y;

    //init stuff
    number_of_constraints = 0;
    start_phase_index = 0;
    phase_duration = floor(step_knots/2); //10 samples both swing and phase NB this can be not a integer so you need to floor
    A.resize(4*step_knots*number_of_steps, horizon_size*2); //assumes all stance phases then we resize
    b.resize(4*step_knots*number_of_steps);
    A.setZero();
    b.setZero();
    if(debug_mode){
       std::cout<<"phase duration "<<phase_duration<<std::endl;
    }

    for (int leg=0;leg<4;leg++){
        feetStates[leg].resize(horizon_size);
        //set always stances
        feetStates[leg].swing.setConstant(horizon_size,false);
        //footHolds[leg].resize(2*number_of_steps); old
        footHolds[leg].resize(number_of_steps);

    }
    //prt(initialCoM.transpose())

    LegDataMap<bool> comCorrectionFlag = false;
    LegDataMap<Vector2d> comCorrectionValue(Vector2d::Zero());

    if (!initialCoM.isZero(0))
    {
        comCorrectionFlag = true;
        comCorrectionValue[LF] = initialCoM + hip_offsets[LF] + userSpeed  - Vector2d(feetValuesX[LF] ,feetValuesY[LF]);
        comCorrectionValue[RF] = initialCoM + hip_offsets[RF] + userSpeed  - Vector2d(feetValuesX[RF] ,feetValuesY[RF]);
        comCorrectionValue[LH] = initialCoM + hip_offsets[LH] + userSpeed  - Vector2d(feetValuesX[LH] ,feetValuesY[LH]);
        comCorrectionValue[RH] = initialCoM + hip_offsets[RH] + userSpeed  - Vector2d(feetValuesX[RH] ,feetValuesY[RH]);       
    }


    dummy1[LF] = initialCoM + hip_offsets[LF];
    dummy1[RF] = initialCoM + hip_offsets[RF];
    dummy1[LH] = initialCoM + hip_offsets[LH];
    dummy1[RH] = initialCoM + hip_offsets[RH];


    //prt(phase_duration)
    for (int i=0; i<number_of_steps;i++)
    {
        //4 stance///////////////////////////
        feetStates[LF].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[LF]);
        feetStates[RF].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[RF]);
        feetStates[LH].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[LH]);
        feetStates[RH].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[RH]);
        feetStates[LF].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[LF]);
        feetStates[RF].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[RF]);
        feetStates[LH].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[LH]);
        feetStates[RH].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[RH]);
        //build inequalities with the set of stance feet and positions
        myPlanner.buildPolygonMatrix(feetStates, start_phase_index,phase_duration, horizon_size, A,  b,  number_of_constraints );
        start_phase_index += phase_duration;

        //3 stance/////////////////////////////////////
        //step
        Vector2d deltaStep;
        if (comCorrectionFlag[schedule.getCurrentSwing()])
        {
            if (debug_mode)
            {
                std::cout<< "correxcting com for leg : " <<legmap[schedule.getCurrentSwing()]
                         <<"  X: "<<comCorrectionValue[schedule.getCurrentSwing()](rbd::X)
                         <<"  Y: "<<comCorrectionValue[schedule.getCurrentSwing()](rbd::Y)<<std::endl;
            }
            deltaStep = comCorrectionValue[schedule.getCurrentSwing()];
            comCorrectionFlag[schedule.getCurrentSwing()] = false;
            dummy2[schedule.getCurrentSwing()] = comCorrectionValue[schedule.getCurrentSwing()];
        } else {
            //default stepping
            deltaStep = userSpeed;
        }

        feetValuesX[schedule.getCurrentSwing()]+=  deltaStep(0); //TODO consider swing duration
        feetValuesY[schedule.getCurrentSwing()]+=  deltaStep(1);

        //save for debug
        footHolds[LF].x(i) = feetValuesX[LF]; footHolds[LF].y(i) = feetValuesY[LF];
        footHolds[RF].x(i) = feetValuesX[RF]; footHolds[RF].y(i) = feetValuesY[RF];
        footHolds[LH].x(i) = feetValuesX[LH]; footHolds[LH].y(i) = feetValuesY[LH];
        footHolds[RH].x(i) = feetValuesX[RH]; footHolds[RH].y(i) = feetValuesY[RH];

        //set swing for that leg
        feetStates[schedule.getCurrentSwing()].swing.segment(start_phase_index, phase_duration).setConstant(true);
        feetStates[LF].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[LF]);
        feetStates[RF].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[RF]);
        feetStates[LH].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[LH]);
        feetStates[RH].x.segment(start_phase_index, phase_duration).setConstant(feetValuesX[RH]);
        feetStates[LF].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[LF]);
        feetStates[RF].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[RF]);
        feetStates[LH].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[LH]);
        feetStates[RH].y.segment(start_phase_index, phase_duration).setConstant(feetValuesY[RH]);
    //    //build inequalities with the set of stance feet and positions
        myPlanner.buildPolygonMatrix(feetStates, start_phase_index,phase_duration,horizon_size, A,  b,  number_of_constraints );
        start_phase_index += phase_duration;
        schedule.next(); //update the step in the schedule
    }


    //compute missing knots last phase is double stance
    int missing_knots = horizon_size - start_phase_index;

    //end with 4 stance
    feetStates[LF].x.segment(start_phase_index, missing_knots).setConstant(feetValuesX[LF]);
    feetStates[RF].x.segment(start_phase_index, missing_knots).setConstant(feetValuesX[RF]);
    feetStates[LH].x.segment(start_phase_index, missing_knots).setConstant(feetValuesX[LH]);
    feetStates[RH].x.segment(start_phase_index, missing_knots).setConstant(feetValuesX[RH]);
    feetStates[LF].y.segment(start_phase_index, missing_knots).setConstant(feetValuesY[LF]);
    feetStates[RF].y.segment(start_phase_index, missing_knots).setConstant(feetValuesY[RF]);
    feetStates[LH].y.segment(start_phase_index, missing_knots).setConstant(feetValuesY[LH]);
    feetStates[RH].y.segment(start_phase_index, missing_knots).setConstant(feetValuesY[RH]);

    myPlanner.buildPolygonMatrix(feetStates, start_phase_index,missing_knots,horizon_size, A,  b,  number_of_constraints);

    //cause you have 3 stances phases and you need to shrink
    A.conservativeResize(number_of_constraints,horizon_size*2);

    b.conservativeResize(number_of_constraints);

    //prt(missing_knots)
}

void MPCPlanner::printSwing(LegID swing)
{
   switch(swing)
   {
    case LF:
        std::cout<<"swing LF:"<<std::endl;
        break;
    case RF:
        std::cout<<"swing RF:"<<std::endl;
        break;
    case LH:
        std::cout<<"swing LH:"<<std::endl;
        break;
    case RH:
        std::cout<<"swing RH:"<<std::endl;
        break;
    default:
        break;
   }
}

void MPCPlanner::setHipOffsets(iit::dog::LegDataMap<Eigen::Vector2d> hip_offsets)
{
    this->hip_offsets = hip_offsets;
}
