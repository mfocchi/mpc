#ifndef LIB_COMMONS_H_
#define LIB_COMMONS_H_

#include <Eigen/Dense>

#include "robot.h"
#include <iit/commons/dog/joint_id_declarations.h>
#include <iit/commons/dog/leg_data_map.h>
#include <math_utils/utils.h>
#include <iit/rbd/rbd.h>
#define prt(x) std::cout << #x " = \n" << x << "\n" << std::endl;

using namespace iit::dog;

namespace iit {
namespace dog {

static const int jointsLegCount = 3; //number of joints per leg
static const int contactConstrCount = 3; //number of constraint for each contact point
static const int legsjointsCount = 12; //total number of joints in the legs
//index to define the joint state
static const int baseJoints = 0;
static const int activeJoints = 6;

#ifndef USECENTAUR
	static const int fbjointsCount = 18; //TODO extension tp centaur
	typedef Eigen::Matrix<double, fbjointsCount, 1> Column18d;//TODO extension tp centaur
	typedef Column18d fbJointState;
#else
	static const int fbjointsCount = 24; //TODO extension tp centaur
	typedef Eigen::Matrix<double, fbjointsCount, 1> Column24d;//TODO extension tp centaur
	typedef Column24d fbJointState;
#endif

typedef  Eigen::Matrix<double, legsjointsCount,1>legsJointState;
typedef  Eigen::Matrix<double, legsjointsCount,1>legsContactForces;


typedef Eigen::Vector3d LegJointState;


enum JointType{PRISMATIC =0, REVOLUTE};


enum FbJointIdentifiers {
	 BASE_AX = 0
	, BASE_AY
	, BASE_AZ
	, BASE_LX
	, BASE_LY
	, BASE_LZ
	, FB_LF_HAA
    , FB_LF_HFE
    , FB_LF_KFE
    , FB_RF_HAA
    , FB_RF_HFE
    , FB_RF_KFE
    , FB_LH_HAA
    , FB_LH_HFE
    , FB_LH_KFE
    , FB_RH_HAA
    , FB_RH_HFE
    , FB_RH_KFE
};

inline iit::dog::JointIdentifiers toJointID(dog::LegID leg, dog::LegJoints j) {
	return dog::orderedJointIDs[ leg*3 + j ];
}


inline iit::dog::LegID toLegID(dog::JointIdentifiers joint) {
	return (dog::LegID) floor(joint/3.0);
}

inline bool belongsTo(const dog::LegID leg, const dog::JointIdentifiers jointID)
{

	if ((leg==dog::LF) &&
			((jointID == dog::LF_HAA)||(jointID == dog::LF_HFE)||(jointID == dog::LF_KFE)))
				return true;
	if ((leg==dog::RF) &&
			((jointID == dog::RF_HAA)||(jointID == dog::RF_HFE)||(jointID == dog::RF_KFE)))
				return true;
	if ((leg==dog::LH) &&
			((jointID == dog::LH_HAA)||(jointID == dog::LH_HFE)||(jointID == dog::LH_KFE)))
				return true;
	if ((leg==dog::RH) &&
			((jointID == dog::RH_HAA)||(jointID == dog::RH_HFE)||(jointID == dog::RH_KFE)))
				return true;

	return false;
}

inline bool belongsTo(const dog::LegID leg, const dog::FbJointIdentifiers jointID)
{

	if ((leg==dog::LF) &&
			((jointID == dog::FB_LF_HAA)||(jointID == dog::FB_LF_HFE)||(jointID == dog::FB_LF_KFE)))
				return true;
	if ((leg==dog::RF) &&
			((jointID == dog::FB_RF_HAA)||(jointID == dog::FB_RF_HFE)||(jointID == dog::FB_RF_KFE)))
				return true;
	if ((leg==dog::LH) &&
			((jointID == dog::FB_LH_HAA)||(jointID == dog::FB_LH_HFE)||(jointID == dog::FB_LH_KFE)))
				return true;
	if ((leg==dog::RH) &&
			((jointID == dog::FB_RH_HAA)||(jointID == dog::FB_RH_HFE)||(jointID == dog::FB_RH_KFE)))
				return true;

	return false;
}
//it assumes the legs are always the first joints
inline void getLegJointsState(dog::LegID leg, const dog::JointState& jstate, LegJointState& vecout)
{
	// WARNING: the following relies on consistency between LegID and the ordering of
	//  elements in the JointState type
	vecout = (jstate.block<3,1>(leg*dog::jointsLegCount,0));
}

inline LegJointState getLegJointsState(dog::LegID leg, const dog::JointState& jstate)
{

	// WARNING: the following relies on consistency between LegID and the ordering of
	//  elements in the JointState type
	return jstate.block<3,1>(leg*dog::jointsLegCount,0);
}


inline void assignLegJointsState(dog::LegID leg, const LegJointState& vecin,  iit::dog::JointState& jstate)
{
	// WARNING: the following relies on consistency between LegID and the ordering of
	//  elements in the JointState type
	jstate.block<3,1>(leg*dog::jointsLegCount,0) = vecin;
}

inline dog::JointState assignLegJointsState(dog::LegID leg, const LegJointState& vecin)
{
	dog::JointState jstate;
	// WARNING: the following relies on consistency between LegID and the ordering of
	//  elements in the JointState type
	jstate.block<3,1>(leg*dog::jointsLegCount,0) = vecin;
	return jstate;
}

inline dog::JointState toFullState(const legsJointState& vecin)
{
	dog::JointState jstate;
	jstate.segment(0,legsjointsCount) = vecin;
	return jstate;
}

}//namespace dog

//namespace iit
struct Point2d {
  Eigen::Vector2d x;
  Eigen::Vector2d xd;
  Eigen::Vector2d xdd;
  explicit Point2d(Eigen::Vector2d _x = Eigen::Vector2d::Zero(),
                   Eigen::Vector2d _xd = Eigen::Vector2d::Zero(),
                   Eigen::Vector2d _xdd = Eigen::Vector2d::Zero())
      : x(_x), xd(_xd), xdd(_xdd) {}
  inline Point2d &   operator=(const Point2d& rhs)
  {
  	x = rhs.x;
  	xd = rhs.xd;
  	xdd = rhs.xdd;
  	return *this;
  }

};



struct Point3d {
  Eigen::Vector3d x;
  Eigen::Vector3d xd;
  Eigen::Vector3d xdd;
  explicit Point3d(Eigen::Vector3d _x = Eigen::Vector3d::Zero(),
                   Eigen::Vector3d _xd = Eigen::Vector3d::Zero(),
                   Eigen::Vector3d _xdd = Eigen::Vector3d::Zero())
      : x(_x), xd(_xd), xdd(_xdd) {}
  inline Point3d &   operator=(const Point3d& rhs)
  {
  	x = rhs.x;
  	xd = rhs.xd;
  	xdd = rhs.xdd;
  	return *this;
  }

};



struct Ori {
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d wd;
  explicit Ori(Eigen::Quaterniond _q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
               Eigen::Vector3d _w    = Eigen::Vector3d::Zero(),
               Eigen::Vector3d _wd    = Eigen::Vector3d::Zero())
  : q(_q), w(_w), wd(_wd) {}

};


struct Pose {
  Point3d pos;
  Ori ori;
};

/** p*x + q*y + r = 0 */
struct LineCoeff2d {
  double p;
  double q;
  double r;
};

/// \brief  Checks if p2 is on the right side of line from p0 to p1
/// \return >0 for P2 right of the line from P0 to P1
///         =0 for P2 on the line
///         <0 for P2 left of the line
inline double Point2isRightOfLine(const Eigen::Vector3d p0, const Eigen::Vector3d p1, const Eigen::Vector3d p2)
{
  return (p2(rbd::X) - p0(rbd::X)) * (p1(rbd::Y) - p0(rbd::Y)) - (p1(rbd::X) - p0(rbd::X)) * (p2(rbd::Y) - p0(rbd::Y));
}

/**
  \brief Performs a  clockwise radial sort of the input points
         starting with the bottom left point
  @param[out] p Unsorted points or footholds.
  Uses the slow n^2 sort algorithm, shouldn't matter for sorting 4 points :)
  Fails when 3 points are on same line and one could be removed
   */

static void ClockwiseSort(std::vector<Eigen::Vector3d>& p)
{

	// sort clockwise
	for (int i = 1; i < p.size() - 1; i++) {
		for (int j = i + 1; j < p.size(); j++) {
	        //the point p2 should always be on the right to be cwise thus if it
	        //is on the left <0 i swap
			if (Point2isRightOfLine(p[0], p[i], p[j])  < 0.0) {
				Eigen::Vector3d tmp = p[i];
				p[i] = p[j];
				p[j] = tmp;
			}
		}
	}
}
/**
  \brief Performs a  counter clockwise radial sort of the input points
         starting with the bottom left point
  @param[out] p Unsorted points or footholds.
  Uses the slow n^2 sort algorithm, shouldn't matter for sorting 4 points :)
  Fails when 3 points are on same line and one could be removed
   */

static void CounterClockwiseSort(std::vector<Eigen::Vector3d>& p)
{

	// sort counter clockwise
	for (int i = 1; i < p.size() - 1; i++) {
		for (int j = i + 1; j < p.size(); j++) {
	        //the point p2 should always be on the left of the line to be ccwise thus if it
	        //is on the right  >0  swap
			if (Point2isRightOfLine(p[0], p[i], p[j])  > 0.0) {
				Eigen::Vector3d tmp = p[i];
				p[i] = p[j];
				p[j] = tmp;
			}
		}
	}
}

inline LineCoeff2d LineCoeff(const Eigen::Vector3d& pt0, const Eigen::Vector3d& pt1, bool normalize = true) {
	//(p,q).dot(x-x0, y-y0)=0 implicit line equation
	//px + qy -rx0 -qy0 = 0 => px + qy + r = 0
	//to find (p,q) I know this is the normal to the line, so if I have another point x1 a solution
	// with (p,q) perpendicular to the segment S=(x1- x0, y1-y0) is p= -(y1-y0) q=x1-x0 for which (p,q).dot(S) = 0
	LineCoeff2d ret;
	ret.p = pt0(rbd::Y) - pt1(rbd::Y);
	ret.q = pt1(rbd::X) - pt0(rbd::X);
	ret.r = -ret.p * pt0(rbd::X) - ret.q * pt0(rbd::Y);

	// normalize the equation in order to intuitively use stability margins
	if (normalize) {
		double norm = hypot(ret.p, ret.q);
		ret.p /= norm;
		ret.q /= norm;
		ret.r /= norm;
	}

	return ret;
}

inline bool pointInTriangle(const  Eigen::Vector3d& A, const  Eigen::Vector3d& B,const  Eigen::Vector3d& C, const  Eigen::Vector3d& P){

   // Barycentric Technique from
   //http://www.blackpawn.com/texts/pointinpoly/default.html
	// Compute vectors
	Eigen::Vector3d v0,v1,v2;
	//v0 = C - A
	v0= C - A;
	//v1 = B - A
	v1=  B - A;
	//v2 = P - A
	v2 = P - A;

	//TODO check coplanarity

	double dot00,	dot01,	dot02,	dot11,	dot12;

	// Compute dot products
	dot00 = v0.dot(v0);
	dot01 = v0.dot(v1);
	dot02 = v0.dot(v2);
	dot11 = v1.dot(v1);
	dot12 = v1.dot(v2);

	 // Compute barycentric coordinates
	 double denom = dot00 * dot11 - dot01 * dot01;

	 if(denom != 0) {

	   double invDenom = 1 / denom;
	   double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	   double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	   // Check if point is in triangle
	   return (u > 0) && (v > 0) && (u + v < 1);

	 } else {
	    printf("pt_in_tr: error denom == 0");
	   return false;
	 }
}


//the distance is from the first point
inline bool  line_intersect_from_points(const Eigen::Vector3d& point1_line1, const Eigen::Vector3d& point2_line1,
								   const Eigen::Vector3d& point1_line2, const Eigen::Vector3d& point2_line2,
								   Eigen::Vector3d & intersection, double & s)
{
	Eigen::Vector3d a,b,c, ret;
	//for 3d  see  http://mathworld.wolfram.com/Line-LineIntersection.html
	//check coplanarity
	a=point2_line1-point1_line1;
	b=point2_line2-point1_line2;
	c=point1_line2-point1_line1;
	bool nonCoplanar = ((fabs(c.dot(a.cross(b))) >= 1E-3));
	if (!nonCoplanar)
	{
		s=(c.cross(b).dot(a.cross(b)) )  / (a.cross(b).squaredNorm());
		intersection = point1_line1 + a*s;
		return true;
	} else {
	 std::cout << "vectors are not coplanar cannot finde intersection" << std::endl << std::endl;
		return false;
	}
}

inline Eigen::Vector3d line_intersect_from_points(const Eigen::Vector3d& point1_line1, const Eigen::Vector3d& point2_line1,const Eigen::Vector3d& point1_line2,const Eigen::Vector3d& point2_line2)
{
	Eigen::Vector3d intersection;
	double s;
	line_intersect_from_points(point1_line1, point2_line1, point1_line2, point2_line2, intersection, s);
	return intersection;
}

// overloading operator<< for more elegant priting of above values
inline std::ostream& operator<<(std::ostream& out, const LineCoeff2d& lc)
{
  out  << "p=" << lc.p << ", q=" << lc.q << ", r=" << lc.r;
  return out;
}

//inputs are the coefficient of the lines and one point on the line
inline std::ostream& operator<<(std::ostream& out, const Point2d& pos)
{
  out << "x=" << pos.x.transpose() << "  "
      << "xd=" << pos.xd.transpose() << "  "
      << "xdd=" << pos.xdd.transpose();
  return out;
}


inline std::ostream& operator<<(std::ostream& out, const Point3d& pos)
{
  out << "x=" << pos.x.transpose() << "  "
      << "xd=" << pos.xd.transpose() << "  "
      << "xdd=" << pos.xdd.transpose();
  return out;
}


inline std::ostream& operator<<(std::ostream& out, const Ori& ori)
{
  Eigen::Vector3d rpy_rad, rpy_deg;
  rpy_rad = commons::quatToRPY(ori.q);
  rpy_deg = rpy_rad * (180.0 / 3.14);
  out << "rpy=" << rpy_deg.transpose() << "  "
      << "w=" << ori.w.transpose() << "  "
      << "wd=" << ori.wd.transpose();
  return out;
}


inline std::ostream& operator<<(std::ostream& out, const Pose& pose)
{
  out << "\tPos: " << pose.pos << "\n"
      << "\tOri: " << pose.ori;
  return out;
}

//namespace iit


}

#endif
