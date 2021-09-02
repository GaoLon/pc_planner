#pragma once

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <vector>
#include <algorithm>
#include <cmath>
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include "cubic_curvature.h"
// #include "RRT.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

using namespace Eigen;
using namespace std;
using namespace cubic_curvature;

class LocalOptimizer;

struct param_restrain
{
	Matrix<double, 4, 1> d_rob = { 1,1,1,1 };
	double d_nom = 0.16;
	//double d_nom = 0.6;
	// double kappa_max = 1.5;
	double kappa_max = 8;
	double roll_max = M_PI_4;
	double pitch_min = -M_PI_4;
	double pitch_max = M_PI_4;
	double rho_max =1;
	double eta = 0.0;
	float r_plane = 0.8;
	float r_res = 0.5;
	double w_rho = 1.0;
	double w_roll = 0.0;
	double w_pitch = 0.0;
	double traversable_threshold=0.01;
};

class PCTrajNode
{
friend LocalOptimizer;
// friend RRT_Node;
friend void pubProjectedPose(ros::Publisher *pubPtr, PCTrajNode node);
private:
	Matrix4d T;
	double tau;	// traversibility
	double kappa; // path curvature value
	VectorXd t;		//kappa, a, b, c, sf
public:
	
	// static ros::Publisher *pubPtr;
	PCTrajNode():T(MatrixXd::Identity(4,4)), tau(0.0),\
		kappa(0.0), t(VectorXd::Zero(5)) {}
	PCTrajNode(const Matrix4d& Tr, double _kap = 0.0);
	PCTrajNode(const Matrix4d& Tr, bool simple);
	bool fp(const Matrix4d& Tr, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd, int K=16);
	double ft(pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd);
	void ter_assess(void);
	bool isTraversable(void) { return tau > PR.traversable_threshold; }
	// bool isTraversable(void) { return true; }
	VectorXd connect(const PCTrajNode& toNode);
	double get_dist(Vector3d p){ return (p - T.topRightCorner(3, 1)).norm(); }
	double get_dist(const PCTrajNode& p) const { return (p.T.topRightCorner(3, 1) - T.topRightCorner(3, 1)).norm(); }
	double get_cost() { return t[4]; }
	double get_tau() { return tau; }
	const Matrix4d& get_T(){return T;}
	inline pcl::PointXYZI Chan2PC();
	inline Matrix4d Trans(const Vector3d& trans) const;	// x,y,theta
	inline void change_T(const Vector3d& trans);
	inline Vector2d projection(const Vector3d& p) const;
	Vector3d get_pos() const { return T.topRightCorner(3, 1); }
	static param_restrain PR;
	static pcl::PointCloud<pcl::PointXYZI>::Ptr PCMap;
	static VectorXd get_connect(const PCTrajNode& fromNode, const PCTrajNode& toNode);
	static vector<PCTrajNode> SRT_Generate(const PCTrajNode& fromNode, const PCTrajNode& toNode, double d1 = 4.0);
	static pair<bool, double> get_rho(pcl::PointXYZI& p, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd);
	// void pubProjectedPose();
	~PCTrajNode(){}
	
};

inline pcl::PointXYZI PCTrajNode::Chan2PC()
{ 
	pcl::PointXYZI p;
	p.x = T.row(0)[3];
	p.y = T.row(1)[3];
	p.z = T.row(2)[3];
	return p;
}

inline Matrix4d PCTrajNode::Trans(const Vector3d& trans) const
{
	Matrix4d T_seg = Matrix4d::Identity();
	T_seg.row(0)[0] = T_seg.row(1)[1] = cos(trans[2]);
	T_seg.row(0)[1] = -sin(trans[2]);
	T_seg.row(1)[0] = sin(trans[2]);
	T_seg.row(0)[3] = trans[0];
	T_seg.row(1)[3] = trans[1];
	return T * T_seg;
}

inline void PCTrajNode::change_T(const Vector3d& trans)
{
	T = this->Trans(trans);
}

inline Vector2d PCTrajNode::projection(const Vector3d& p) const
{
	Vector4d pl(p[0], p[1], p[2], 1);
	Vector4d pt = T.inverse()*pl;

	return pt.head(2);
}

