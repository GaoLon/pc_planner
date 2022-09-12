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
#include <config.hpp>
using namespace Eigen;
using namespace std;
using namespace cubic_curvature;

class LocalOptimizer;

struct param_restrain
{
	Matrix<double, 4, 1> d_rob ;
	// double d_nom = 0.16;
	// double kappa_max = 8;

	double d_nom ;
	double kappa_max ;

	double roll_max ;
	double pitch_min ;
	double pitch_max ;
	double rho_max ;
	double eta;
	double normal_max;
	double w_normal;
	//拟平面的
	float r_plane ;

	//算粗糙度的
	float r_res ;
	double w_rho ;
	double w_roll ;
	double w_pitch ;
	double traversable_threshold;
};

class PCTrajNode//such as Ni Ni+1,when init,fp and ft,if (positon of tmr～ - mean_p).norm() < 0.5,tau=0
{
friend LocalOptimizer;
// friend RRT_Node;
friend void pubProjectedPose(ros::Publisher *pubPtr, PCTrajNode node);
private:
	// Matrix4d T;//such as TMRi,this node's position and orientation in map
	double tau;	// traversibility
	double kappa; // path curvature value
	VectorXd t;		//kappa, a, b, c, sf
	//this node is the start of t
public:
		Matrix4d T;//such as TMRi,this node's position and orientation in map
	// static ros::Publisher *pubPtr;
	PCTrajNode():T(MatrixXd::Identity(4,4)), tau(0.0),\
		kappa(0.0), t(VectorXd::Zero(5)) {}
	//fp and ft,if (positon of tmr～ - mean_p).norm() < 0.5,tau=0
	PCTrajNode(const Matrix4d& Tr, double _kap = 0.0);
	PCTrajNode(const Matrix4d& Tr, bool simple);
	//fp: TMR->TMR～  proj to caculate proper pose ,change T to be TMR～ ,check validity and return bool
	bool fp(const Matrix4d& Tr, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd, int K=16);

	//ft: TMR->tau    proj to caculate traversibility
	double ft(pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd);

	void ter_assess(void);
	void set_sf(double sf){
		t(4)=sf;
	}
	// bool isTraversable(void) { return true; }
	bool isTraversable(void); 

	VectorXd connect(const PCTrajNode& toNode);

	//give a point in 3d ,return the dist between point and this node
	double get_dist(Vector3d p){ return (p - T.topRightCorner(3, 1)).norm(); }

	//return the dist between other node and this node
	double get_dist(const PCTrajNode& p) const { return (p.T.topRightCorner(3, 1) - T.topRightCorner(3, 1)).norm(); }

	//sf is the total length of t
	double get_cost() { return t[4]; }

	double get_tau() { return tau; }
	const Matrix4d& get_T(){return T;}

	//from matrix with orientation  to  pointcloud with position
	inline pcl::PointXYZI Chan2PC();

	//return the result of "this node T rotate around z,and translate +x +y"
	// x,y,theta
	inline Matrix4d Trans(const Vector3d& trans) const;	

	//make "this node T rotate around z,and translate +x +y" happened
	inline void change_T(const Vector3d& trans);

	// p in map frame  -> p in this node frame 
	inline Vector2d projection(const Vector3d& p) const;

	//return T's position
	Vector3d get_pos() const { return T.topRightCorner(3, 1); }

//静态成员函数/变量属于这一个类的，可以在类外不实例化直接用类名调用
	static param_restrain PR;
	static Config config;
	static pcl::PointCloud<pcl::PointXYZI>::Ptr PCMap;
	static pcl::PointCloud<pcl::PointXYZI>::Ptr debug;
	static pcl::PointCloud<pcl::PointXYZI>::Ptr PCMap_fiter_rho_vis;

	static VectorXd get_connect(const PCTrajNode& fromNode, const PCTrajNode& toNode);
	static vector<PCTrajNode> SRT_Generate(const PCTrajNode& fromNode, const PCTrajNode& toNode, double d1 = 2.0, double d2 =4.0);
	static pair<bool, double> get_rho(pcl::PointXYZI& p, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd);
	static Vector3d calculate_rho_and_normal(pcl::PointXYZI& p, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd);
	static Vector3d calculate_normal(pcl::PointXYZI& p, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd);
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

