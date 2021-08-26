#include <iostream>
// #include <conio.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <geometry_msgs/PoseArray.h>
#include "PCTrajNode.h"
#include "RRT.h"
// #include "RRT_star.h"
// #include "LocalOptimizer.h"
#include "Astarplanner.h"

using namespace std;
using namespace Eigen;

pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
bool mapReceived = false;
ros::Publisher *pubPtr, *pubArrayPtr;
RRT* planner;

void visualizeTreeNode(const vector<RRT_Node> *vec_ptr, ros::Publisher *pub_ptr)
{
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "/map";
	msg.header.stamp = ros::Time::now();

	geometry_msgs::Pose pose;
	tf::Matrix3x3 R;
	Vector3d point;
	Matrix4d T;

	for (auto node : *vec_ptr)
	{
		T=node.node.get_T();
		point=T.block<3,1>(0,3);
		pose.position.x=point(0);
		pose.position.y=point(1);
		pose.position.z=point(2);
		for(int i=0; i<3; i++)
		{
			for (int j=0; j<3; j++)
			{
				R[i][j]=T(i,j);
			}
		}
		tfScalar yaw, pitch,row;
		R.getEulerYPR(yaw, pitch, row);
		pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
		msg.poses.push_back(pose);
	}
	pub_ptr->publish(msg);
}

void planOnce()
{
	kdtree.setInputCloud(PCTrajNode::PCMap);
	srand((unsigned)time(0));
	// init start and goal
	/// rabbit
	Vector3d start(0, 0, 0);
	Vector3d goal(100, 50, 0);
	// Vector3d goal(3.275, 5.977, 3.952);
	/// dragon
	/*Vector3d start(-4.172, 13.63, 0.062);
  Vector3d goal(5.446, 12.07, -0.2122);*/
	Matrix4d st = Matrix4d::Identity();
	Matrix4d gt = Matrix4d::Identity();
	/*gt << 0, 1, 0,0,\
	  -1, 0, 0, 0,\
	  0, 0, 1,0,\
	  0,0,0,1;*/

	st.topRightCorner(3, 1) = Matrix<double, 3, 1>(start);
	gt.topRightCorner(3, 1) = Matrix<double, 3, 1>(goal);
	pcl::StopWatch time;

	PCTrajNode s(st);
	// ROS_INFO("damn");
	cout << "constructor once consume:" << time.getTime() << "ms" << std::endl;
	PCTrajNode g(gt);

	pubProjectedPose(pubPtr, g);
	// system("pause");

	// launch planner
	
	// Astarplanner planner;
	vector<PCTrajNode> path;
	planner = new RRT();

	ROS_INFO("before search done");
	vector<RRT_Node> pa = planner->find_path(s, g);
	// visualizeTreeNode(planner.get_start_tree(), pubArrayPtr);
	visualizeTreeNode(planner->get_goal_tree(), pubArrayPtr);

	//
	// if (planner.AstarSearch(s, g))
	// {
	// 	path = planner.getPath();
	// }
	for (auto p : pa)
		path.push_back(p.node);

	ROS_INFO("planner finished");
}

void visualizePointCloud()
{
	// PCTrajNode::PCMap.
	pcl::PointXYZI start, target;
	start.x = 0;
	start.y = 0;
	start.z = 0;
	start.intensity = 1.0f;
	target.x = 100;
	target.y = 50;
	target.z = 0;
	target.intensity = 1.0f;
	PCTrajNode::PCMap->push_back(start);
	PCTrajNode::PCMap->push_back(target);

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(PCTrajNode::PCMap, "intensity");
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloud(PCTrajNode::PCMap, fildColor, "sample cloud");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.spin();
	ROS_INFO("visualization done");
}

void pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
	// pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	if (!mapReceived)
	{
		ROS_INFO("Callback called!");
		pcl::fromROSMsg(*laserCloudMsg, *PCTrajNode::PCMap);

		int n = PCTrajNode::PCMap->points.size();
		for (int i = 0; i < n; i++)
		{
			PCTrajNode::PCMap->points[i].intensity = -1;
		}
		mapReceived = true;

		pcl::io::savePCDFileASCII<pcl::PointXYZI>("/home/edward/indoor.pcd", *PCTrajNode::PCMap);

		ROS_INFO("Point cloud planner: Map received.");
		visualizePointCloud();
		planOnce();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_planner");
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/overall_map", 2, pclMapCallback);

	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pcl_debug_pose", 2);
	pubPtr = &pose_pub;
	ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/pcl_debug_poses", 2);
	pubArrayPtr = &pose_array_pub;

	// kdtree.setInputCloud(PCTrajNode::PCMap);

	// visulization
	/*pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(PCTrajNode::PCMap, "intensity");
  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  viewer.addPointCloud(PCTrajNode::PCMap,fildColor, "sample cloud");
  viewer.setBackgroundColor(0, 0, 0);
  viewer.spin();
  system("pause");*/

	// return (0);
	ROS_INFO("planner initialized");
	ros::spin();
}

// test gsl
//#include <iostream>
//#include <cmath>
//#include <gsl/gsl_sf.h>
//#include <gsl/gsl_integration.h>
//#include "PCTrajNode.h"
//#include "RRT.h"
//#include "RRT_star.h"
//#include "LocalOptimizer.h"
// pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
//
// double f(double x, void * params) {
//	double* alpha = (double *)params;
//	double a = alpha[0];
//	double b = alpha[1];
//	double c = alpha[2];
//	double kappa = alpha[3];
//	double theta = alpha[4];
//	double f = theta + kappa * x + a * x*x / 2 + b * pow(x, 3) / 3 + c * pow(x, 4) / 4;
//
//	return cos(f);
//}
//
//
// int main()
//
//{
//
//	gsl_integration_workspace * w
//		= gsl_integration_workspace_alloc(1000);
//
//	double result, error;
//	double expected = 0.290879242285901;
//	double alpha[5] = { 67.8973613365547,-292.8943271995715,292.9903274469080,0,0 };
//
//	gsl_function F;
//	F.function = &f;
//	F.params = (void*)alpha;
//
//	gsl_integration_qag(&F, 0, 0.38, 0, 1e-10, 1e-10, 1000, \
//		w, &result, &error);
//
//	printf("result          = % .18f\n", result);
//	printf("exact result    = % .18f\n", expected);
//	printf("estimated error = % .18f\n", error);
//	printf("actual error    = % .18f\n", result - expected);
//	printf("intervals =  %d\n", w->size);
//
//	return 0;
//
//}
