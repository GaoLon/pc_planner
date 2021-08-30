#pragma once
#include "PCTrajNode.h"
#include "cubic_curvature.h"
#include <ctime>
#include <cstdlib>
#include <utility>
#include <algorithm>
#include <set>
#include "kdtree.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

struct kdRRT_Node
{
	PCTrajNode node;
	kdRRT_Node* parent = nullptr;
	bool useful = true;
	double cost = std::numeric_limits<double>::infinity();
	int traj_idx = 11;
	set<int> no_use{ 0,1,2,3,4,5,6,7,8,9,10};
	kdRRT_Node(PCTrajNode a) :node(a) {}
	kdRRT_Node(PCTrajNode a, kdRRT_Node* b) :node(a), parent(b) {}
	~kdRRT_Node() {}
};

typedef kdRRT_Node* kdRRT_NodePtr;

class kdRRT
{
protected:
ros::NodeHandle nh;
	bool inProcess = false;
	int max_iter;
	double r_exp = PCTrajNode::PR.d_nom;
	double step;
	vector<kdRRT_NodePtr> start_node_pool;
	vector<kdRRT_NodePtr> goal_node_pool;
	kdRRT_NodePtr start_node = nullptr;
	kdRRT_NodePtr goal_node = nullptr;
	struct kdtree* start_tree = nullptr;
	struct kdtree* goal_tree = nullptr;
	bool direction = true;		// true: start tree; false: goal tree
	Vector3d traj_lib[11] = { {0.16,0,0}, { 0.159992000066666, 0.001599973333467,0.03}, \
	{ 0.159992000066666, -0.001599973333467, -0.03} , { 0.159928005399838 ,  0.004799280032399 ,0.06},\
	{ 0.159928005399838, -0.004799280032399, -0.06}, { 0.159872017065756 ,  0.006398293469861 ,0.09},\
	{ 0.159872017065756, -0.006398293469861, -0.09}, { 0.159712086389633 ,  0.009594241036711,0.12 },\
	{ 0.159712086389633, -0.009594241036711, -0.12 }, { 0.159608160040525 ,  0.011190855574005 ,0.15},\
	{ 0.159608160040525, -0.011190855574005, -0.15} };		//x, y, theta
	Vector3d traj_lib_back[11] = { {-0.16,0,0}, { -0.159992000066666, 0.001599973333467,-0.03}, \
	{ -0.159992000066666, -0.001599973333467, 0.03} , { -0.159928005399838 ,  0.004799280032399 ,-0.06},\
	{ -0.159928005399838, -0.004799280032399, 0.06}, { -0.159872017065756 ,  0.006398293469861 ,-0.09},\
	{ -0.159872017065756, -0.006398293469861, 0.09}, { -0.159712086389633 ,  0.009594241036711,-0.12 },\
	{ -0.159712086389633, -0.009594241036711, 0.12 }, { -0.159608160040525 ,  0.011190855574005 ,-0.15},\
	{ -0.159608160040525, -0.011190855574005, 0.15} };		//x, y, theta
	double traj_cost[12] = { 0.16,0.1600288 ,0.1600288,0.1601152 ,0.1601152 ,\
		0.1602592, 0.1602592, 0.1604608, 0.1604608, 0.16072, 0.16072, 0 };		//sf
	vector<kdRRT_Node> get_path();
public:
	kdRRT() 
	{
		nh = ros::NodeHandle("~");
		nh.getParam("step", step);
		nh.getParam("max_iter", max_iter);
		PCTrajNode::PR.d_nom = step;
		r_exp = step;
		float k = step / 0.16; //0.16 is the step size of the trajectory library
		for (auto i = 0; i < 11; i++)
		{
			// TODO: change the scale factor?
			// the trajectory library was generated using step size of 0.6m
			// to adapt to the step of 0.15m, the x and y part of the trajectories should be scaled
			traj_lib[i][0] *= k;
			traj_lib[i][1] *= k;
			traj_lib_back[i][0] *= k;
			traj_lib_back[i][1] *= k;
			traj_cost[i] *= k;
		};

		ROS_INFO("max_iter: %d", max_iter);
		gsl_set_error_handler_off();

		pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/overall_map", 2, &kdRRT::pclMapCallback, this);
		// goal_sub = nh.subscribe("/move_base_simple/goal", 10, &kdRRT::setGoalCallback, this);
		tree_node_pub = nh.advertise<geometry_msgs::PoseArray>("/planning/tree_poses", 2);
		tree_edge_pub = nh.advertise<visualization_msgs::Marker>("/planning/tree_edges", 2);
		path_pub = nh.advertise<geometry_msgs::PoseArray>("/planning/path", 2);
	}
	ros::Publisher tree_node_pub, tree_edge_pub, path_pub;
	ros::Subscriber goal_sub, pcl_sub;

	// void setGoalCallback(const geometry_msgs::PoseStamped &goal_msg);
	void pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
	void visualizeTrees();
	void visualizePath(vector<kdRRT_Node> path);

	//RRT() {}
	inline Vector3d sample();
	void update_cost(kdRRT_NodePtr p);
	kdRRT_NodePtr extend(Vector3d p);
	kdRRT_NodePtr sensible_extend(void);
	bool try_connect(kdRRT_NodePtr p);
	vector<kdRRT_Node> find_path(const PCTrajNode& start, const PCTrajNode& goal);
	
	void free_pool(void)
	{
		for (auto& p : start_node_pool)
		{
			delete p;
		}
		for (auto& p : goal_node_pool)
			delete p;
		start_node_pool.clear();
		goal_node_pool.clear();
	}
	~kdRRT() {
		kd_free(start_tree);
		kd_free(goal_tree);
		free_pool();
	}
};

inline Vector3d kdRRT::sample()
{
	int index = rand() % PCTrajNode::PCMap->size();
	return Vector3d(PCTrajNode::PCMap->points[index].x, PCTrajNode::PCMap->points[index].y, PCTrajNode::PCMap->points[index].z);
}
