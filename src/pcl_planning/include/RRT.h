#pragma once
#include "PCTrajNode.h"
#include "cubic_curvature.h"
#include <ctime>
#include <cstdlib>
#include <utility>
#include <algorithm>
#include <set>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

struct RRT_Node
{
	PCTrajNode node;
	pair<bool, int> parent;
	bool useful = true;
	double cost = std::numeric_limits<double>::infinity();
	int traj_idx = 13;
	set<int> no_use{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
	RRT_Node(PCTrajNode a, pair<bool, int> b) : node(a), parent(b) {}
	~RRT_Node() {}
};

class RRT
{
protected:
	ros::NodeHandle nh;
	bool inProcess = false;
	int max_iter;
	int heuristic_straight_thresh;
	double tau_weight;
	double step;
	bool clear_start_tree;
	double trees_merge_thresh = PCTrajNode::PR.d_nom * 3;
	Vector3d g2s = Vector3d::Zero();
	Vector3d s2g = Vector3d::Zero();
	vector<RRT_Node> start_tree;
	vector<RRT_Node> goal_tree;
	int goal_idx = -1;
	pair<bool, vector<RRT_Node> *> direct = make_pair(true, &start_tree);																																																																																																																																							 // true: from start to goal
	Vector3d traj_lib[13] = {{0.6, 0, 0}, {0.599250156236980, 0.029987501562407, 0.1}, {0.599250156236980, -0.029987501562407, -0.1}, {0.597002499166815, 0.059900049988097, 0.2}, {0.597002499166815, -0.059900049988097, -0.2}, {0.593262646761625, 0.089662879484160, 0.3}, {0.593262646761625, -0.089662879484160, -0.3}, {0.588039946704745, 0.119201598477037, 0.4}, {0.588039946704745, -0.119201598477037, -0.4}, {0.581347453026387, 0.148442375552714, 0.5}, {0.581347453026387, -0.148442375552714, -0.5}, {0.573201893475364, 0.177312123996804, 0.6}, {0.573201893475364, -0.177312123996804, -0.6}};					 //x, y, theta
	Vector3d traj_lib_back[13] = {{-0.6, 0, 0}, {-0.599250156236980, 0.029987501562407, -0.1}, {-0.599250156236980, -0.029987501562407, 0.1}, {-0.597002499166815, 0.059900049988097, -0.2}, {-0.597002499166815, -0.059900049988097, 0.2}, {-0.593262646761625, 0.089662879484160, -0.3}, {-0.593262646761625, -0.089662879484160, 0.3}, {-0.588039946704745, 0.119201598477037, -0.4}, {-0.588039946704745, -0.119201598477037, 0.4}, {-0.581347453026387, 0.148442375552714, -0.5}, {-0.581347453026387, -0.148442375552714, 0.5}, {-0.573201893475364, 0.177312123996804, -0.6}, {-0.573201893475364, -0.177312123996804, 0.6}}; //x, y, theta
	double traj_cost[14] = {0.6, 0.600364448468052, 0.600364448468052,
							0.601458896685019, 0.601458896685019, 0.603283799402430, 0.603283799402430,
							0.605833261322673, 0.605833261322673, 0.609086090310233, 0.609086090310233,
							0.612995520383789, 0.612995520383789, 0}; //sf
	// Vector3d traj_lib[11] = { {0.16,0,0}, { 0.159992000066666, 0.001599973333467,0.03}, \
	// { 0.159992000066666, -0.001599973333467, -0.03} , { 0.159928005399838 ,  0.004799280032399 ,0.06},\
	// { 0.159928005399838, -0.004799280032399, -0.06}, { 0.159872017065756 ,  0.006398293469861 ,0.09},\
	// { 0.159872017065756, -0.006398293469861, -0.09}, { 0.159712086389633 ,  0.009594241036711,0.12 },\
	// { 0.159712086389633, -0.009594241036711, -0.12 }, { 0.159608160040525 ,  0.011190855574005 ,0.15},\
	// { 0.159608160040525, -0.011190855574005, -0.15} };		//x, y, theta
	// Vector3d traj_lib_back[11] = { {-0.16,0,0}, { -0.159992000066666, 0.001599973333467,-0.03}, \
	// { -0.159992000066666, -0.001599973333467, 0.03} , { -0.159928005399838 ,  0.004799280032399 ,-0.06},\
	// { -0.159928005399838, -0.004799280032399, 0.06}, { -0.159872017065756 ,  0.006398293469861 ,-0.09},\
	// { -0.159872017065756, -0.006398293469861, 0.09}, { -0.159712086389633 ,  0.009594241036711,-0.12 },\
	// { -0.159712086389633, -0.009594241036711, 0.12 }, { -0.159608160040525 ,  0.011190855574005 ,-0.15},\
	// { -0.159608160040525, -0.011190855574005, 0.15} };		//x, y, theta
	// double traj_cost[12] = { 0.16,0.1600288 ,0.1600288,0.1601152 ,0.1601152 ,\
	// 	0.1602592, 0.1602592, 0.1604608, 0.1604608, 0.16072, 0.16072, 0 };		//sf
	vector<RRT_Node> get_path();

	// PCTrajNode start, goal;
public:
	RRT()
	{
		nh = ros::NodeHandle("~");
		// PCTrajNode id(Matrix4d::Identity());
		// start=id;
		// goal=id;
		nh.getParam("step", step);
		nh.getParam("max_iter", max_iter);
		nh.getParam("heuristic_straight_thresh", heuristic_straight_thresh);
		nh.getParam("tau_weight", tau_weight);
		nh.getParam("clear_start_tree", clear_start_tree);
		PCTrajNode::PR.d_nom = step;
		ROS_INFO("step: %.2f, max_iter: %d, heuristic loop times: %d", step, max_iter, heuristic_straight_thresh);

		float k = step / 0.6; //0.6 is the step size of the trajectory library
		for (auto i = 0; i < 13; i++)
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

		gsl_set_error_handler_off();

		sample_pub = nh.advertise<geometry_msgs::PointStamped>("/sample_point", 10);
		pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/overall_map", 2, &RRT::pclMapCallback, this);
		// stop_sub = nh.subscribe("/rugged_car/tracking/stop", 1, &stanley_control::stop_callback, this);

		// goal_sub = nh.subscribe("/move_base_simple/goal", 10, &RRT::setGoalCallback, this);
		tree_node_pub = nh.advertise<geometry_msgs::PoseArray>("/planning/tree_poses", 2);
		tree_edge_pub = nh.advertise<visualization_msgs::Marker>("/planning/tree_edges", 2);
		path_pub = nh.advertise<geometry_msgs::PoseArray>("/planning/path", 2);
	}
	//RRT() {}
	
	ros::Publisher sample_pub, tree_node_pub, tree_edge_pub, path_pub;
	ros::Subscriber goal_sub, pcl_sub;
	inline Vector3d sample();
	inline void change_direction();
	inline RRT_Node &get_Node(pair<bool, int> p);
	void update_cost(pair<bool, int> p);
	int find_nearest(Vector3d p);
	pair<bool, int> extend(Vector3d p);
	bool try_connect(pair<bool, int> p);

	vector<RRT_Node> find_path(const PCTrajNode &start, const PCTrajNode &goal);
	// vector<RRT_Node> find_path();

	const vector<RRT_Node> *get_start_tree() { return &start_tree; }
	const vector<RRT_Node> *get_goal_tree() { return &goal_tree; }

	void publishSamplePoint(Vector3d point);
	void setGoalCallback(const geometry_msgs::PoseStamped &goal_msg);
	void pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
	void visualizeTrees();
	void visualizePath(vector<RRT_Node> path);
	//void visual_rrt(void);
	~RRT() {}
};

inline Vector3d RRT::sample()
{
	int index = rand() % PCTrajNode::PCMap->size();
	return Vector3d(PCTrajNode::PCMap->points[index].x, PCTrajNode::PCMap->points[index].y, PCTrajNode::PCMap->points[index].z);
}

inline void RRT::change_direction()
{
	if (direct.first)
	{
		direct.first = false;
		direct.second = &goal_tree;
	}
	else
	{
		direct.first = true;
		direct.second = &start_tree;
	}
}

inline RRT_Node &RRT::get_Node(pair<bool, int> p)
{
	if (p.first)
		return start_tree[p.second];
	else
		return goal_tree[p.second];
}