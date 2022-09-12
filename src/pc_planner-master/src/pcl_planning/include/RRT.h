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
#include <nav_msgs/Odometry.h>
#include <config.hpp>
#include <visualization_msgs/Marker.h>
//init node,parent
struct RRT_Node
{
	PCTrajNode node;
	pair<bool, int> parent;
	bool useful = true;
	double cost = std::numeric_limits<double>::infinity();
	int traj_idx = 13;
	set<int> no_use{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,   13,14,15,16};
	RRT_Node(PCTrajNode a, pair<bool, int> b) : node(a), parent(b) {}
	~RRT_Node() {}
};

class RRT
{
private:

protected:
	ros::NodeHandle nh;
	bool inProcess = false;
	bool clear_start_tree;

	Vector3d g2s = Vector3d::Zero();
	Vector3d s2g = Vector3d::Zero();
	vector<RRT_Node> start_tree;
	vector<RRT_Node> goal_tree;
	int goal_idx = -1;
	pair<bool, vector<RRT_Node> *> direct = make_pair(true, &start_tree);	
	
	Vector3d traj_lib[17] = 
	{{2, 0, 0}, 
	{1.9975, 0.1, 0.1}, {1.9975, -0.1, -0.1}, 
	{1.99, 0.1997, 0.2}, {1.99, -0.1997, -0.2},
	{1.9775, 0.2989, 0.3}, {1.9775, -0.2989, -0.3}, 
	{1.9601, 0.3973, 0.4}, {1.9601, -0.3973, -0.4}, 
	{1.9378,0.4948,0.5},{1.9378,-0.4948,-0.5},
	{1.9107,0.5910,0.6},{1.9107,-0.5910,-0.6},    
	{1.5297,1.2884,1.5},{1.5297,-1.2884,-1.5},
	{1.086,1.6829,1.5},{1.086,-1.6829,-1.5}};
	Vector3d traj_lib_back[17] = 
	{{-2, 0, 0}, 
	{-1.9975, 0.1, 		-0.1}, {-1.9975, -0.1, 		0.1}, 
	{-1.99, 0.1997, 	-0.2}, {-1.99, -0.1997, 	0.2},
	{-1.9775, 0.2989, 	-0.3}, {-1.9775, -0.2989, 	0.3}, 
	{-1.9601, 0.3973, 	-0.4}, {-1.9601, -0.3973, 	0.4}, 
	{-1.9378,0.4948,	-0.5},{-1.9378,-0.4948,		0.5},
	{-1.9107,0.5910,	-0.6},{-1.9107,-0.5910,		0.6},   
	{-1.5297,1.2884,	-1.5},{-1.5297,-1.2884,		1.5},
	{-1.086,1.6829,		-1.5},{-1.086,-1.6829,		1.5}};
	double traj_cost[18] = {2, 2.0012, 2.0012,
							2.0049, 2.0049, 2.0110, 2.0110,
							2.0196, 2.0196, 2.0307, 2.0307,
							2.0444, 2.0444,
							 2.3050,2.3050,2.3304,2.3304,//最后四个是人为补充的四条轨迹
							 0};

	// Vector3d traj_lib[13] = {{0.6, 0, 0}, {0.599250156236980, 0.029987501562407, 0.1}, {0.599250156236980, -0.029987501562407, -0.1}, {0.597002499166815, 0.059900049988097, 0.2}, {0.597002499166815, -0.059900049988097, -0.2}, {0.593262646761625, 0.089662879484160, 0.3}, {0.593262646761625, -0.089662879484160, -0.3}, {0.588039946704745, 0.119201598477037, 0.4}, {0.588039946704745, -0.119201598477037, -0.4}, {0.581347453026387, 0.148442375552714, 0.5}, {0.581347453026387, -0.148442375552714, -0.5}, {0.573201893475364, 0.177312123996804, 0.6}, {0.573201893475364, -0.177312123996804, -0.6}};					 //x, y, theta
	// Vector3d traj_lib_back[13] = {{-0.6, 0, 0}, {-0.599250156236980, 0.029987501562407, -0.1}, {-0.599250156236980, -0.029987501562407, 0.1}, {-0.597002499166815, 0.059900049988097, -0.2}, {-0.597002499166815, -0.059900049988097, 0.2}, {-0.593262646761625, 0.089662879484160, -0.3}, {-0.593262646761625, -0.089662879484160, 0.3}, {-0.588039946704745, 0.119201598477037, -0.4}, {-0.588039946704745, -0.119201598477037, 0.4}, {-0.581347453026387, 0.148442375552714, -0.5}, {-0.581347453026387, -0.148442375552714, 0.5}, {-0.573201893475364, 0.177312123996804, -0.6}, {-0.573201893475364, -0.177312123996804, 0.6}}; //x, y, theta
	// double traj_cost[14] = {0.6, 0.600364448468052, 0.600364448468052,
	// 						0.601458896685019, 0.601458896685019, 0.603283799402430, 0.603283799402430,
	// 						0.605833261322673, 0.605833261322673, 0.609086090310233, 0.609086090310233,
	// 						0.612995520383789, 0.612995520383789, 0}; //sf




	// Vector3d [11] = { {0.16,0,0}, { 0.159992000066666, 0.001599973333467,0.03}, \
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
	std::ofstream rrt_star_info;
	ros::Publisher sample_pub,sample_pub_2, tree_node_pub, tree_edge_pub, path_pub,_raw_map_pub,_actual_map_pub,csd_pub,line_pub;
	ros::Subscriber goal_sub, pcl_sub,triggerSub;
public:
	double pre_time=0;
	double all_average_curvature=0;
	bool trgger_for_debug=false;
	RRT()
	{
		std::cout<<"init rrt"<<std::endl;
		nh = ros::NodeHandle("~");
		PCTrajNode::config.load(nh);
		PCTrajNode::PR.d_rob(0,0)=PCTrajNode::config.pc_node_Param.d_rob[0];
		PCTrajNode::PR.d_rob(1,0)=PCTrajNode::config.pc_node_Param.d_rob[1];
		PCTrajNode::PR.d_rob(2,0)=PCTrajNode::config.pc_node_Param.d_rob[2];
		PCTrajNode::PR.d_rob(3,0)=PCTrajNode::config.pc_node_Param.d_rob[3];
		PCTrajNode::PR.d_nom=PCTrajNode::config.pc_node_Param.d_nom;
		PCTrajNode::PR.eta=PCTrajNode::config.pc_node_Param.eta;
		PCTrajNode::PR.kappa_max=PCTrajNode::config.pc_node_Param.kappa_max;
		PCTrajNode::PR.pitch_max=PCTrajNode::config.pc_node_Param.pitch_max;
		PCTrajNode::PR.pitch_min=PCTrajNode::config.pc_node_Param.pitch_min;
		PCTrajNode::PR.r_plane=PCTrajNode::config.pc_node_Param.r_plane;
		PCTrajNode::PR.r_res=PCTrajNode::config.pc_node_Param.r_res; 
		PCTrajNode::PR.rho_max=PCTrajNode::config.pc_node_Param.rho_max;
		PCTrajNode::PR.roll_max=PCTrajNode::config.pc_node_Param.roll_max;
		PCTrajNode::PR.traversable_threshold=PCTrajNode::config.pc_node_Param.traversable_threshold;
		PCTrajNode::PR.w_pitch=PCTrajNode::config.pc_node_Param.w_pitch;
		PCTrajNode::PR.w_rho=PCTrajNode::config.pc_node_Param.w_rho;
		PCTrajNode::PR.w_roll=PCTrajNode::config.pc_node_Param.w_roll;
		PCTrajNode::PR.normal_max=PCTrajNode::config.pc_node_Param.normal_max;
		PCTrajNode::PR.w_normal=PCTrajNode::config.pc_node_Param.w_normal;
		ROS_WARN("step: %.2f, max_iter: %d, heuristic loop times: %d",PCTrajNode::config.rrt_param.step, PCTrajNode::config.rrt_param.max_iter, PCTrajNode::config.rrt_param.heuristic_straight_thresh);
		
		// float k = PCTrajNode::config.rrt_param.step / PCTrajNode::config.pc_node_Param.d_nom; //0.6 is the config.rrt_param.step size of the trajectory library
		// for (auto i = 0; i < 13; i++)
		// {
		// 	// TODO: change the scale factor?
		// 	// the trajectory library was generated using config.rrt_param.step size of 0.6m
		// 	// to adapt to the config.rrt_param.step of 0.15m, the x and y part of the trajectories should be scaled
		// 	traj_lib[i][0] *= k;
		// 	traj_lib[i][1] *= k;
		// 	traj_lib_back[i][0] *= k;
		// 	traj_lib_back[i][1] *= k;
		// 	traj_cost[i] *= k;
		// };

		gsl_set_error_handler_off();
		line_pub = nh.advertise<visualization_msgs::Marker>("/line_path", 10);
  		_raw_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/raw_map", 1);
		_actual_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/actual_map", 1);
		sample_pub = nh.advertise<geometry_msgs::PointStamped>("/sample_point", 10);
		sample_pub_2 = nh.advertise<geometry_msgs::PointStamped>("/sample_point_2", 10);
		pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_pcd", 2, &RRT::pclMapCallback, this);
		// stop_sub = nh.subscribe("/rugged_car/tracking/stop", 1, &stanley_control::stop_callback, this);

		// goal_sub = nh.subscribe("/move_base_simple/goal", 10, &RRT::setGoalCallback, this);
		tree_node_pub = nh.advertise<geometry_msgs::PoseArray>("/planning/tree_poses", 2);
		tree_edge_pub = nh.advertise<visualization_msgs::Marker>("/planning/tree_edges", 2);
		path_pub = nh.advertise<geometry_msgs::PoseArray>("/planning/path", 2);
    	csd_pub =nh.advertise<nav_msgs::Odometry>("/test_csd",10);
	}

	//RRT() {}
	

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
	void publishSamplePoint_2(Vector3d point);
	void setGoalCallback(const geometry_msgs::PoseStamped &goal_msg);
	void pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
	void visualizeTrees();
	void visualizePath(vector<RRT_Node> path);
	//void visual_rrt(void);
	~RRT() {}
};

inline Vector3d RRT::sample()
{
	// int index = rand() % PCTrajNode::PCMap->size();
	// return Vector3d(PCTrajNode::PCMap->points[index].x, PCTrajNode::PCMap->points[index].y, PCTrajNode::PCMap->points[index].z);

	int index = rand() % PCTrajNode::PCMap_fiter_rho_vis->size();
	return Vector3d(PCTrajNode::PCMap_fiter_rho_vis->points[index].x, PCTrajNode::PCMap_fiter_rho_vis->points[index].y, PCTrajNode::PCMap_fiter_rho_vis->points[index].z);
	
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