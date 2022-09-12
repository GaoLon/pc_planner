#pragma once
#include "RRT_star.h"
#include "config.hpp"
#include "voxel_map.hpp"
#include "sdf_map.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <fstream>
#include <cmath>
struct djk_node;
typedef djk_node* djk_nodePtr; 
struct shadow {
	PCTrajNode s[3];	//s[0]: base; s[1]: left; s[2]: right
	double delta;
};
struct djk_node
{   
	int state;//0=未探索 1=扔进open里但是还没被弹出  -1从open里弹出过了，即closeset
	int id_in_graph;//	N0,N1 
	int id_in_node;//0中 1左位 2右位 

    double fScore;
    djk_nodePtr cameFrom;
    djk_node(int id_in_graph_, int id_in_node_){  
		id_in_graph = id_in_graph_;
		id_in_node = id_in_node_;
		state=0;
		fScore = numeric_limits<double>::infinity();
		cameFrom = NULL;
    }

    djk_node(){};
    ~djk_node(){};
};

class LocalOptimizer :
	public RRT_star
{
private:
	vector<PCTrajNode> op_path;
	vector<shadow> Graph;
	std::ofstream plan_time,access_time,traj_length,avg_cur;
	double traj_length_all=0,plan_time_all=0,all_average_curvature=0;
public:
	// LocalOptimizer(ros::NodeHandle &nh_):RRT_star(nh_)
	LocalOptimizer()
	{	
		std::cout<<"init LocalOptimizer"<<std::endl;
		std::cout<<"PCTrajNode::config.rrt_param.max_iter"<<PCTrajNode::config.rrt_param.max_iter<<std::endl;
		goal_sub = nh.subscribe("/goal", 10, &LocalOptimizer::setGoalCallback, this);
	    triggerSub = nh.subscribe("/initialpose", 1, &LocalOptimizer::triggerCallBack, this);
	}
	void visualizePath(vector<PCTrajNode> path);
	void pub_csd(vector<Eigen::Vector3d> x_y_theta);
	Vector2d forward(double x0,double y0,double theta0,double k0,double a, double b,double c,double s_);
	void draw_path_by_intergrate(vector<PCTrajNode> p);
	void draw_path_line(vector<PCTrajNode> p);
	void setGoalCallback(const geometry_msgs::PoseStamped &goal_msg);
	void triggerCallBack(const  geometry_msgs::PoseWithCovarianceStamped &trigger_Msg);
	vector<PCTrajNode> optimized_path(const PCTrajNode& start, const PCTrajNode& goal);
	void separate(void);
	double get_op_cost(size_t index, int from, int to);
	vector<int> dijk(void);
	~LocalOptimizer() {};
	bool first_in_cb=true;
};

