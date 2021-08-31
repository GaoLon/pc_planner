#pragma once
#include "RRT_star.h"

struct shadow {
	PCTrajNode s[3];	//s[0]: base; s[1]: left; s[2]: right
	double delta;
};

class LocalOptimizer :
	public RRT_star
{
private:
	double din_max = 1.5*PCTrajNode::PR.d_nom;
	double din_min = 0.5*PCTrajNode::PR.d_nom;
	double w_len = 0.25;
	double w_curv = 0.25;
	double w_trav = 0.5;
	double delta_max = 0.06;
	double delta_min = 0.01; 
	vector<PCTrajNode> op_path;
	vector<shadow> Graph;
public:
	LocalOptimizer() 
	{
		din_max = 1.5*PCTrajNode::PR.d_nom;
		din_min = 0.5*PCTrajNode::PR.d_nom;
		ROS_INFO("din_max: %.2f, din_min: %.2f", din_max, din_min);
		goal_sub = nh.subscribe("/move_base_simple/goal", 10, &LocalOptimizer::setGoalCallback, this);
	}
	void visualizePath(vector<PCTrajNode> path);
	void setGoalCallback(const geometry_msgs::PoseStamped &goal_msg);
	vector<PCTrajNode> optimized_path(const PCTrajNode& start, const PCTrajNode& goal);
	void separate(void);
	double get_op_cost(size_t index, int from, int to);
	vector<int> dijk(void);
	~LocalOptimizer() {};
};

