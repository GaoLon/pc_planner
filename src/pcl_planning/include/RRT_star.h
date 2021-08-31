#pragma once
#include "RRT.h"

class RRT_star :
	public RRT
{
private:
	double rs_exp = 2 * PCTrajNode::PR.d_nom;
	double r_near = 6 * PCTrajNode::PR.d_nom;
	double rs_exp_k;
	double r_near_k;
	double path_r;
	int max_refine_num;
	int N_ma = 30;
	int N_a = 0;
	int N_rewire = 0;
	double alpha = 0.1;
	vector<RRT_Node> path;
public:
	RRT_star()
	{
		nh.getParam("rs_exp_k", rs_exp_k);
		nh.getParam("r_near_k", r_near_k);
		nh.getParam("path_r", path_r);
		nh.getParam("max_refine_num", max_refine_num);
		rs_exp = rs_exp_k* PCTrajNode::PR.d_nom;
		r_near = r_near_k*PCTrajNode::PR.d_nom;
		ROS_INFO("rs_exp_k: %.2f, r_near_k: %.2f, path_r: %.2f", rs_exp_k,r_near_k,path_r);
		// goal_sub = nh.subscribe("/move_base_simple/goal", 10, &RRT_star::setGoalCallback, this);
	}
	Vector3d op_sample(float d);
	int reconstruct(Vector3d p);
	vector<pair<bool, int>> get_near(RRT_Node p);
	vector<RRT_Node> refined_path(const PCTrajNode& start, const PCTrajNode& goal);
	void setGoalCallback(const geometry_msgs::PoseStamped &goal_msg);
	pair<bool, int> extend(Vector3d p);
	void update_cost_in_rrt_star(pair<bool, int> p);
};

