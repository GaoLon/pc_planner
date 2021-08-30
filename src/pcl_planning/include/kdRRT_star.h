#pragma once
#include "kdRRT.h"

class kdRRT_star :
	public kdRRT
{
private:
	double r_near = 3 * PCTrajNode::PR.d_nom;
	double rs_exp_k;
	double r_near_k;
	int max_refine_num;
	vector<kdRRT_Node> path;
public:
	kdRRT_star()
	{
		nh.getParam("r_near_k", r_near_k);
		nh.getParam("max_refine_num", max_refine_num);
		r_near = r_near_k*PCTrajNode::PR.d_nom;
		ROS_INFO("r_near_k: %.2f", r_near_k);
		goal_sub = nh.subscribe("/move_base_simple/goal", 10, &kdRRT_star::setGoalCallback, this);
	}
	vector<kdRRT_Node> refined_path(const PCTrajNode& start, const PCTrajNode& goal);
	void setGoalCallback(const geometry_msgs::PoseStamped &goal_msg);
	kdRRT_NodePtr extend(Vector3d p);
    kdRRT_NodePtr sensible_extend(void);
    kdRRT_NodePtr cpar(kdRRT_NodePtr p);
	void update_cost_in_rrt_star(kdRRT_NodePtr p);
};
