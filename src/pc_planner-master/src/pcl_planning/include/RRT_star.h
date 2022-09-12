#pragma once
#include "RRT.h"
#include <config.hpp>
class RRT_star :
	public RRT
{
private:
	vector<RRT_Node> path;
public:
	// RRT_star(ros::NodeHandle &nh_)
	// :RRT(nh_)

	RRT_star()
	{
		std::cout<<"init RRT_star"<<std::endl;
		PCTrajNode::config.rrt_star_param.r_exp_rrtstar = PCTrajNode::config.rrt_star_param.rs_exp_k* PCTrajNode::PR.d_nom;
		PCTrajNode::config.rrt_star_param.r_near_rrt_star = PCTrajNode::config.rrt_star_param.r_near_k*PCTrajNode::PR.d_nom;
		ROS_WARN("rrt_star_param.rs_exp_k: %.2f, rrt_star_param.r_near_k: %.2f, config.rrt_star_param.path_r: %.2f", PCTrajNode::config.rrt_star_param.rs_exp_k,PCTrajNode::config.rrt_star_param.r_near_k,PCTrajNode::config.rrt_star_param.path_r);
		ROS_WARN("rrt_star_param.r_exp_rrtstar: %.2f, rrt_star_param.r_near_rrt_star: %.2f", PCTrajNode::config.rrt_star_param.r_exp_rrtstar,PCTrajNode::config.rrt_star_param.r_near_rrt_star);
		// goal_sub = nh.subscribe("/move_base_simple/goal", 10, &RRT_star::setGoalCallback, this);
	}
	int opsample_index;
	Vector3d op_sample(float d);
	int reconstruct(Vector3d p);
	vector<pair<bool, int>> get_near(RRT_Node p);
	vector<RRT_Node> refined_path(const PCTrajNode& start, const PCTrajNode& goal);
	void setGoalCallback(const geometry_msgs::PoseStamped &goal_msg);
	pair<bool, int> extend(Vector3d p);
	void update_cost_in_rrt_star(pair<bool, int> p);
};

