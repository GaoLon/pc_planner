#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <ros/ros.h>

struct PC_Traj_Node_param
{
    std::vector<double> d_rob;
	double d_nom;
	double kappa_max;
	double roll_max ;
	double pitch_min;
	double pitch_max;
	double rho_max;
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
struct RRT_param
{
	int max_iter;
	int heuristic_straight_thresh;
	double tau_weight;
	double step;
	double trees_merge_thresh;

};

struct RRT_Star_param
{
	double r_exp_rrtstar;
	double r_near_rrt_star;
	double rs_exp_k;
	double r_near_k;
	double path_r;
	int max_refine_num;
	double N_max_avg ;
	double N_a ;
	double N_rewire;
	double alpha ;
};

struct LocalOptimizer_param
{
	double din_max ;
	double din_min ;
	double w_len ;
	double w_curv ;
	double w_trav ;
	double delta_max ;
	double delta_min ; 
};

struct Config
{
    // Configuration of all parameters
    int use_planner;
	PC_Traj_Node_param pc_node_Param;
	RRT_param rrt_param;
	RRT_Star_param rrt_star_param;
    LocalOptimizer_param local_param;
    // Load all parameters specified by ROS script
    inline void load(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("use_planner", use_planner);
        nh_priv.getParam("pc_node_Param_d_nom", pc_node_Param.d_nom);
        nh_priv.getParam("pc_node_Param_d_rob", pc_node_Param.d_rob);
        nh_priv.getParam("pc_node_Param_eta", pc_node_Param.eta);
        nh_priv.getParam("pc_node_Param_kappa_max", pc_node_Param.kappa_max);
        nh_priv.getParam("pc_node_Param_pitch_max", pc_node_Param.pitch_max);
        nh_priv.getParam("pc_node_Param_pitch_min", pc_node_Param.pitch_min);
        nh_priv.getParam("pc_node_Param_r_plane", pc_node_Param.r_plane);
        nh_priv.getParam("pc_node_Param_r_res", pc_node_Param.r_res);
        nh_priv.getParam("pc_node_Param_rho_max", pc_node_Param.rho_max);
        nh_priv.getParam("pc_node_Param_roll_max", pc_node_Param.roll_max);
        nh_priv.getParam("pc_node_Param_traversable_threshold", pc_node_Param.traversable_threshold);
        nh_priv.getParam("pc_node_Param_w_pitch", pc_node_Param.w_pitch);
        nh_priv.getParam("pc_node_Param_w_rho", pc_node_Param.w_rho);
        nh_priv.getParam("pc_node_Param_w_roll", pc_node_Param.w_roll);
        nh_priv.getParam("pc_node_Param_normal_max", pc_node_Param.normal_max);
        nh_priv.getParam("pc_node_Param_w_normal", pc_node_Param.w_normal);

        nh_priv.getParam("rrt_param_heuristic_straight_thresh", rrt_param.heuristic_straight_thresh);
        nh_priv.getParam("rrt_param_max_iter", rrt_param.max_iter);
        nh_priv.getParam("rrt_param_step", rrt_param.step);
        nh_priv.getParam("rrt_param_tau_weight", rrt_param.tau_weight);
        nh_priv.getParam("rrt_param_trees_merge_thresh", rrt_param.trees_merge_thresh);

        nh_priv.getParam("rrt_star_param_alpha", rrt_star_param.alpha);
        nh_priv.getParam("rrt_star_param_max_refine_num", rrt_star_param.max_refine_num);
        nh_priv.getParam("rrt_star_param_N_a", rrt_star_param.N_a);
        nh_priv.getParam("rrt_star_param_N_max_avg", rrt_star_param.N_max_avg);
        nh_priv.getParam("rrt_star_param_N_rewire", rrt_star_param.N_rewire);
        nh_priv.getParam("rrt_star_param_path_r", rrt_star_param.path_r);
        nh_priv.getParam("rrt_star_param_r_exp_rrtstar", rrt_star_param.r_exp_rrtstar);
        nh_priv.getParam("rrt_star_param_r_near_k", rrt_star_param.r_near_k);
        nh_priv.getParam("rrt_star_param_r_near_rrt_star", rrt_star_param.r_near_rrt_star);
        nh_priv.getParam("rrt_star_param_rs_exp_k", rrt_star_param.rs_exp_k);

        nh_priv.getParam("local_param_delta_max", local_param.delta_max);
        nh_priv.getParam("local_param_delta_min", local_param.delta_min);
        nh_priv.getParam("local_param_din_max", local_param.din_max);
        nh_priv.getParam("local_param_din_min", local_param.din_min);
        nh_priv.getParam("local_param_w_curv", local_param.w_curv);
        nh_priv.getParam("local_param_w_len", local_param.w_len);
        nh_priv.getParam("local_param_w_trav", local_param.w_trav);

    }
};

#endif