#include "RRT_star.h"

extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
extern bool mapReceived;

void RRT_star::setGoalCallback(const geometry_msgs::PoseStamped &goal_msg)
{
	if (mapReceived)
	{
		tf::Quaternion q;
		tf::quaternionMsgToTF(goal_msg.pose.orientation, q);
		tf::Matrix3x3 R(q);

		Matrix4d goal_mat = Matrix4d::Identity();
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				goal_mat(i, j) = R[i][j];
			}
		}
		goal_mat(0, 3) = goal_msg.pose.position.x;
		goal_mat(1, 3) = goal_msg.pose.position.y;
		goal_mat(2, 3) = goal_msg.pose.position.z;
		//TODO
		PCTrajNode pcn(goal_mat), pseudoOdom(Matrix4d::Identity());
		refined_path(pseudoOdom, pcn);
		visualizeTrees();
	}
	else
	{
		ROS_INFO("No map received yet! Can't plan now.");
	}
}

Vector3d RRT_star::op_sample(float d)
{
	// sample vertex on path
	int index = rand() % path.size();
	RRT_Node sp = path[index];

	// sample point on map
	/// find near point clouds
	Vector3d p;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	Vector3d pos = sp.node.get_pos();
	pcl::PointXYZI searchPoint;
	searchPoint.x = pos[0];
	searchPoint.y = pos[1];
	searchPoint.z = pos[2];
	kdtree.radiusSearch(searchPoint, d, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	/// sample
	int pindex = rand() % pointIdxRadiusSearch.size();
	return { PCTrajNode::PCMap->points[pointIdxRadiusSearch[pindex]].x,\
		PCTrajNode::PCMap->points[pointIdxRadiusSearch[pindex]].y,\
		PCTrajNode::PCMap->points[pointIdxRadiusSearch[pindex]].z };
}

vector<pair<bool, int>> RRT_star::get_near(RRT_Node p)
{
	vector<pair<bool,int>> res;

	for (auto i = 0; i < start_tree.size(); i++)
	{
		if (p.node.get_dist(start_tree[i].node) < r_near)
		{
			res.push_back({ true,i });
		}
	}

	return res;
}

int RRT_star::reconstruct(Vector3d p)
{
	int n = 0;

	// find nearist vertex
	// cout << "find nearist vertex" << endl;
	pair<bool, int> nearss = { true, find_nearest(p) };

	// projection and get p_new
	//cout << "projection and get p_new" << endl;
	RRT_Node near_node = get_Node(nearss);
	Vector2d p_pro = near_node.node.projection(p);
	double dist = p_pro.norm();
	//Vector3d p_t(rs_exp*p_pro[0] / dist, rs_exp*p_pro[1] / dist, atan2(p_pro[1], p_pro[0]));
	Vector3d p_t(rs_exp*p_pro[0] / dist, rs_exp*p_pro[1] / dist, 0.0);
	Matrix4d tr = near_node.node.Trans(p_t);
	PCTrajNode pc_new(tr);
	if (!pc_new.isTraversable())
		return n;
	RRT_Node p_new(pc_new, {true,-1});
	
	// find perfect parent
	vector<pair<bool, int>> nears = get_near(p_new);
	// cout << "nears size: " << nears.size() << endl;
	if (N_a == 0)
		N_a = nears.size();
	N_a = alpha * nears.size() + (1 - alpha)*N_a;
	cout << N_a << endl;
	double d = std::numeric_limits<double>::infinity();
	pair<bool, int> upmost = { true,-1 };
	for (auto& i : nears)
	{
		RRT_Node p(get_Node(i));
		vector<PCTrajNode> srt = PCTrajNode::SRT_Generate(p.node, p_new.node);
		if (srt.empty())
		{
			continue;
		}

		double dist = p.cost;
		for (size_t j = 0; j < srt.size() - 1; j++)
		{
			dist += srt[j].get_cost();
		}
		if (dist < d)
		{
			d = dist;
			upmost = i;
		}
	}

	if (upmost.second == -1)
		return n;

	// add nodes to tree
	// cout << "add nodes to tree" << endl;
	vector<PCTrajNode> usrt = PCTrajNode::SRT_Generate(get_Node(upmost).node, p_new.node);
	pair<bool, int> tp = upmost;
	for (size_t j = 1; j < usrt.size(); j++)
	{
		RRT_Node temp(usrt[j],tp);
		temp.cost = get_Node(tp).cost + usrt[j - 1].get_cost();
		start_tree.push_back(temp);
		tp = { true, start_tree.size() - 1 };
	}

	if (upmost == nearss)
		return n;

	// cout << "rewire" << endl;
	for (size_t i = 0; i < nears.size(); i++)
	{
		vector<PCTrajNode> nsrt = PCTrajNode::SRT_Generate(p_new.node, get_Node(nears[i]).node);
		if (nsrt.empty())
			continue;

		double dist = get_Node(tp).cost;
		for (size_t j = 0; j < nsrt.size() - 1; j++)
		{
			dist += nsrt[j].get_cost();
		}

		RRT_Node& ni = get_Node(nears[i]);
		if (dist < ni.cost)
		{
			n++;
			pair<bool, int> tpnew = tp;
			for (size_t j = 1; j < nsrt.size() - 1; j++)
			{
				RRT_Node temp(nsrt[j], tpnew);
				temp.cost = get_Node(tpnew).cost + nsrt[j - 1].get_cost();
				start_tree.push_back(temp);
				tpnew = { true, start_tree.size() - 1 };
			}
			ni.parent = tpnew;
			ni.cost = dist;
			update_cost(nears[i]);
		}
	}

	N_rewire += n;
	cout << "rewired " << n << " nodes" << endl;
	return n;
}

void RRT_star::update_cost_in_rrt_star(pair<bool, int> p)
{
	if (p.first)
	{
		for (size_t i = 0; i < start_tree.size(); i++)
		{
			
			if (start_tree[i].parent == p)
			{
				double c = start_tree[p.second].node.get_cost();
				if (c==0)
				{
					c = PCTrajNode::get_connect(start_tree[p.second].node, start_tree[i].node)[4];
				}
				start_tree[i].cost = start_tree[p.second].cost + c;
				update_cost_in_rrt_star({true, i});
			}
		}
	}
	else
	{
		for (size_t i = 0; i < goal_tree.size(); i++)
		{
			
			if (goal_tree[i].parent == p)
			{
				goal_tree[i].cost = goal_tree[p.second].cost + PCTrajNode::get_connect( goal_tree[i].node,goal_tree[p.second].node)[4];
				update_cost_in_rrt_star({false, i});
			}
		}
	}
}

pair<bool, int> RRT_star::extend(Vector3d p)
{
	//static int debug;
	//cout << "extend: " << debug++ << endl;
	int nearest = find_nearest(p);

	if (nearest == -1) //which suggests that the tree is empty now
	{
		return {true, -1}; // second is -1 means that it's invalid
	}

	Matrix4d T_new = Matrix4d::Identity();
	double dist = std::numeric_limits<double>::infinity();
	int new_index = -1;

	// find nearist extend traj
	for (auto index : (*direct.second)[nearest].no_use)
	{
		Matrix4d T_i;
		if (direct.first)
		{
			T_i = (*direct.second)[nearest].node.Trans(traj_lib[index]);
		}
		else
		{
			T_i = (*direct.second)[nearest].node.Trans(traj_lib_back[index]);
		}

		double d = (p - T_i.topRightCorner(3, 1)).norm();

		if (d < dist)
		{
			dist = d;
			new_index = index;
			T_new = T_i;
		}
	}

	// update node's state
	(*direct.second)[nearest].no_use.erase(new_index);
	if ((*direct.second)[nearest].no_use.empty())
	{
		(*direct.second)[nearest].useful = false;
	}

	// check traversable、choose parent and rewire
	PCTrajNode pc_new(T_new);
	if (pc_new.isTraversable())
	{
		// find perfect parent
		RRT_Node p_new(pc_new, {direct.first, nearest});
		p_new.cost = (*direct.second)[nearest].cost + traj_cost[new_index];
		p_new.traj_idx = new_index;

		// find near nodes
		vector<int> nears;
		for (auto i = 0; i < (*direct.second).size(); i++)
		{
			if (p_new.node.get_dist((*direct.second)[i].node) < r_near)
			{
				nears.push_back(i);
			}
		}

		// find dist
		double d = std::numeric_limits<double>::infinity();
		int upmost = -1;
		for (auto& i : nears)
		{
			RRT_Node p = (*direct.second)[i];
			vector<PCTrajNode> srt;
			if (direct.first)
				srt= PCTrajNode::SRT_Generate(p.node, p_new.node);
			else
				srt= PCTrajNode::SRT_Generate(p_new.node, p.node);
			if (srt.empty())
			{
				continue;
			}
			double dist = p.cost;
			for (size_t j = 0; j < srt.size() - 1; j++)
			{
				dist += srt[j].get_cost();
			}
			if (dist < d)
			{
				d = dist;
				upmost = i;
			}
		}

		if (upmost == -1||upmost == nearest )
		{
			(*direct.second).push_back(p_new);
			return {direct.first, (*direct.second).size() - 1};
		}

		// add nodes to tree
		vector<PCTrajNode> usrt;
		if (direct.first)
		{
			usrt= PCTrajNode::SRT_Generate((*direct.second)[upmost].node, p_new.node);
			int tp = upmost;
			for (size_t j = 1; j < usrt.size(); j++)
			{
				RRT_Node temp(usrt[j],{direct.first,tp});
				temp.cost = (*direct.second)[tp].cost + usrt[j - 1].get_cost();
				start_tree.push_back(temp);
				tp = start_tree.size() - 1;
			}

			// rewire
			for (size_t i = 0; i < nears.size(); i++)
			{
				vector<PCTrajNode> nsrt = PCTrajNode::SRT_Generate(p_new.node, start_tree[nears[i]].node);
				if (nsrt.empty())
					continue;

				double dist =start_tree[tp].cost;
				for (size_t j = 0; j < nsrt.size() - 1; j++)
				{
					dist += nsrt[j].get_cost();
				}

				RRT_Node& ni = start_tree[nears[i]];
				if (dist < ni.cost)
				{
					pair<bool, int> tpnew = {true,tp};
					for (size_t j = 1; j < nsrt.size() - 1; j++)
					{
						RRT_Node temp(nsrt[j], tpnew);
						temp.cost = get_Node(tpnew).cost + nsrt[j - 1].get_cost();
						start_tree.push_back(temp);
						tpnew = { true, start_tree.size() - 1 };
					}
					ni.parent = tpnew;
					ni.cost = dist;
					
					// update cost
					update_cost_in_rrt_star({true, nears[i]});
				}
			}
		}
		else
		{
			usrt= PCTrajNode::SRT_Generate(p_new.node,(*direct.second)[upmost].node);
			int tp = upmost;
			for (int j = usrt.size()-2; j >= 0; j--)
			{
				RRT_Node temp(usrt[j],{direct.first,tp});
				temp.cost = (*direct.second)[tp].cost + usrt[j].get_cost();
				goal_tree.push_back(temp);
				tp = goal_tree.size() - 1;
			}
			
			// rewire
			for (size_t i = 0; i < nears.size(); i++)
			{
				vector<PCTrajNode> nsrt = PCTrajNode::SRT_Generate( goal_tree[nears[i]].node,p_new.node);
				if (nsrt.empty())
					continue;

				double dist =goal_tree[tp].cost;
				for (size_t j = 0; j < nsrt.size() - 1; j++)
				{
					dist += nsrt[j].get_cost();
				}

				RRT_Node& ni = goal_tree[nears[i]];
				if (dist < ni.cost)
				{
					pair<bool, int> tpnew = {true,tp};
					for (size_t j = nsrt.size() - 2; j >=  1; j--)
					{
						RRT_Node temp(nsrt[j], tpnew);
						temp.cost = get_Node(tpnew).cost + nsrt[j].get_cost();
						goal_tree.push_back(temp);
						tpnew = { true, goal_tree.size() - 1 };
					}
					ni.parent = tpnew;
					ni.cost = dist;
					update_cost_in_rrt_star({false, nears[i]});
				}
			}
		}
		return {direct.first, (*direct.second).size() - 1};
	}
	else
	{
		// ROS_INFO("Not traversable");
		return {true, -1}; // not traversable, invalid
	}
}

vector<RRT_Node> RRT_star::refined_path(const PCTrajNode& start, const PCTrajNode& goal)
{
	/*---- do optimization directly ----*/
	// inProcess = true;
	// // initialization
	// vector<RRT_Node> result;
	// if(clear_start_tree)
	// {
	// 	start_tree.clear();
	// }
	// goal_tree.clear();
	// RRT_Node s(start, {true, -1});
	// RRT_Node g(goal, {false, -1});
	// s.cost = 0;
	// g.cost = 0;
	// start_tree.push_back(s);
	// goal_tree.push_back(g);

	// // begin iteration
	// cout << "RRT_star begin!" << endl;
	// pcl::StopWatch time;
	// for (int i = 0; i < max_iter; i++)
	// {
	// 	// sample, extend this tree
	// 	// cout << "iteration: " << i << endl;
	// 	if (i % 10 == 0)
	// 	{
	// 		ROS_INFO("Iteration: %d", i);
	// 		visualizeTrees();
	// 	}
	// 	Vector3d p = sample();
	// 	//visualize
	// 	publishSamplePoint(p);
	// 	pair<bool, int> p_new = extend(p);

	// 	if (p_new.second != -1) // if it's valid
	// 	{
	// 		// change, and try connect with that tree
	// 		change_direction();		//
	// 		if (try_connect(p_new)) // path found
	// 		{
	// 			cout << "get path in iteration: " << i << endl;
	// 			cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
	// 			goal_idx = start_tree.size() - 1;
	// 			return get_path();
	// 		}
	// 		// can't connect, from that tree extend to p_new, try connect continuely
	// 		else
	// 		{
	// 			Vector3d p_new_pos = get_Node(p_new).node.get_pos();
	// 			pair<bool, int> p_t = extend(p_new_pos);
	// 			int cnt = 0;
	// 			while (p_t.second != -1)
	// 			{
	// 				cnt++;
	// 				if (cnt >= heuristic_straight_thresh)
	// 				{
	// 					break;
	// 				}
	// 				change_direction();
	// 				if (try_connect(p_t))
	// 				{
	// 					cout << "get path in  iteration: " << i << endl;
	// 					cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
	// 					goal_idx = start_tree.size() - 1;
	// 					return get_path();
	// 				}
	// 				change_direction();
	// 				p_t = extend(p_new_pos);
	// 			}
	// 		}
	// 	}
	// }

	// // can't find path
	// cout << "can't find path!!!" << endl;
	// ROS_INFO("start tree size: %d, goal tree size: %d", start_tree.size(), goal_tree.size());
	// inProcess = false;
	// return result;

	/*---- do rrt first, than optimization ----*/
	path = find_path(start, goal);

	if (path.empty())
	{
		cout << "can't refine path! RRT failed!" << endl;
		inProcess = false;
		return path;
	}

	float d = path_r*path.size()*PCTrajNode::PR.d_nom;
	N_ma = (int)d*3;

	if (!direct.first)
		change_direction();

	int refine_num = 0;
	N_a = 0;
	N_rewire = 0;
	cout << "begin refine path" << endl;
	pcl::StopWatch time;
	while (N_rewire < N_ma)
	{
		Vector3d p = op_sample(d);
		// reconstruct(p);
		N_rewire = alpha * reconstruct(p) + (1 - alpha)*N_rewire;
		cout << "refine iteration: " << ++refine_num << endl;
		if (refine_num > max_refine_num)
		{
			break;
		}
	}
	cout << "poject done, refine consume: " << time.getTime() / 1000 << "s" << std::endl;
	cout << "refine nodes number: " << N_rewire << endl;
	path = get_path();

	return  path;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RRT_star_node");
	ROS_INFO("Planner started!");
	RRT_star planner;
	ros::spin();
}