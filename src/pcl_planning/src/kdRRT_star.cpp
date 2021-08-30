#include "kdRRT_star.h"

extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
extern bool kdmapReceived;

void kdRRT_star::setGoalCallback(const geometry_msgs::PoseStamped &goal_msg)
{
	if (kdmapReceived)
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

void kdRRT_star:: update_cost_in_rrt_star(kdRRT_NodePtr p)
{
	for (size_t i = 0; i < start_node_pool.size(); i++)
	{
		if (start_node_pool[i]->parent == p)
		{
			double c = p->node.get_cost();
			if (c == 0)
			{
				c = PCTrajNode::get_connect(p->node, start_node_pool[i]->node)[4];
			}

			start_node_pool[i]->cost = p->cost + c;
			update_cost_in_rrt_star(start_node_pool[i]);
		}
	}

    for (size_t i = 0; i < goal_node_pool.size(); i++)
	{
		if (goal_node_pool[i]->parent == p)
		{
			double c = p->node.get_cost();
			if (c == 0)
			{
				c = PCTrajNode::get_connect(goal_node_pool[i]->node, p->node)[4];
			}

			goal_node_pool[i]->cost = p->cost + c;
			update_cost_in_rrt_star(goal_node_pool[i]);
		}
	}
}

// kdRRT_NodePtr kdRRT_star::cpar(kdRRT_NodePtr p)
// {
// 	if (direction)
// 	{
// 		start_node_pool.push_back(p);
//		Vector3d p_pos = p->node.get_pos();
//		kd_insert3(start_tree, p_pos[0], p_pos[1], p_pos[2], p);
// 	}
// 	else
// 	{
// 		goal_node_pool.push_back(p);
//		Vector3d p_pos = p->node.get_pos();
//		kd_insert3(start_tree, p_pos[0], p_pos[1], p_pos[2], p);
// 	}
// 	return p;
// }

kdRRT_NodePtr kdRRT_star::cpar(kdRRT_NodePtr p)
{
	vector<kdRRT_NodePtr>* pool;
	if (direction)
	{
		pool = &start_node_pool;
	}
	else
	{
		pool = &goal_node_pool;
	}
	kdRRT_NodePtr paren = p->parent;
	Vector3d p_pos = p->node.get_pos();
	struct kdres *p_nbr_set = kd_nearest_range3(goal_tree, p_pos[0], p_pos[1], p_pos[2],  r_near);
	if (p_nbr_set == nullptr)
	{
		//cout << "kd range query error!" << endl;
		return nullptr;
	}
	double d = std::numeric_limits<double>::infinity();
	kdRRT_NodePtr upmost = nullptr;
	vector<vector<PCTrajNode>> rewire_ram;
	vector<kdRRT_NodePtr> rewire_ptr;
	while (!kd_res_end(p_nbr_set))
	{
		kdRRT_NodePtr to_node = (kdRRT_NodePtr)kd_res_item_data(p_nbr_set);
		vector<PCTrajNode> path_pc,path_pc_back;
		kdRRT_NodePtr tp = nullptr;
		kdRRT_NodePtr srt_end = nullptr;
		if (direction)
		{
			path_pc =  PCTrajNode::SRT_Generate(to_node->node,p->node);
			path_pc_back = PCTrajNode::SRT_Generate(p->node, to_node->node);
		}
		else
		{
			path_pc = PCTrajNode::SRT_Generate(p->node, to_node->node);
			path_pc_back =  PCTrajNode::SRT_Generate(to_node->node,p->node);
		}

		rewire_ptr.push_back(to_node);
		rewire_ram.push_back(path_pc_back);
		if (path_pc.empty())
		{
			kd_res_next(p_nbr_set);
			continue;
		}

		double dist = p->cost;
		for (size_t j = 0; j < path_pc.size() - 1; j++)
		{
			dist += path_pc[j].get_cost();
		}
		if (dist < d)
		{
			d = dist;
			upmost = to_node;
		}
	}

	if (upmost == paren || upmost==nullptr)
	{
		(*pool).push_back(p);	
		Vector3d p_pos = p->node.get_pos();
		kd_insert3(start_tree, p_pos[0], p_pos[1], p_pos[2], p);
		return p;
	}

	// add parents and go on
	if (direction)
	{
		vector<PCTrajNode> path_pc =  PCTrajNode::SRT_Generate(upmost->node,p->node);
		kdRRT_NodePtr tp = upmost;
		for (size_t j = 1; j < path_pc.size(); j++)
		{
			kdRRT_NodePtr temp = new kdRRT_Node(path_pc[j], tp);
			temp->cost = tp->cost + path_pc[j - 1].get_cost();
			start_node_pool.push_back(temp);
			Vector3d temp_pos = temp->node.get_pos();
			kd_insert3(start_tree, temp_pos[0], temp_pos[1], temp_pos[2], temp);
			tp = temp;
		}

		// rewire
		for (size_t i=0;i<rewire_ptr.size();i++)
		{
			if (!rewire_ram[i].empty())
			{
				double dist =tp->cost;
				for (size_t j = 0; j < rewire_ram[i].size() - 1; j++)
				{
					dist += rewire_ram[i][j].get_cost();
				}

				if (dist < rewire_ptr[i]->cost)
				{
					kdRRT_NodePtr tpnew = tp;
					for (size_t j = 1; j <  rewire_ram[i].size()-1; j++)
					{
							kdRRT_NodePtr temp = new kdRRT_Node(rewire_ram[i][j], tpnew);
							temp->cost = tp->cost +rewire_ram[i][j - 1].get_cost();
							start_node_pool.push_back(temp);
							Vector3d temp_pos = temp->node.get_pos();
							kd_insert3(start_tree, temp_pos[0], temp_pos[1], temp_pos[2], temp);
							tpnew = temp;
					}
					rewire_ptr[i]->parent = tpnew;
					rewire_ptr[i]->cost = dist;
					update_cost_in_rrt_star(rewire_ptr[i]);
				}
			}
		}

		return tp;
	}
	else
	{
		vector<PCTrajNode> path_pc =  PCTrajNode::SRT_Generate(p->node, upmost->node);
		kdRRT_NodePtr tp = upmost;
		for (int j = path_pc.size()-2; j >=0; j--)
		{
			kdRRT_NodePtr temp = new kdRRT_Node(path_pc[j], tp);
			temp->cost = tp->cost + path_pc[j].get_cost();
			goal_node_pool.push_back(temp);
			Vector3d temp_pos = temp->node.get_pos();
			kd_insert3(start_tree, temp_pos[0], temp_pos[1], temp_pos[2], temp);
			tp = temp;
		}

		// rewire
		for (size_t i=0;i<rewire_ptr.size();i++)
		{
			if (!rewire_ram[i].empty())
			{
				double dist =tp->cost;
				for (size_t j = 0; j < rewire_ram[i].size() - 1; j++)
				{
					dist += rewire_ram[i][j].get_cost();
				}

				if (dist < rewire_ptr[i]->cost)
				{
					kdRRT_NodePtr tpnew = tp;
					for (size_t j = path_pc.size()-2; j >=1; j--)
					{
							kdRRT_NodePtr temp = new kdRRT_Node(rewire_ram[i][j], tpnew);
							temp->cost = tp->cost +rewire_ram[i][j].get_cost();
							goal_node_pool.push_back(temp);
							Vector3d temp_pos = temp->node.get_pos();
							kd_insert3(start_tree, temp_pos[0], temp_pos[1], temp_pos[2], temp);
							tpnew = temp;
					}
					rewire_ptr[i]->parent = tpnew;
					rewire_ptr[i]->cost = dist;
					update_cost_in_rrt_star(rewire_ptr[i]);
				}
			}
		}
		return tp;
	}
	
	return nullptr;
}

kdRRT_NodePtr kdRRT_star::sensible_extend(void)
{
    // initialization
	int index = -1;
	vector<kdRRT_NodePtr>* pool;
	if (direction)
	{
		pool = &start_node_pool;
	}
	else
	{
		pool = &goal_node_pool;
	}
	
	// sample a useful node
	do
	{
		index = rand() % (*pool).size();
	} while (!(*pool)[index]->useful);
	
	// sample in trajectory lib
	kdRRT_NodePtr sample_ptr = (*pool)[index];
	set<int>::const_iterator it(sample_ptr->no_use.begin());
	advance(it, rand() % sample_ptr->no_use.size());
	int new_index = *it;
	Matrix4d T_new;
	if (direction)
	{
		T_new = sample_ptr->node.Trans(traj_lib[new_index]);
	}
	else
	{
		T_new = sample_ptr->node.Trans(traj_lib_back[new_index]);
	}

	// add node
	PCTrajNode pc_new(T_new);
	if (pc_new.isTraversable())
	{
		kdRRT_NodePtr p_new = new kdRRT_Node(pc_new, sample_ptr);
		p_new->cost = sample_ptr->cost + traj_cost[new_index];
		p_new->traj_idx = new_index;
        p_new = cpar(p_new);
		return p_new;
	}
	else
	{
		return nullptr;
	}
}

kdRRT_NodePtr kdRRT_star::extend(Vector3d p)
{
	// find nearist node
	kdRRT_NodePtr nearest = nullptr;
	double dist = std::numeric_limits<double>::infinity();

	vector<kdRRT_NodePtr>* pool;
	if (direction)
	{
		pool = &start_node_pool;
	}
	else
	{
		pool = &goal_node_pool;
	}

	for (size_t i = 0; i < pool->size(); i++)
	{
		if ((*pool)[i]->useful)
		{
			double d = (*pool)[i]->node.get_dist(p);
			if (d < dist)
			{
				dist = d;
				nearest = (*pool)[i];
			}
		}
	}
	/*kdRRT_NodePtr nearest = nullptr;
	struct kdres *p_nbr_set = kd_nearest3(start_tree, p[0], p[1], p[2]);
	if (p_nbr_set == nullptr)
	{
		return nullptr;
	}
	nearest = (kdRRT_NodePtr)kd_res_item_data(p_nbr_set);
	if (!nearest->useful)
	{
		return nullptr;
	}*/
	if (nearest == nullptr)
	{
		return nullptr;
	}

	// find nearist extend traj
	Matrix4d T_new = Matrix4d::Identity();
	dist = std::numeric_limits<double>::infinity();
	int new_index = -1;

	for (auto index : nearest->no_use)
	{
		Matrix4d T_i;
		if (direction)
		{
			T_i = nearest->node.Trans(traj_lib[index]);
		}
		else
		{
			T_i = nearest->node.Trans(traj_lib_back[index]);
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
	nearest->no_use.erase(new_index);
	if (nearest->no_use.empty())
	{
		nearest->useful = false;
	}

	// check traversable and return
	//cout << "check traversable and return" << endl;
	PCTrajNode pc_new(T_new);
	if (pc_new.isTraversable())
	{
		kdRRT_NodePtr p_new = new kdRRT_Node(pc_new, nearest);
		p_new->cost = nearest->cost + traj_cost[new_index];
		p_new->traj_idx = new_index;
        p_new = cpar(p_new);
		return p_new;
	}
	else
	{
		return nullptr;
	}
}

vector<kdRRT_Node> kdRRT_star::refined_path(const PCTrajNode& start, const PCTrajNode& goal)
{
	inProcess = true;

	// initialization
	vector<kdRRT_Node> result;
	direction = true;
	start_node = new kdRRT_Node(start, nullptr);
	goal_node = new kdRRT_Node(goal, nullptr);
	start_node->cost = 0;
	goal_node->cost = 0;
	free_pool();
	goal_node_pool.push_back(goal_node);
	start_node_pool.push_back(start_node);
	Vector3d s_pos = start_node->node.get_pos();
	Vector3d g_pos = goal_node->node.get_pos();
	kd_free(start_tree);
	kd_free(goal_tree);
	start_tree = kd_create(3);
	goal_tree = kd_create(3);
	kd_insert3(start_tree, s_pos[0], s_pos[1], s_pos[2], start_node);
	kd_insert3(goal_tree, g_pos[0], g_pos[1], g_pos[2], goal_node);

	// begin iteration
	cout << "kdRRT_star begin!" << endl;
	pcl::StopWatch time;
	for (int i = 0; i < max_iter; i++)
	{
		// sample, extend this tree
		// cout << "iteration: " << i << endl;
		if (i % 10 == 0)
		{
			ROS_INFO("Iteration: %d", i);
			visualizeTrees();
		}
		//Vector3d p = sample();
		//kdRRT_NodePtr p_new = extend(p, start_tree);
		kdRRT_NodePtr p_new = sensible_extend();
		if (p_new != nullptr)
		{
			Vector3d p_new_pos = p_new->node.get_pos();
			// kd_insert3(start_tree, p_new_pos[0], p_new_pos[1], p_new_pos[2], p_new);
			// try connect with that tree
			if (try_connect(p_new))
			{
				cout << "get path in iteration: " << i << endl;
				cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
				return get_path();
			}
			// can't connect, from that tree extend to p_new, try connect continuely
			else
			{
				struct kdtree* temp = start_tree;
				start_tree = goal_tree;
				goal_tree = temp;
				direction = !direction;
				kdRRT_NodePtr p_t = extend(p_new_pos);
				int cnt = 0;
				while (p_t != nullptr)
				{
					Vector3d p_t_pos = p_t->node.get_pos();
					// kd_insert3(start_tree, p_t_pos[0], p_t_pos[1], p_t_pos[2], p_t);
					cnt++;
					if (cnt >= 3)
					{
						break;
					}
					if (try_connect(p_t))
					{
						cout << "get path in  iteration: " << i << endl;
						cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
						return get_path();
					}
					p_t = extend(p_new_pos);
				}
			}
		}
	}

	// can't find path
	cout << "can't find path!!!" << endl;
	ROS_INFO("start tree size: %d, goal tree size: %d", start_node_pool.size(), goal_node_pool.size());
	inProcess = false;
	return result;
}

// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "kdRRT_star_node");
// 	ROS_INFO("Planner started!");
// 	kdRRT_star planner;
// 	ros::spin();
// }