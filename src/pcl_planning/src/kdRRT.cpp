#include "kdRRT.h"

bool kdmapReceived;
extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

void kdRRT::visualizePath(vector<kdRRT_Node> path)
{
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "/map";
	msg.header.stamp = ros::Time::now();

	geometry_msgs::Pose pose;
	tf::Matrix3x3 R;
	Matrix4d T;
	Vector3d point, point_parent;
	geometry_msgs::Point point_msg1, point_msg2;

	for (auto node : path)
	{
		T = node.node.get_T();
		point = T.block<3, 1>(0, 3);
		pose.position.x = point(0);
		pose.position.y = point(1);
		pose.position.z = point(2);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				R[i][j] = T(i, j);
			}
		}
		tfScalar yaw, pitch, row;
		R.getEulerYPR(yaw, pitch, row);
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
		msg.poses.push_back(pose);
	}
	path_pub.publish(msg);
}

void kdRRT::visualizeTrees()
{

	// visualize tree nodes
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "/map";
	msg.header.stamp = ros::Time::now();

	//visualize tree edges
	visualization_msgs::Marker edges;
	edges.header.frame_id = "/map";
	edges.header.stamp = ros::Time::now();
	edges.ns = "RRT";
	edges.id = 0;
	edges.type = visualization_msgs::Marker::LINE_LIST;
	edges.scale.x = 0.2;
	edges.color.a = 1;

	geometry_msgs::Pose pose;
	tf::Matrix3x3 R;
	Matrix4d T;
	Vector3d point, point_parent;
	geometry_msgs::Point point_msg1, point_msg2;

	for (auto node : start_node_pool)
	{
		T = node->node.get_T();
		point = T.block<3, 1>(0, 3);
		pose.position.x = point(0);
		pose.position.y = point(1);
		pose.position.z = point(2);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				R[i][j] = T(i, j);
			}
		}
		tfScalar yaw, pitch, row;
		R.getEulerYPR(yaw, pitch, row);
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
		msg.poses.push_back(pose);

		if (node->parent != nullptr)
		{
			point_msg1.x = point(0);
			point_msg1.y = point(1);
			point_msg1.z = point(2);
			point_parent = node->parent->node.get_T().block<3, 1>(0, 3);
			point_msg2.x = point_parent(0);
			point_msg2.y = point_parent(1);
			point_msg2.y = point_parent(2);
			edges.points.push_back(point_msg1);
			edges.points.push_back(point_msg2);
		}
	}
	for (auto node : goal_node_pool)
	{
		T = node->node.get_T();
		point = T.block<3, 1>(0, 3);
		pose.position.x = point(0);
		pose.position.y = point(1);
		pose.position.z = point(2);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				R[i][j] = T(i, j);
			}
		}
		tfScalar yaw, pitch, row;
		R.getEulerYPR(yaw, pitch, row);
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
		msg.poses.push_back(pose);

		if (node->parent != nullptr)
		{
			point_msg1.x = point(0);
			point_msg1.y = point(1);
			point_msg1.z = point(2);
			point_parent = node->parent->node.get_T().block<3, 1>(0, 3);
			point_msg2.x = point_parent(0);
			point_msg2.y = point_parent(1);
			point_msg2.y = point_parent(2);
			edges.points.push_back(point_msg1);
			edges.points.push_back(point_msg2);
		}
	}

	tree_node_pub.publish(msg);
	tree_edge_pub.publish(edges);
}

void kdRRT::pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
	// pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	if (!kdmapReceived)
	{
		ROS_INFO("Callback called!");
		pcl::fromROSMsg(*laserCloudMsg, *PCTrajNode::PCMap);
		kdtree.setInputCloud(PCTrajNode::PCMap);

		int n = PCTrajNode::PCMap->points.size();
		for (int i = 0; i < n; i++)
		{
			PCTrajNode::PCMap->points[i].intensity = -1;
		}
		kdmapReceived = true;

		// pcl::io::savePCDFileASCII<pcl::PointXYZI>("/home/edward/indoor.pcd", *PCTrajNode::PCMap);

		ROS_INFO("Point cloud planner: Map received.");
		// visualizePointCloud();
		// planOnce();
	}
}

// void kdRRT::setGoalCallback(const geometry_msgs::PoseStamped &goal_msg)
// {
// 	if (kdmapReceived)
// 	{
// 		tf::Quaternion q;
// 		tf::quaternionMsgToTF(goal_msg.pose.orientation, q);
// 		tf::Matrix3x3 R(q);

// 		Matrix4d goal_mat = Matrix4d::Identity();
// 		for (int i = 0; i < 3; i++)
// 		{
// 			for (int j = 0; j < 3; j++)
// 			{
// 				goal_mat(i, j) = R[i][j];
// 			}
// 		}
// 		goal_mat(0, 3) = goal_msg.pose.position.x;
// 		goal_mat(1, 3) = goal_msg.pose.position.y;
// 		goal_mat(2, 3) = goal_msg.pose.position.z;
// 		//TODO
// 		PCTrajNode pcn(goal_mat), pseudoOdom(Matrix4d::Identity());
// 		find_path(pseudoOdom, pcn);
// 		visualizeTrees();
// 	}
// 	else
// 	{
// 		ROS_INFO("No map received yet! Can't plan now.");
// 	}
// }

void kdRRT::update_cost(kdRRT_NodePtr p)
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
			update_cost(start_node_pool[i]);
		}
	}
}

kdRRT_NodePtr kdRRT::sensible_extend(void)
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
		(*pool).push_back(p_new);
		return p_new;
	}
	else
	{
		return nullptr;
	}

}

kdRRT_NodePtr kdRRT::extend(Vector3d p)
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
		(*pool).push_back(p_new);
		return p_new;
	}
	else
	{
		return nullptr;
	}
}

bool kdRRT::try_connect(kdRRT_NodePtr p)
{
	//cout << "try connect" << endl;
	struct kdtree* true_start_tree;

	Vector3d p_pos = p->node.get_pos();
	struct kdres *p_nbr_set = kd_nearest_range3(goal_tree, p_pos[0], p_pos[1], p_pos[2], 3 * r_exp);
	if (p_nbr_set == nullptr)
	{
		//cout << "kd range query error!" << endl;
		return false;
	}
	while (!kd_res_end(p_nbr_set))
	{
		//cout << "try connect truely" << endl;
		kdRRT_NodePtr to_node = (kdRRT_NodePtr)kd_res_item_data(p_nbr_set);
		vector<PCTrajNode> path_pc;
		kdRRT_NodePtr tp = nullptr;
		kdRRT_NodePtr srt_end = nullptr;
		if (direction)
		{
			path_pc = PCTrajNode::SRT_Generate(p->node, to_node->node);
			tp = p;
			srt_end = to_node;
			true_start_tree = start_tree;
		}
		else
		{
			path_pc = PCTrajNode::SRT_Generate(to_node->node, p->node);
			tp = to_node;
			srt_end = p;
			true_start_tree = goal_tree;
		}
		if (path_pc.empty())
		{
			kd_res_next(p_nbr_set);
			continue;
		}
		else
		{
			// add nodes to start_tree
			for (size_t j = 1; j < path_pc.size() - 1; j++)
			{
				kdRRT_NodePtr temp = new kdRRT_Node(path_pc[j], tp);
				temp->cost = tp->cost + path_pc[j - 1].get_cost();
				start_node_pool.push_back(temp);
				Vector3d temp_pos = temp->node.get_pos();
				kd_insert3(true_start_tree, temp_pos[0], temp_pos[1], temp_pos[2], temp);
				tp = temp;
			}

			// merge goal tree
			kdRRT_NodePtr pg = srt_end->parent;
			srt_end->parent = tp;
			srt_end->cost = tp->cost + path_pc[path_pc.size() - 2].get_cost();
			kdRRT_NodePtr srt_end_mirror = new kdRRT_Node(*srt_end);
			start_node_pool.push_back(srt_end_mirror);
			Vector3d srt_end_pos = srt_end->node.get_pos();
			kd_insert3(true_start_tree, srt_end_pos[0], srt_end_pos[1], srt_end_pos[2], srt_end_mirror);

			kdRRT_NodePtr pgb = srt_end_mirror;
			double c = traj_cost[srt_end->traj_idx];
			double pgb_c = srt_end->cost;
			while (pg!=nullptr)
			{
				kdRRT_NodePtr pga = pg->parent;
				pg->parent = pgb;
				pg->cost = pgb_c + c;
				kdRRT_NodePtr pg_mirror = new kdRRT_Node(*pg);
				start_node_pool.push_back(pg_mirror);
				Vector3d pg_pos = pg->node.get_pos();
				kd_insert3(true_start_tree, pg_pos[0], pg_pos[1], pg_pos[2], pg_mirror);
				pgb_c = pg->cost;
				c = traj_cost[pg->traj_idx];
				pgb = pg_mirror;
				pg = pga;
			}
			//update_cost(srt_end);
			update_cost(start_node);
			kd_res_free(p_nbr_set);
			return true;
		}
	}
	kd_res_free(p_nbr_set);
	return false;
}

vector<kdRRT_Node> kdRRT::get_path()
{
	vector<kdRRT_Node> result;
	kdRRT_NodePtr g = goal_node;
	cout << "trajectory cost=" << g->cost << endl;
	while (g != nullptr)
	{
		kdRRT_Node& p = *g;
		//cout << p.cost << endl;
		// cout << p.node.get_pos()[0] << " " << p.node.get_pos()[1] << " " << p.node.get_pos()[2] << endl;
		result.push_back(p);
		g = p.parent;
	}
	inProcess = false;
	visualizePath(result);
	ROS_INFO("Path nodes number: %d", result.size());
	return result;
}

vector<kdRRT_Node> kdRRT::find_path(const PCTrajNode& start, const PCTrajNode& goal)
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
	cout << "kdRRT begin!" << endl;
	pcl::StopWatch time;
	for (int i = 0; i < max_iter; i++)
	{
		// sample, extend this tree
		// cout << "iteration: " << i << endl;
		if (i % 100 == 0)
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
			kd_insert3(start_tree, p_new_pos[0], p_new_pos[1], p_new_pos[2], p_new);
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
					kd_insert3(start_tree, p_t_pos[0], p_t_pos[1], p_t_pos[2], p_t);
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
// 	ros::init(argc, argv, "kdRRT_node");
// 	ROS_INFO("Planner started!");
// 	kdRRT planner;
// 	ros::spin();
// }