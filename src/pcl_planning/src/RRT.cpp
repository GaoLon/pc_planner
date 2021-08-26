#include "RRT.h"

pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
bool mapReceived = false;
// kdtree.setInputCloud(PCTrajNode::PCMap);

void RRT::visualizePath(vector<RRT_Node> path)
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

void RRT::visualizeTrees()
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

	for (auto node : start_tree)
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

		if (node.parent.second != -1)
		{
			point_msg1.x = point(0);
			point_msg1.y = point(1);
			point_msg1.z = point(2);
			point_parent = start_tree[node.parent.second].node.get_T().block<3, 1>(0, 3);
			point_msg2.x = point_parent(0);
			point_msg2.y = point_parent(1);
			point_msg2.y = point_parent(2);
			edges.points.push_back(point_msg1);
			edges.points.push_back(point_msg2);
		}
	}
	for (auto node : goal_tree)
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

		if (node.parent.second != -1)
		{
			point_msg1.x = point(0);
			point_msg1.y = point(1);
			point_msg1.z = point(2);
			point_parent = goal_tree[node.parent.second].node.get_T().block<3, 1>(0, 3);
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

void RRT::pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
	// pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	if (!mapReceived)
	{
		ROS_INFO("Callback called!");
		pcl::fromROSMsg(*laserCloudMsg, *PCTrajNode::PCMap);
		kdtree.setInputCloud(PCTrajNode::PCMap);

		int n = PCTrajNode::PCMap->points.size();
		for (int i = 0; i < n; i++)
		{
			PCTrajNode::PCMap->points[i].intensity = -1;
		}
		mapReceived = true;

		// pcl::io::savePCDFileASCII<pcl::PointXYZI>("/home/edward/indoor.pcd", *PCTrajNode::PCMap);

		ROS_INFO("Point cloud planner: Map received.");
		// visualizePointCloud();
		// planOnce();
	}
}

void RRT::setGoalCallback(const geometry_msgs::PoseStamped &goal_msg)
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
		find_path(pseudoOdom, pcn);
		visualizeTrees();
	}
	else
	{
		ROS_INFO("No map received yet! Can't plan now.");
	}
}

void RRT::publishSamplePoint(Vector3d point)
{
	geometry_msgs::PointStamped msg;
	msg.header.frame_id = "/map";
	msg.header.stamp = ros::Time::now();
	msg.point.x = point(0);
	msg.point.y = point(1);
	msg.point.z = point(2);
	sample_pub.publish(msg);
}

int RRT::find_nearest(Vector3d p)
{
	int nearest = -1;
	double dist = std::numeric_limits<double>::infinity();

	//TODO: traverse to find the nearest seems too horible in terms of time consuming
	for (size_t i = 0; i < (*direct.second).size(); i++)
	{
		if ((*direct.second)[i].useful)
		{
			double d = (*direct.second)[i].node.get_dist(p);
			// double d = (*direct.second)[i].node.get_dist(p)*(tau_weight-(*direct.second)[i].node.get_tau());
			if (d < dist)
			{
				dist = d;
				nearest = i;
			}
		}
	}

	return nearest;
}

pair<bool, int> RRT::extend(Vector3d p)
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

	// check traversable and return
	PCTrajNode pc_new(T_new);
	if (pc_new.isTraversable())
	{
		RRT_Node p_new(pc_new, {direct.first, nearest});
		p_new.cost = (*direct.second)[nearest].cost + traj_cost[new_index];
		p_new.traj_idx = new_index;
		(*direct.second).push_back(p_new);
		return {direct.first, (*direct.second).size() - 1};
	}
	else
	{
		// ROS_INFO("Not traversable");
		return {true, -1}; // not traversable, invalid
	}
}

void RRT::update_cost(pair<bool, int> p)
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
			update_cost({true, i});
		}
	}
	/*if (p.first)
	{
		for (size_t i = 0; i < start_tree.size(); i++)
		{
			if (start_tree[i].parent == p)
			{
				start_tree[i].cost = start_tree[p.second].cost + traj_cost[start_tree[p.second].traj_idx];
				update_cost({true,i});
			}
		}
		for (size_t i = 0; i < goal_tree.size(); i++)
		{
			if (goal_tree[i].parent == p)
			{
				goal_tree[i].cost = start_tree[p.second].cost + traj_cost[start_tree[p.second].traj_idx];
				update_cost({ false,i });
			}
		}
	}
	else
	{
		for (size_t i = 0; i < start_tree.size(); i++)
		{
			if (start_tree[i].parent == p)
			{
				start_tree[i].cost = goal_tree[p.second].cost + traj_cost[goal_tree[p.second].traj_idx];
				update_cost({ true,i });
			}
		}
		for (size_t i = 0; i < goal_tree.size(); i++)
		{
			if (goal_tree[i].parent == p)
			{
				goal_tree[i].cost = goal_tree[p.second].cost + traj_cost[goal_tree[p.second].traj_idx];
				update_cost({ false,i });
			}
		}
	}*/
}

bool RRT::try_connect(pair<bool, int> p)
{
	// cout<<"try connect!"<<endl;
	vector<RRT_Node> *tree;
	if (p.first)
	{
		tree = &start_tree;
	}
	else
	{
		tree = &goal_tree;
	}
	for (size_t i = 0; i < (*direct.second).size(); i++) // traverse the other tree
	{
		if ((*direct.second)[i].node.get_dist((*tree)[p.second].node) < trees_merge_thresh)
		{
			// get srt, save direct
			vector<PCTrajNode> path_pc;
			pair<bool, int> tp = {true, -1};
			pair<bool, int> srt_end = {true, -1};
			if (direct.first)
			{
				path_pc = PCTrajNode::SRT_Generate((*direct.second)[i].node, (*tree)[p.second].node);
				tp = {direct.first, i};
				srt_end = p;
			}
			else
			{
				path_pc = PCTrajNode::SRT_Generate((*tree)[p.second].node, (*direct.second)[i].node);
				tp = p;
				srt_end = {direct.first, i};
			}

			if (path_pc.empty())
			{
				continue;
			}
			else
			{
				// add nodes to start_tree
				// cout<<"add nodes to start tree!"<<endl;
				// cout<<"path pc size="<<path_pc.size()<<endl;
				for (size_t j = 1; j < path_pc.size() - 1; j++)
				{
					RRT_Node temp(path_pc[j], tp);
					temp.cost = get_Node(tp).cost + path_pc[j - 1].get_cost();
					start_tree.push_back(temp);
					tp = {true, start_tree.size() - 1};
				}

				// merge goal tree
				// cout<<"merge goal tree!"<<endl;
				RRT_Node &srt_end_node = get_Node(srt_end);
				pair<bool, int> pg = srt_end_node.parent;
				srt_end_node.parent = tp;
				srt_end_node.cost = get_Node(tp).cost + path_pc[path_pc.size() - 2].get_cost();
				start_tree.push_back(srt_end_node);
				pair<bool, int> pgb = {true, start_tree.size() - 1};
				double c = traj_cost[srt_end_node.traj_idx];
				double pgb_c = srt_end_node.cost;

				// double debug=0;
				while (pg.second != -1)
				{
					// cout<<debug++<<endl;
					RRT_Node &temp = get_Node(pg);
					pair<bool, int> pga = temp.parent;
					temp.parent = pgb;
					temp.cost = pgb_c + c;
					start_tree.push_back(temp);
					pgb_c = temp.cost;
					c = traj_cost[temp.traj_idx];
					pgb = {true, start_tree.size() - 1};
					pg = pga;
				}
				//update_cost(srt_end);
				// cout<<"update cost "<<endl;
				update_cost({true, 0});
				return true;
			}
		}
	}
	// cout<<"can't connect!"<<endl;
	return false;
}

vector<RRT_Node> RRT::get_path()
{
	vector<RRT_Node> result;
	pair<bool, int> g = {true, goal_idx};
	cout << "trajectory cost=" << get_Node(g).cost << endl;
	while (g.second != -1)
	{
		RRT_Node &p = get_Node(g);
		// cout << p.node.get_pos()[0] << " " << p.node.get_pos()[1] << " " << p.node.get_pos()[2] << endl;
		result.push_back(p);
		g = p.parent;
	}
	inProcess = false;

	visualizePath(result);
	ROS_INFO("Path nodes number: %d", result.size());
	return result;
}

vector<RRT_Node> RRT::find_path(const PCTrajNode &start, const PCTrajNode &goal)
{
	inProcess = true;

	// initialization
	vector<RRT_Node> result;
	if(clear_start_tree)
	{
		start_tree.clear();
	}
	// 
	goal_tree.clear();
	RRT_Node s(start, {true, -1});
	//Vector3d s_pos = start.get_pos();
	//Vector2d s2g_p = goal.projection(s_pos);
	//g2s = { 0,0, atan2(s2g_p[1], s2g_p[0]) };
	//s2g = -g2s;
	//Matrix4d cg = goal.Trans(g2s);
	//PCTrajNode pcg(cg);
	RRT_Node g(goal, {false, -1});
	s.cost = 0;
	g.cost = 0;
	start_tree.push_back(s);
	goal_tree.push_back(g);

	// begin iteration
	cout << "RRT begin!" << endl;
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
		Vector3d p = sample();
		//visualize
		publishSamplePoint(p);
		pair<bool, int> p_new = extend(p);

		if (p_new.second != -1) // if it's valid
		{
			// change, and try connect with that tree
			change_direction();		//
			if (try_connect(p_new)) // path found
			{
				cout << "get path in iteration: " << i << endl;
				cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
				goal_idx = start_tree.size() - 1;
				return get_path();
			}
			// can't connect, from that tree extend to p_new, try connect continuely
			else
			{
				Vector3d p_new_pos = get_Node(p_new).node.get_pos();
				pair<bool, int> p_t = extend(p_new_pos);
				int cnt = 0;
				while (p_t.second != -1)
				{
					cnt++;
					if (cnt >= heuristic_straight_thresh)
					{
						break;
					}
					change_direction();
					if (try_connect(p_t))
					{
						cout << "get path in  iteration: " << i << endl;
						cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
						goal_idx = start_tree.size() - 1;
						return get_path();
					}
					change_direction();
					p_t = extend(p_new_pos);
				}
			}
		}
	}

	// can't find path
	cout << "can't find path!!!" << endl;
	ROS_INFO("start tree size: %d, goal tree size: %d", start_tree.size(), goal_tree.size());
	inProcess = false;
	return result;
}

// vector<RRT_Node> RRT::find_path()
// {
// 	find_path(start, goal);
// }

// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "RRT_node");
// 	ROS_INFO("Planner started!");
// 	RRT planner;
// 	ros::spin();
// }

//void RRT::visual_rrt(void)
//{
//	for (auto& p : *direct.second)
//	{
//		pcl::PointXYZRGB pc;
//		pc = p.node.Chan2PC();
//		PCTrajNode::PCMap->push_back(pc);
//	}
//
//	change_direct();
//	for (auto& p : *direct.second)
//	{
//		pcl::PointXYZRGB pc;
//		pc = p.node.Chan2PC();
//		PCTrajNode::PCMap->push_back(pc);
//	}
//}