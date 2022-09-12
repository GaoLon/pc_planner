#include "RRT.h"
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_filter;
bool mapReceived = false;
// kdtree.setInputCloud(PCTrajNode::PCMap);

void RRT::visualizePath(vector<RRT_Node> path)
{
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();

	geometry_msgs::Pose pose;
	tf::Matrix3x3 R;
	Matrix4d T;
	Vector3d point, point_parent;
	geometry_msgs::Point point_msg1, point_msg2;
	std::cout<<"---------------------------------------------------------------------------------------------------------path.size() "<<path.size()<<std::endl;
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
		// std::cout<<"	start tree cost "<<node.cost<<std::endl;
		// std::cout<<"						node.parent.second "<<node.parent.second<<std::endl;
	}
	path_pub.publish(msg);
}

void RRT::visualizeTrees()
{

	// visualize tree nodes
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();

	//visualize tree edges
	visualization_msgs::Marker edges;
	edges.header.frame_id = "map";
	edges.header.stamp = ros::Time::now();
	edges.ns = "RRT";
	edges.id = 0;
	edges.type = visualization_msgs::Marker::LINE_LIST;
	edges.scale.x = 0.2;
	edges.color.b = 1;
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
		// std::cout<<"	Trees  cost "<<node.cost<<std::endl;
		// std::cout<<"	Trees	node.parent.second "<<node.parent.second<<std::endl;
		if (node.parent.second != -1)
		{
			point_msg1.x = point(0);
			point_msg1.y = point(1);
			point_msg1.z = point(2);
			point_parent = start_tree[node.parent.second].node.get_T().block<3, 1>(0, 3);
			point_msg2.x = point_parent(0);
			point_msg2.y = point_parent(1);
			point_msg2.z = point_parent(2);
			edges.points.push_back(point_msg1);
			edges.points.push_back(point_msg2);
		}
	}
			// std::cout<<"----------------------------------------------"<<std::endl;
	// for (auto node : goal_tree)
	// {
	// 	T = node.node.get_T();
	// 	point = T.block<3, 1>(0, 3);
	// 	pose.position.x = point(0);
	// 	pose.position.y = point(1);
	// 	pose.position.z = point(2);
	// 	for (int i = 0; i < 3; i++)
	// 	{
	// 		for (int j = 0; j < 3; j++)
	// 		{
	// 			R[i][j] = T(i, j);
	// 		}
	// 	}
	// 	tfScalar yaw, pitch, row;
	// 	R.getEulerYPR(yaw, pitch, row);
	// 	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
	// 	msg.poses.push_back(pose);
	// 	std::cout<<"	end tree cost "<<node.cost<<std::endl;
	// 	std::cout<<"						node.parent.second "<<node.parent.second<<std::endl;
	// 	if (node.parent.second != -1)
	// 	{
	// 		point_msg1.x = point(0);
	// 		point_msg1.y = point(1);
	// 		point_msg1.z = point(2);
	// 		point_parent = goal_tree[node.parent.second].node.get_T().block<3, 1>(0, 3);
	// 		point_msg2.x = point_parent(0);
	// 		point_msg2.y = point_parent(1);
	// 		point_msg2.z = point_parent(2);
	// 		edges.points.push_back(point_msg1);
	// 		edges.points.push_back(point_msg2);
	// 	}
	// }

	tree_node_pub.publish(msg);
	tree_edge_pub.publish(edges);
}

void RRT::pclMapCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
	// pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	if (!mapReceived)
	{
		std::cout<<" rrt  pcl CB"<<std::endl;
		ROS_WARN("Callback called!");
		pcl::fromROSMsg(*laserCloudMsg, *PCTrajNode::PCMap);
		kdtree.setInputCloud(PCTrajNode::PCMap);
		int n = PCTrajNode::PCMap->points.size();
		std::cout<<"PCMap"<<n<<std::endl;
		pcl::StopWatch time;
		ros::Time t1 = ros::Time::now();
		for (int i = 0; i < n; i++)
		{
			PCTrajNode::PCMap->points[i].intensity = -1;
			double angle_Rad=PCTrajNode::calculate_rho(PCTrajNode::PCMap->points[i],PCTrajNode::PCMap,kdtree);
			pair<bool,double> temp = PCTrajNode::get_rho(PCTrajNode::PCMap->points[i],PCTrajNode::PCMap,kdtree);
			pcl::PointXYZI Point;
			if(temp.second<= PCTrajNode::PR.rho_max && angle_Rad<PCTrajNode::PR.normal_max)
			{
				Point.intensity	=-1;
				Point.x			=PCTrajNode::PCMap->points[i].x;
				Point.y			=PCTrajNode::PCMap->points[i].y;
				Point.z			=PCTrajNode::PCMap->points[i].z;
				Point.intensity	=angle_Rad;
				PCTrajNode::PCMap_fiter_rho_vis->push_back(Point);
			}	
			else{
				// std::cout<<"rho "<<temp.second<<std::endl;
			}
		}
		pre_time=(ros::Time::now() - t1).toSec() * 1000;
		n = PCTrajNode::PCMap_fiter_rho_vis->points.size();
		kdtree_filter.setInputCloud(PCTrajNode::PCMap_fiter_rho_vis);
		std::cout<<"PCMap_fiter_rho_vis"<<n<<std::endl;

		// PCTrajNode::PCMap->is_dense=false;
		// pcl::removeNaNFromPointCloud(*PCTrajNode::PCMap,,);


		sensor_msgs::PointCloud2 raw_map_ros,actual_map_ros;

		pcl::toROSMsg(*PCTrajNode::PCMap, raw_map_ros);
  		raw_map_ros.header.frame_id = "map";
		_raw_map_pub.publish(raw_map_ros);
		pcl::toROSMsg(*PCTrajNode::PCMap_fiter_rho_vis, actual_map_ros);
  		actual_map_ros.header.frame_id = "map";
		_actual_map_pub.publish(actual_map_ros);

		mapReceived = true;


		n = PCTrajNode::PCMap_fiter_rho_vis->points.size();
		std::cout<<"PCMap_Size"<<n<<std::endl;
		
		cout << " calculate_all_rho consume_FINISHED:" << time.getTime() << "ms" << std::endl;
		// pcl::io::savePCDFileASCII<pcl::PointXYZI>("/home/edward/indoor.pcd", *PCTrajNode::PCMap);

		ROS_WARN("Point cloud planner: Map received.");
		// visualizePointCloud();
		// planOnce();
	}
}

void RRT::setGoalCallback(const geometry_msgs::PoseStamped &goal_msg)
{
	std::cout<<"RRT::setGoalCallback"<<std::endl;
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
		if (!pcn.isTraversable())
		{
			ROS_WARN("goal is not traversable");
			return;
		}
		
		find_path(pseudoOdom, pcn);
		visualizeTrees();
	}
	else
	{
		ROS_WARN("No map received yet! Can't plan now.");
	}
}

void RRT::publishSamplePoint(Vector3d point)
{
	geometry_msgs::PointStamped msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.point.x = point(0);
	msg.point.y = point(1);
	msg.point.z = point(2);
	sample_pub.publish(msg);
}
void RRT::publishSamplePoint_2(Vector3d point)
{
	geometry_msgs::PointStamped msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.point.x = point(0);
	msg.point.y = point(1);
	msg.point.z = point(2);
	sample_pub_2.publish(msg);
}
int RRT::find_nearest(Vector3d p)
{
	int nearest = -1;
	double dist = std::numeric_limits<double>::infinity();
	// std::cout<<"(*direct.second).size()"<<(*direct.second).size()<<std::endl;
	//TODO: traverse to find the nearest seems too horible in terms of time consuming
	for (size_t i = 0; i < (*direct.second).size(); i++)
	{
		// std::cout<<"i "<<i<<std::endl;
		// std::cout<<"(*direct.second)[i].useful  "<<(*direct.second)[i].useful<<std::endl;
		if ((*direct.second)[i].useful)
		{
			double d = (*direct.second)[i].node.get_dist(p);
			// double d = (*direct.second)[i].node.get_dist(p)*(config.rrt_param.tau_weight-(*direct.second)[i].node.get_tau());
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
					// 	while(!trgger_for_debug)
					// {
					// 	ros::spinOnce();
					// }
					// visualize tree nodes
					geometry_msgs::PoseArray msg;
					msg.header.frame_id = "map";
					msg.header.stamp = ros::Time::now();

					//visualize tree edges
					visualization_msgs::Marker edges;
					edges.header.frame_id = "map";
					edges.header.stamp = ros::Time::now();
					edges.ns = "RRT";
					edges.id = 0;
					edges.type = visualization_msgs::Marker::LINE_LIST;
					edges.scale.x = 0.2;
					edges.color.b = 1;
					edges.color.a = 1;

					geometry_msgs::Point point_msg1, point_msg2;







	//static int debug;
	//cout << "extend: " << debug++ << endl;
	//在已有的rrt树上找到一个离采样点最近的
	int nearest = find_nearest(p);
	// cout << "------------------------------nearest: " << nearest << endl;
	if (nearest == -1) //which suggests that the tree is empty now
	{	
		// cout << "find_nearest=-1 may tree empty" << endl;
		return {true, -1}; // second is -1 means that it's invalid
	}

	Matrix4d T_new = Matrix4d::Identity();
	double dist = std::numeric_limits<double>::infinity();
	int new_index = -1;

	// find nearist extend traj
	//no use是指这个轨迹还长不长，轨迹库里有13个轨迹嘛
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
		//T_i是以nearest的node为原点展开的平面上按照轨迹库某一条去trans后得到的T_i
		//找一条距离采样点最近的
		if (d < dist)
		{
			dist = d;
			new_index = index;
			// std::cout<<"traj_lib[index]"<<traj_lib[index]<<std::endl;
			T_new = T_i;
		}
	}
// std::cout<<"------------------------------new_index "<<new_index<<std::endl;
	// update node's state
	(*direct.second)[nearest].no_use.erase(new_index);
	if ((*direct.second)[nearest].no_use.empty())
	{
		(*direct.second)[nearest].useful = false;
	}

	// check traversable and return
	PCTrajNode pc_new(T_new);
	// std::cout<<"RRT this sample isTraversable()"<<pc_new.isTraversable()<<std::endl;
	// std::cout<<"---------------------------------------"<<std::endl;
	trgger_for_debug=false;


	// // if(pc_new.isTraversable())
	// // {
	// 	Vector3d pt;
	// 	pt(0)=pc_new.T(0,3);
	// 	pt(1)=pc_new.T(1,3);
	// 	pt(2)=pc_new.T(2,3);
	// 			point_msg1.x = pt(0);
	// 			point_msg1.y = pt(1);
	// 			point_msg1.z = pt(2);
	// 			edges.points.push_back(point_msg1);
	// 	publishSamplePoint(pt);
	// 	// std::cout<<"-----pc_new.isTraversable()-"<<std::endl;
	// // }
	// 	// Vector3d pt;
	// 	pt(0)=(*direct.second)[nearest].node.T(0,3);
	// 	pt(1)=(*direct.second)[nearest].node.T(1,3);
	// 	pt(2)=(*direct.second)[nearest].node.T(2,3);
	// 			point_msg2.x = pt(0);
	// 			point_msg2.y = pt(1);
	// 			point_msg2.z = pt(2);
	// 			edges.points.push_back(point_msg2);
	// 			tree_edge_pub.publish(edges);
	if (pc_new.isTraversable())
	{
		RRT_Node p_new(pc_new, {direct.first, nearest});
		p_new.cost = (*direct.second)[nearest].cost + traj_cost[new_index];
		p_new.traj_idx = new_index;
		// (*direct.second)[nearest].node.set_sf(traj_cost[new_index]);//父节点的cost给set一下，但是其实一个父节点有好多个子节点，sf会被覆盖的 这里是有问题的
		// Vector3d pt;
		// pt(0)=(*direct.second)[nearest].node.T(0,3);
		// pt(1)=(*direct.second)[nearest].node.T(1,3);
		// pt(2)=(*direct.second)[nearest].node.T(2,3);
		// publishSamplePoint_2(pt);


				// point_msg2.x = pt(0);
				// point_msg2.y = pt(1);
				// point_msg2.z = pt(2);
				// edges.points.push_back(point_msg2);
		// std::cout<<"direct.first    start or end 					"<<direct.first<<std::endl;
		// std::cout<<"extending				    father 				"<<nearest<<std::endl;
		// std::cout<<"extending    				father   cost 		"<<(*direct.second)[nearest].cost<<std::endl;
		// std::cout<<"new_index "<<new_index<<std::endl;
		(*direct.second).push_back(p_new);





		// tree_edge_pub.publish(edges);


		return {direct.first, (*direct.second).size() - 1};
		
	}
	else
	{
		ROS_WARN("Not traversable");
		return {true, -1}; // not traversable, invalid
	}
}

void RRT::update_cost(pair<bool, int> p)
{
	for (size_t i = 0; i < start_tree.size(); i++)
	{
		// std::cout<<"	i "<<i<<std::endl;
		// std::cout<<"	 父亲  "<<start_tree[i].parent.first<<std::endl;
		// std::cout<<"	 父亲  "<<start_tree[i].parent.second<<std::endl;
		// if(!start_tree[i].parent.second==-1)
		// 	std::cout<<"	Trees  这个树 parent cost"<<start_tree[start_tree[i].parent.second].node.get_cost()<<std::endl;
		if (start_tree[i].parent == p) // 父节点是p的
		{
			double c = start_tree[p.second].node.get_cost();//可能死在start_tree[6785].node.get_cost()
			// std::cout<<"	i "<<i<<std::endl;
			// std::cout<<"	Trees  这个树 cost "<<start_tree[i].node.get_cost()<<std::endl;
			// std::cout<<"	Trees  这个树 parent "<<start_tree[i].parent.second<<std::endl; // 连父节点和这个子
			// std::cout<<"	c "<<c<<std::endl;
			if (c==0)//
			{
				c = PCTrajNode::get_connect(start_tree[p.second].node, start_tree[i].node)[4];
				if(c==INFINITY){
				// ROS_ERROR("MAYBE ERROR!");
				// std::cout<<"===========c==0  "<<"start_tree[i].parent "<<start_tree[i].parent.second<<std::endl; // 连父节点和这个子
				// std::cout<<"start_tree[i].node  i"<<i<<std::endl; // 连父节点和这个子
				// std::cout<<"start_tree[i].node  cost"<<start_tree[i].node.get_cost()<<std::endl; // 连父节点和这个子
				// std::cout<<"FROM"<<start_tree[p.second].node.T<<std::endl;
				// std::cout<<"END"<<start_tree[i].node.T<<std::endl;
				// std::cout<<"c "<<c<<std::endl; // 连父节点和这个子//无语,怎么扔进去父子节点一模一样的//
				}
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
	// cout<<"try_connect   start_tree.size()"<<start_tree.size()<<endl;
	for (size_t j = 0; j <=start_tree.size() - 1; j++)
	{
		// cout<<"		start_tree_j "<<j<<" start_tree(i).cost "<<start_tree[j].cost<<endl;
	}

	vector<RRT_Node> *tree;
	if (p.first)//每次try_connect之前会换方向,p如果是1,即现在的direct是0   这个p他的int都是其所在的tree的当时的size-1,因为每次p都是push进去都是最后一个的
	{
		tree = &start_tree;
	}
	else
	{
		tree = &goal_tree;
	}
	// cout<<"要试着连的树节点有多少个  "<<(*direct.second).size()<<endl;
	for (size_t i = 0; i < (*direct.second).size(); i++) // traverse the other tree
	{
		if ((*direct.second)[i].node.get_dist((*tree)[p.second].node) < PCTrajNode::config.rrt_param.trees_merge_thresh)
		{
			// cout<<"				距离海星 可以试着连连  这是这个树上的第几个？ "<<i<<endl;
			// get srt, save direct
			vector<PCTrajNode> path_pc;
			pair<bool, int> tp = {true, -1};
			pair<bool, int> srt_end = {true, -1};
			if (direct.first)//此时的direct是正,说明之前的p是end tree上的
			{
				// cout<<"				此时的direct是正,说明之前的p是end tree上的"<<endl;
				//可能是有点小问题
					// 					while(!trgger_for_debug)
					// {
					// 	ros::spinOnce();
					// }
					// Vector3d pt;
					// pt(0)=(*tree)[p.second].node.T(0,3);
					// pt(1)=(*tree)[p.second].node.T(1,3);
					// pt(2)=(*tree)[p.second].node.T(2,3);
					// publishSamplePoint(pt);
					// pt(0)=(*direct.second)[i].node.T(0,3);
					// pt(1)=(*direct.second)[i].node.T(1,3);
					// pt(2)=(*direct.second)[i].node.T(2,3);
					// publishSamplePoint_2(pt);
					// vector<RRT_Node> path;
					// RRT_Node vis1((*direct.second)[i].node, tp);
					// path.push_back(vis1);
					// RRT_Node vis2((*tree)[p.second].node, tp);
					// path.push_back(vis2);
					// visualizePath(path);
				path_pc = PCTrajNode::SRT_Generate((*direct.second)[i].node, (*tree)[p.second].node);
				// cout<<"				tp   whichtree index"<<direct.first <<"  "<<i<<endl;
				// cout<<"				path_pc[0]  "<<path_pc[0].T <<endl;
				
				tp = {direct.first, i};
				// cout<<"				tp  			"<<get_Node(tp).node.T <<endl;
				srt_end = p;
				// trgger_for_debug=false;
			}
			else//此时的direct是负,说明之前的p是start tree上的
			{
				// cout<<"				此时的direct是负,说明之前的p是start tree上的"<<endl;
					// 					while(!trgger_for_debug)
					// {
					// 	ros::spinOnce();
					// }
					// Vector3d pt;
					// pt(0)=(*tree)[p.second].node.T(0,3);
					// pt(1)=(*tree)[p.second].node.T(1,3);
					// pt(2)=(*tree)[p.second].node.T(2,3);
					// publishSamplePoint(pt);
					// pt(0)=(*direct.second)[i].node.T(0,3);
					// pt(1)=(*direct.second)[i].node.T(1,3);
					// pt(2)=(*direct.second)[i].node.T(2,3);
					// publishSamplePoint_2(pt);

					// vector<RRT_Node> path;
					// RRT_Node vis1((*direct.second)[i].node, tp);
					// path.push_back(vis1);
					// RRT_Node vis2((*tree)[p.second].node, tp);
					// path.push_back(vis2);
					// visualizePath(path);
				path_pc = PCTrajNode::SRT_Generate((*tree)[p.second].node, (*direct.second)[i].node);
				// cout<<"				tp   whichtree index"<<direct.first <<"  "<<i<<endl;
				// cout<<"				path_pc[0]  "<<path_pc[0].T <<endl;
				tp = p;
				// cout<<"				tp  			"<<get_Node(tp).node.T <<endl;
				srt_end = {direct.first, i};
					// trgger_for_debug=false;
			}

			if (path_pc.empty())
			{
				// cout<<"		path_pc.empty()"<<endl;
				continue;
			}

			if (path_pc.size()<=1)
			{
				// cout<<"		path_pc.empty()"<<endl;
				continue;
			}
			else
			{
// cout<<"		path_pc[0] get_T"<<endl;
// cout<<path_pc[0].get_T()<<endl;
// cout<<"		path_pc[1] get_T"<<endl;
// cout<<path_pc[1].get_T()<<endl;
				// add nodes to start_tree
				cout<<"add SRT nodes(中间的，不带始末) to start tree!"<<endl;
cout<<"		SRT出来的 size（包含SRT的fromnode和tonode）="<<path_pc.size()<<endl;
				for (size_t j = 1; j < path_pc.size() - 1; j++)
				{
					RRT_Node temp(path_pc[j], tp);
// cout<<"		path_pc[j] get_T"<<endl;
// cout<<path_pc[j].get_T()<<endl;
// cout<<"		get_Node(tp).cost  "<<get_Node(tp).cost<<endl;
// cout<<"		path_pc[j - 1].get_cost()   "<<path_pc[j - 1].get_cost()<<endl;

					temp.cost = get_Node(tp).cost + path_pc[j - 1].get_cost();
				// cout<<"		temp.cost    "<<temp.cost<<endl;
					start_tree.push_back(temp);
					tp = {true, start_tree.size() - 1};
				}

				// merge goal tree
				// cout<<"merge goal tree!"<<endl;
				RRT_Node &srt_end_node = get_Node(srt_end);
				pair<bool, int> pg = srt_end_node.parent;
				srt_end_node.parent = tp;
				srt_end_node.cost = get_Node(tp).cost + path_pc[path_pc.size() - 2].get_cost();
				cout<<"		srt_end_node.cost    "<<srt_end_node.cost<<endl;
				start_tree.push_back(srt_end_node);
				pair<bool, int> pgb = {true, start_tree.size() - 1};
				double c = traj_cost[srt_end_node.traj_idx];
				double pgb_c = srt_end_node.cost;
				// cout<<"		srt_end_node.node.T   "<<srt_end_node.node.T<<endl;
				// double debug=0;
				while (pg.second != -1)
				{
					// cout<<debug++<<endl;
					RRT_Node &temp = get_Node(pg);
					pair<bool, int> pga = temp.parent;
					temp.parent = pgb;
					temp.cost = pgb_c + c;
					// cout<<"	剩下那段	temp.cost    "<<temp.cost<<endl;
					// cout<<"		temp.T   "<<temp.node.T<<endl;
					start_tree.push_back(temp);
					pgb_c = temp.cost;
					c = traj_cost[temp.traj_idx];
					pgb = {true, start_tree.size() - 1};
					pg = pga;
				}
				//update_cost(srt_end);
				// cout<<"update cost "<<endl;

				// for (size_t i = 0; i < start_tree.size(); i++)
				// {
				// 	std::cout<<"	before update cost start tree cost "<<start_tree[i].cost<<std::endl;
				// 	std::cout<<"						node.parent.second "<<start_tree[i].parent.second<<std::endl;
				// 	std::cout<<"	start_tree[i].node.T "<<start_tree[i].node.T<<std::endl;
				// }
				update_cost({true, 0});//这个操作有问题 挂
				ROS_ERROR("maybe die there333");
				// cout<<"after  try_connect   start_tree.size()"<<start_tree.size()<<endl;
				return true;
			}
		}
		else{
			// cout<<"				too long"<<endl;
		}
	}
	cout<<"can't connect!"<<endl;
	return false;
}

vector<RRT_Node> RRT::get_path()
{
	vector<RRT_Node> result;
	pair<bool, int> g = {true, goal_idx};
	cout << "==============================get_path=============================="<< endl;
	cout << "RRT  trajectory cost=" << get_Node(g).cost << endl;
	cout << "g.second" << g.second << endl;
	while (g.second != -1)
	{	
		RRT_Node &p = get_Node(g);
		if(!p.node.isTraversable())
			cout<<"is Traversable   		"<<p.node.isTraversable()<<p.node.get_pos()[0] << " " << p.node.get_pos()[1] << " " << p.node.get_pos()[2]<<endl;
		if(g.second != 0){
		PCTrajNode to_node = p.node;
		PCTrajNode  from_node= get_Node(p.parent).node;
		// cout<<"get connect in get path"<<endl;
		VectorXd par = PCTrajNode::get_connect(from_node, to_node);
		if(par[4]>10e7){
			ROS_ERROR("get connect error");
			std::cout<<"	from_node "<<from_node.get_pos()[0]<<from_node.get_pos()[1]<<from_node.get_pos()[2]<<std::endl;
			std::cout<<"	to_node "<<to_node.get_pos()[0]<< " " <<to_node.get_pos()[1]<< " " <<to_node.get_pos()[2]<<std::endl;
			}
		}

		// std::cout<<"	get_path_from_goal cost "<<p.cost<<std::endl;
		// std::cout<<"						node.parent.second "<<p.parent.second<<std::endl;
		// cout << p.node.get_pos()[0] << " " << p.node.get_pos()[1] << " " << p.node.get_pos()[2] << endl;
		result.push_back(p);
		g = p.parent;
	}
	inProcess = false;

	visualizePath(result);
	ROS_WARN("RRT path nodes number: %d", result.size());
	cout << "==============================get_path finished======================"<< endl;
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
	RRT_Node g(goal, {false, -1});
	s.cost = 0;
	g.cost = 0;

	start_tree.push_back(s);
	goal_tree.push_back(g);
		Vector3d pt;
		pt(0)=g.node.T(0,3);
		pt(1)=g.node.T(1,3);
		pt(2)=g.node.T(2,3);
		// publishSamplePoint_2(pt);


	// begin iteration
	cout << "RRT begin!" << endl;
	pcl::StopWatch time;
	std::cout<<"PCTrajNode::config.rrt_param.max_iter"<<PCTrajNode::config.rrt_param.max_iter<<std::endl;
	for (int i = 0; i < PCTrajNode::config.rrt_param.max_iter; i++)
	{
		// std::cout<<"iter "<<i<<" trgger_for_debug "<<trgger_for_debug<<std::endl;



		// sample, extend this tree
		// cout << "iteration: " << i << endl;


		// if (i % 100 == 0)
		// {
			// ROS_WARN("Iteration: %d", i);

			// visualizeTrees();

		// }

		// std::cout<<"在pcmap的kdtree上rand"<<std::endl;
		Vector3d p = sample();
		// publishSamplePoint_2(p);
		//visualize
		// publishSamplePoint_2(p);
		pair<bool, int> p_new = extend(p);//extend return的是方向direct.first和 (*direct.second).size() - 1
		// std::cout<<"p_new.second"<<p_new.second<<std::endl;



					


		if (p_new.second != -1) // if it's valid
		{
			// change, and try connect with that tree
			
			change_direction();		//
			// std::cout<<"换方向,拿对面每个点跟p_new连"<<std::endl;
			if (try_connect(p_new)) // path found
			{   cout << "try connect p_new yes "<< endl;
				cout << "get path in iteration: " << i << endl;
				cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
				goal_idx = start_tree.size() - 1;
				return get_path();
			}
			
			// can't connect, from that tree extend to p_new, try connect continuely
			else
			{
				// std::cout<<"p_new连不动"<<std::endl;
				Vector3d p_new_pos = get_Node(p_new).node.get_pos();
				// publishSamplePoint_2(p_new_pos);
				pair<bool, int> p_t = extend(p_new_pos);
				int cnt = 0;
				//本来论文是要尝试连接两个树上所有能相互连（<3*rep）的点
				//徐隆写的是一次try_connect就是拿一个点去和另一个树上所有能try的点have try
				//然后去尝试这个p_new当sample点然后去搞出新的p_new_new即p_t,如果pt没因为不可通行被ban，就去try_connect(p_t)
				//p_t是会变得，虽然p_new固定，找出来的near也固定，但是轨迹库有13条，其中某条的p_t也许会被used，used之后就不再extend的时候选了
				while (p_t.second != -1)
				{
					cnt++;
					if (cnt >= PCTrajNode::config.rrt_param.heuristic_straight_thresh)
					{
						cout << "cnt >= PCTrajNode::config.rrt_param.heuristic_straight_thresh break"<< endl;
						break;
					}
					//我觉得可能找出来的near可能也不固定？因此之前会把extend加进去？但是按照论文这里是不extend的   算了这里先不管了
					change_direction();
					if (try_connect(p_t))
					{
						cout << "try connect from other yes "<< endl;
						cout << "get path in  iteration: " << i << endl;
						cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
						goal_idx = start_tree.size() - 1;
						return get_path();
					}
					change_direction();
					p_t = extend(p_new_pos);



					//
				}
			}
		}

	}

	// can't find path
	cout << "can't find path!!!" << endl;
	ROS_WARN("start tree size: %d, goal tree size: %d", start_tree.size(), goal_tree.size());
	visualizeTrees();
	inProcess = false;
	return result;
}

// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "RRT_node");
// 	ROS_WARN("Planner started!");
// 	RRT planner;
// 	ros::spin();
// }