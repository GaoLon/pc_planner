#include "RRT_star.h"

extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_filter;
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
		// visualizeTrees();
	}
	else
	{
		ROS_WARN("No map received yet! Can't plan now.");
	}
}
//这个大概就是在已有的path上随便rand出一个点，然后按照d的范围搜近邻的点云，然后在近邻里面rand出一个
//有可能找出来是空的然后挂掉
//有可能rand出的这个点云就是树上的一个点，到时候找最近邻会出现值一样的问题 然后dist=0，一除nan挂了
Vector3d RRT_star::op_sample(float d)
{
	// sample vertex on path
	ros::Time sample_time=ros::Time::now();
	int index = rand() % path.size();

	// opsample_index++;
	// if(opsample_index> path.size()-1)
	// 	opsample_index=0;
	// int index =opsample_index;

	RRT_Node sp = path[index];

	// sample point on map
	/// find near point clouds
	
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	Vector3d pos = sp.node.get_pos();
	pcl::PointXYZI searchPoint;
	searchPoint.x = pos[0];
	searchPoint.y = pos[1];
	searchPoint.z = pos[2];
	// kdtree.radiusSearch(searchPoint, d, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	// /// sample
	// int pindex = rand() % pointIdxRadiusSearch.size();
	// Vector3d p(PCTrajNode::PCMap->points[pointIdxRadiusSearch[pindex]].x,PCTrajNode::PCMap->points[pointIdxRadiusSearch[pindex]].y,PCTrajNode::PCMap->points[pointIdxRadiusSearch[pindex]].z);
	
	kdtree_filter.radiusSearch(searchPoint, d, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	if(pointIdxRadiusSearch.size()==0)
	{
		cout<<"(pointIdxRadiusSearch.size()==0 "<<endl;
		kdtree_filter.radiusSearch(searchPoint, 10 , pointIdxRadiusSearch, pointRadiusSquaredDistance);
	}
	/// sample
	int pindex = rand() % pointIdxRadiusSearch.size();

	// std::vector<int> pointIdxRadiusSearch_must_z_near;
	// for(int j=0;j<pointIdxRadiusSearch.size();j++){
	// 	double z_value= PCTrajNode::PCMap_fiter_rho_vis->points[pointIdxRadiusSearch[j]].z;
	// 	if(abs(z_value-searchPoint.z)<0.8)
	// 		pointIdxRadiusSearch_must_z_near.push_back(j);
	// }
	
	// int pindex;
	// if(pointIdxRadiusSearch_must_z_near.size()>0)
	// {
	// 	int tmp = rand() % pointIdxRadiusSearch_must_z_near.size();
	// 	pindex=pointIdxRadiusSearch_must_z_near[tmp];
	// }
	// else{
	// 	pindex = rand() % pointIdxRadiusSearch.size();
	// }
	Vector3d p(PCTrajNode::PCMap_fiter_rho_vis->points[pointIdxRadiusSearch[pindex]].x,PCTrajNode::PCMap_fiter_rho_vis->points[pointIdxRadiusSearch[pindex]].y,PCTrajNode::PCMap_fiter_rho_vis->points[pointIdxRadiusSearch[pindex]].z);

				double tmp_sample=(ros::Time::now() - sample_time).toSec() * 1000;
				sample_rrt_time_all+=tmp_sample;
	return p;
}
Vector3d RRT_star::op_sample_circle(float d){
	// sample vertex on path
	ros::Time sample_time=ros::Time::now();
	int index = rand() % path_circle.size();

	// opsample_index++;
	// if(opsample_index> path.size()-1)
	// 	opsample_index=0;
	// int index =opsample_index;

	RRT_Node sp = path_circle[index];

	// sample point on map
	/// find near point clouds
	
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	Vector3d pos = sp.node.get_pos();
	pcl::PointXYZI searchPoint;
	searchPoint.x = pos[0];
	searchPoint.y = pos[1];
	searchPoint.z = pos[2];
	// kdtree.radiusSearch(searchPoint, d, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	// /// sample
	// int pindex = rand() % pointIdxRadiusSearch.size();
	// Vector3d p(PCTrajNode::PCMap->points[pointIdxRadiusSearch[pindex]].x,PCTrajNode::PCMap->points[pointIdxRadiusSearch[pindex]].y,PCTrajNode::PCMap->points[pointIdxRadiusSearch[pindex]].z);
	
	kdtree_filter.radiusSearch(searchPoint, d , pointIdxRadiusSearch, pointRadiusSquaredDistance);
	if(pointIdxRadiusSearch.size()==0)
	{
		cout<<"(pointIdxRadiusSearch.size()==0 circle"<<endl;
		kdtree_filter.radiusSearch(searchPoint, 10 , pointIdxRadiusSearch, pointRadiusSquaredDistance);
	}
		
	/// sample
	std::vector<int> pointIdxRadiusSearch_must_z_near;
	for(int j=0;j<pointIdxRadiusSearch.size();j++){
		double z_value= PCTrajNode::PCMap_fiter_rho_vis->points[pointIdxRadiusSearch[j]].z;
		if(abs(z_value-searchPoint.z)<0.5)
			pointIdxRadiusSearch_must_z_near.push_back(j);
	}
	// int pindex = rand() % pointIdxRadiusSearch.size();
	int pindex;
	if(pointIdxRadiusSearch_must_z_near.size()>0)
	{
		int tmp = rand() % pointIdxRadiusSearch_must_z_near.size();
		pindex=pointIdxRadiusSearch_must_z_near[tmp];
	}
	else{
		pindex = rand() % pointIdxRadiusSearch.size();
	}
	Vector3d p(PCTrajNode::PCMap_fiter_rho_vis->points[pointIdxRadiusSearch[pindex]].x,PCTrajNode::PCMap_fiter_rho_vis->points[pointIdxRadiusSearch[pindex]].y,PCTrajNode::PCMap_fiter_rho_vis->points[pointIdxRadiusSearch[pindex]].z);

				double tmp_sample=(ros::Time::now() - sample_time).toSec() * 1000;
				sample_rrt_time_all+=tmp_sample;
	return p;
}
vector<pair<bool, int>> RRT_star::get_near(RRT_Node p)
{
	vector<pair<bool,int>> res;

	for (auto i = 0; i < start_tree.size(); i++)
	{	
		if (p.node.get_dist(start_tree[i].node) < PCTrajNode::config.rrt_star_param.r_near_rrt_star)
		{
			res.push_back({ true,i });
		}
	}

	return res;
}

//返回的是rewire的节点数目

//首先在已有的path上随便rand出一个点，然后按照d的范围搜近邻的点云，然后在近邻里面rand出一个作为输入reconstruct的点
int RRT_star::reconstruct(Vector3d p)
{
	int n = 0;

	// find nearist vertex
	// //cout << "find nearist vertex" << endl;
	pair<bool, int> nearss = { true, find_nearest(p) };//在当前的树上找一个最近的node
	// cout<<"(*direct.second).size() "<<(*direct.second).size()<<endl;
	// //cout << "nearss.second " <<nearss.second<< endl;
	// projection and get p_new
	////cout << "projection and get p_new" << endl;
	RRT_Node near_node = get_Node(nearss);
	// //cout << "nearss " <<near_node.node.get_pos()[0] <<near_node.node.get_pos()[1] <<near_node.node.get_pos()[2] << endl;
	// //cout << "p " <<p[0] <<p[1] <<p[2] << endl;
	Vector2d p_in_node_frame = near_node.node.projection(p);
	double dist = p_in_node_frame.norm();
	int loop=0;
	while(dist==0)
	{
		loop++;
		//cout << "loop " <<loop<< endl;
		Vector3d p = op_sample(3);
		pair<bool, int> nearss = { true, find_nearest(p) };//在当前的树上找一个最近的node
		// cout<<"(*direct.second).size() "<<(*direct.second).size()<<endl;
		// //cout << "nearss.second " <<nearss.second<< endl;
		// projection and get p_new
		////cout << "projection and get p_new" << endl;
		RRT_Node near_node = get_Node(nearss);
		// //cout << "nearss " <<near_node.node.get_pos()[0] <<near_node.node.get_pos()[1] <<near_node.node.get_pos()[2] << endl;
		// //cout << "p " <<p[0] <<p[1] <<p[2] << endl;
		Vector2d p_in_node_frame = near_node.node.projection(p);
		dist = p_in_node_frame.norm();
	}
		// //cout << "p_in_node_frame "<<p_in_node_frame << endl;
	Vector3d p_t(PCTrajNode::config.rrt_star_param.r_exp_rrtstar*p_in_node_frame[0] / dist, PCTrajNode::config.rrt_star_param.r_exp_rrtstar*p_in_node_frame[1] / dist, atan2(p_in_node_frame[1], p_in_node_frame[0]));
	// Vector3d p_t(PCTrajNode::config.rrt_star_param.r_exp_rrtstar*p_in_node_frame[0] / dist, PCTrajNode::config.rrt_star_param.r_exp_rrtstar*p_in_node_frame[1] / dist, 0.0);
	Matrix4d tr = near_node.node.Trans(p_t);
	//cout << "p_t "<<p_t << endl;
	PCTrajNode pc_new(tr);


		

	if (!pc_new.isTraversable())
	{
		//cout << "pc_new is not Traversable " << endl;
		return n;
	}
	else{
	}
		
	RRT_Node p_new(pc_new, {true,-1});
	//p_new是v_near朝着psample的点方向，沿着投影走了r_exp_rrtstar

	// find perfect parent
		
	vector<pair<bool, int>> nears = get_near(p_new);//这个多少取决于r_near_rrt_star  要大于r_exp_rrtstar
	// //cout << "nears size: " << nears.size() << endl;
	if (PCTrajNode::config.rrt_star_param.N_a == 0)
		PCTrajNode::config.rrt_star_param.N_a = nears.size();
	// cout <<"-----------------------------------------nears.size()"<< nears.size()<< endl;

	PCTrajNode::config.rrt_star_param.N_a = PCTrajNode::config.rrt_star_param.alpha * nears.size() + (1 - PCTrajNode::config.rrt_star_param.alpha)*PCTrajNode::config.rrt_star_param.N_a;
	// cout <<"-----------------------------------------PCTrajNode::config.rrt_star_param.N_a"<< PCTrajNode::config.rrt_star_param.N_a << endl;
	double d = std::numeric_limits<double>::infinity();
	pair<bool, int> upmost = { true,-1 };
	int good_neighbor=0;
	int false_neighbor=0;
	for (auto& i : nears)
	{
		RRT_Node p(get_Node(i));

		vector<PCTrajNode> srt = PCTrajNode::SRT_Generate(p.node, p_new.node);
		if(srt.size()==1) //可能在这里 挂 了一个1370 37.1 73.6 32.1 -22.75 52.75 5.75
		{
			// ROS_ERROR("11 可能在这里 挂 了一个1370 37.1 73.6 32.1 -22.75 52.75 5.75");
			continue;
		}
		if (srt.empty())
		{
			false_neighbor++;
			// cout <<"     connect p_new with p_new_near   failed"<< endl;
			continue;
		}
		// cout <<"     			connect p_new with p_new_near   successed"<< endl;
		good_neighbor++;
		Vector3d pt;
		pt(0)=p.node.T(0,3);
		pt(1)=p.node.T(1,3);
		pt(2)=p.node.T(2,3);
		// cout <<"index in tree(this successed neighbor)   "<<i.second<<"   p.node  "<< pt<<endl;
		// cout <<"     			srt.size()  "<< srt.size()<< endl;
		double dist = p.cost;//从start到邻居的cost
		// cout <<"  i.second     "<< i.second <<"   	dist  "<<dist<< endl;
		for (size_t j = 0; j < srt.size() - 1; j++)
		{
			dist += srt[j].get_cost();//PCTrajNode get_cost得到的是sf ;这里get_cost纯粹是空的 挂
			
		}
		if (dist < d)//找到Vnear里面所有可行的轨迹,从所有可行的轨迹中,选择一个从start tree起始点经过vnear中这个点到v_new最短的一个
		{
			d = dist;
			upmost = i;
		}
	}
		// Vector3d pt;
		// pt(0)=p_new.node.T(0,3);
		// pt(1)=p_new.node.T(1,3);
		// pt(2)=p_new.node.T(2,3);
		// publishSamplePoint(pt);
	// std::cout<<"good_neighbor"<<good_neighbor<<std::endl;
	// std::cout<<"false_neighbor"<<false_neighbor<<std::endl;
	// std::cout<<"nears.size()"<<nears.size()<<std::endl;


	// cout <<" upmost.second   "<< upmost.second <<endl;
	if (upmost.second == -1)
		return n;

	// cout <<"--------------------------------------------------------------------------------------------------------"<<endl;
	// add nodes to tree
	// //cout << "add nodes to tree" << endl;
		// Vector3d pt;
		// pt(0)=get_Node(upmost).node.T(0,3);
		// pt(1)=get_Node(upmost).node.T(1,3);
		// pt(2)=get_Node(upmost).node.T(2,3);
		// cout <<" index in tree    (upmost.node  "<< pt<<endl;

	vector<PCTrajNode> usrt = PCTrajNode::SRT_Generate(get_Node(upmost).node, p_new.node);//这里就不应该前面选到一个work的了还出现不work的
	// cout <<"--------------------------------------------------------------------------------------------------------"<<endl;
			if(usrt.size()==1) //可能在这里 挂 了一个1370 37.1 73.6 32.1 -22.75 52.75 5.75
		{
			// ROS_ERROR(" 22 可能在这里 挂 了一个1370 37.1 73.6 32.1 -22.75 52.75 5.75");
			return n;
		}
	
	if (usrt.empty())
	{
		// ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!这里就不应该前面选到一个work的了还出现不work的");
		return n;
	}

	pair<bool, int> tp = upmost;
	for (size_t j = 1; j < usrt.size() ; j++)//最后一个即p_new.node 也被扔进去了
	{
		RRT_Node temp(usrt[j],tp);
		temp.cost = get_Node(tp).cost + usrt[j - 1].get_cost();
		start_tree.push_back(temp);
		tp = { true, start_tree.size() - 1 };
	}

	if (upmost == nearss)
	{
		// std::cout<<"				upmost == nearss"<<std::endl;
		return n;
	}
		
	// std::cout<<"nears.size()"<<nears.size()<<std::endl;
	// ROS_ERROR("--------start   rewire------------" );
	for (size_t i = 0; i < nears.size(); i++)
	{
		vector<PCTrajNode> nsrt = PCTrajNode::SRT_Generate(p_new.node, get_Node(nears[i]).node);
		if(nsrt.size()==1) //可能在这里 挂 了一个1370 37.1 73.6 32.1 -22.75 52.75 5.75
		{
			// ROS_ERROR("可能在这里 挂 了一个1370 37.1 73.6 32.1 -22.75 52.75 5.75");
			continue;
		}
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
				// cout<<"nsrt[j - 1].get_cost() "<<nsrt[j - 1].get_cost() <<endl;
				start_tree.push_back(temp);
				tpnew = { true, start_tree.size() - 1 };
			}
			ni.parent = tpnew;
			ni.cost = dist;
			// ROS_ERROR("1maybe die there111");
			update_cost(nears[i]);
			// ROS_ERROR("2maybe die there222");
		}
	}

	PCTrajNode::config.rrt_star_param.N_rewire += n;
	// ROS_ERROR("rewired %d ", n );
	//cout << "rewired " << n << " nodes" << endl;
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
	// ROS_ERROR("我记得是没有用到rrtstar的extend的");
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
			if (p_new.node.get_dist((*direct.second)[i].node) < PCTrajNode::config.rrt_star_param.r_near_rrt_star)
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
		// ROS_WARN("Not traversable");
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
					// //cout << "RRT_star begin!" << endl;
					// pcl::StopWatch time;
					// for (int i = 0; i < max_iter; i++)
					// {
					// 	// sample, extend this tree
					// 	// //cout << "iteration: " << i << endl;
					// 	if (i % 10 == 0)
					// 	{
					// 		ROS_WARN("Iteration: %d", i);
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
					// 			//cout << "get path in iteration: " << i << endl;
					// 			//cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
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
					// 					//cout << "get path in  iteration: " << i << endl;
					// 					//cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
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
					// //cout << "can't find path!!!" << endl;
					// ROS_WARN("start tree size: %d, goal tree size: %d", start_tree.size(), goal_tree.size());
					// inProcess = false;
					// return result;

	/*---- do rrt first, than optimization ----*/
	ros::Time t_rrt = ros::Time::now();
	path = find_path(start, goal);
	cout<<"rrt done,time consume = "<<(ros::Time::now() - t_rrt).toSec() * 1000<<"ms"<<endl;
	if (path.empty())
	{
		ROS_ERROR("RRT faileddddddddddddddddddddd ");
		inProcess = false;
		return path;
	}
	// get_path();
	cout<<"path.size() "<<path.size()<<endl;
	ROS_ERROR("-----------------start rewire-----------------");
	for(int i=0;i<path.size();i++)
	{
		if(path[i].node.T(2,3)>0.5 && path[i].node.T(2,3)<4)
		{
			path_circle.push_back(path[i]);
		}
		
	}
	cout<<"path_circle.size() "<<path_circle.size()<<endl;
	// vector<PCTrajNode> p;

	// vector<RRT_Node>::reverse_iterator riter;
	// for (riter = path.rbegin(); riter != path.rend(); riter++)
	// {
	// 	//cout << "(*riter).node.t(4) " << (*riter).node.get_cost() << std::endl;
	// 	// p.push_back((*riter).node);
	// }
					// 论文里说这个d是 和 起点与终点之间的距离成正比，距离越长，rand出path上一个点之后，在这个r的球内resample，path越长，球越大
					//用path.size()*PCTrajNode::PR.d_nom当作路径长度，path_r是占比，比如十分之一这样
					//d是rand的球的半径，单位是m
						float d = PCTrajNode::config.rrt_star_param.path_r*path.size()*PCTrajNode::PR.d_nom;
						
						
						// PCTrajNode::config.rrt_star_param.N_max_avg = d*3;
						// PCTrajNode::config.rrt_star_param.N_max_avg = 2*PCTrajNode::config.rrt_star_param.r_near_rrt_star/PCTrajNode::PR.d_nom ;
						if (!direct.first)
							change_direction();

						int refine_times = 0;
						PCTrajNode::config.rrt_star_param.N_a = 0;
						PCTrajNode::config.rrt_star_param.N_rewire = 0;
						// //cout << "begin refine path" << endl;
						pcl::StopWatch time;
						// while (PCTrajNode::config.rrt_star_param.N_a < PCTrajNode::config.rrt_star_param.N_max_avg)
						// while (1)
						double kkk=path.size()/50.0;
						// kkk=1;
						double max_num=kkk*PCTrajNode::config.rrt_star_param.max_refine_num;
						if(max_num>2000)
							max_num=2000;
						cout << "refine_times " << refine_times << endl;
						cout<<"max_times "<<kkk*PCTrajNode::config.rrt_star_param.max_refine_num<<endl;
						//cout << "poject done, refine consume: " << time.getTime() / 1000 << "s" << std::endl;
						
						cout << "N_max_avg: " << kkk*PCTrajNode::config.rrt_star_param.N_max_avg << endl;
						while (PCTrajNode::config.rrt_star_param.N_rewire < kkk*PCTrajNode::config.rrt_star_param.N_max_avg)
						{	
							// //cout << "PCTrajNode::config.rrt_star_param.N_max_avg " << PCTrajNode::config.rrt_star_param.N_max_avg << endl;
							// cout << "refine_times: " << refine_times << endl;
							Vector3d p;
							if(refine_times < 100)//d 0.3   norm 0.4 exp 0.2   +  d 3   norm 1 exp 1 
							{
								if(path_circle.size()>10)
								{
								d = 1;
								PCTrajNode::PR.d_nom=0.5;
								PCTrajNode::config.rrt_star_param.r_exp_rrtstar=0.5;
								p = op_sample_circle(d);
								}
								else{
								d = 3;
								PCTrajNode::PR.d_nom=1.5;
								PCTrajNode::config.rrt_star_param.r_exp_rrtstar=1.5;
								p = op_sample(d);
								}

								// publishSamplePoint(p);
							}
							else{
								d = 3;
								PCTrajNode::PR.d_nom=1.5;
								PCTrajNode::config.rrt_star_param.r_exp_rrtstar=1.5;
								p = op_sample(d);
								// publishSamplePoint(p);
							}
							
							// publishSamplePoint(p);
							// reconstruct(p);
							//alpha等于0.1
							PCTrajNode::config.rrt_star_param.N_rewire = PCTrajNode::config.rrt_star_param.alpha * reconstruct(p) + (1 - PCTrajNode::config.rrt_star_param.alpha)*PCTrajNode::config.rrt_star_param.N_rewire;
							// cout << "PCTrajNode::config.rrt_star_param.N_rewire   " << PCTrajNode::config.rrt_star_param.N_rewire << std::endl;
							
							// get_path();
							// visualizeTrees();
							if (refine_times > max_num)
							{
								break;
							}
							// if (refine_times > max_num/2)
							// {
							// 	PCTrajNode::PR.d_nom=0.6;
							// 	PCTrajNode::config.rrt_star_param.r_exp_rrtstar=0.4;
							// }
							refine_times ++;
						}
						PCTrajNode::PR.d_nom=0.6;
						//cout << "-----------------------------------------------d是rand的球的半径 " << d << std::endl;
						cout << "refine_times " << refine_times << endl;
						cout<<"max_times "<<kkk*PCTrajNode::config.rrt_star_param.max_refine_num<<endl;
						//cout << "poject done, refine consume: " << time.getTime() / 1000 << "s" << std::endl;
						cout << "refine nodes number: " << PCTrajNode::config.rrt_star_param.N_rewire << endl;
						cout << "N_max_avg: " << PCTrajNode::config.rrt_star_param.N_max_avg << endl;
	path = get_path();

	return  path;
}

// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "RRT_star_node");
// 	ROS_WARN("Planner started!");
// 	RRT_star planner;
// 	ros::spin();
// }