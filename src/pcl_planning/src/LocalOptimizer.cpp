#include "LocalOptimizer.h"

extern bool mapReceived;

void LocalOptimizer::visualizePath(vector<PCTrajNode> path)
{
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();

	geometry_msgs::Pose pose;
	tf::Matrix3x3 R;
	Matrix4d T;
	Vector3d point, point_parent;
	geometry_msgs::Point point_msg1, point_msg2;

	for (auto node : path)
	{
		T = node.get_T();
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

void LocalOptimizer::setGoalCallback(const geometry_msgs::PoseStamped &goal_msg)
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
		optimized_path(pseudoOdom, pcn);
		visualizeTrees();
	}
	else
	{
		ROS_INFO("No map received yet! Can't plan now.");
	}
}

static pair<double, double> get_direct(Vector2d v1, Vector2d v3)
{
	double d1 = -(v1[0] * v1[0] + v1[1] * v1[1]);
	double d2 = v3[0] * v3[0] + v3[1] * v3[1];
	double fm = 2 * (v3[0] * v1[1] - v3[1] * v1[0]);
	Vector2d o((v3[1] * d1 + v1[1] * d2) / fm, (-v1[0] * d2 - v3[0] * d1) / fm);
	double ka = 1.0 / o.norm();
	double theta = atan2(o[1], o[0]);
	if (theta > 0)
	{
		theta -= M_PI_2;
	}
	else
	{
		theta += M_PI_2;
	}

	return  { ka,theta };
}

vector<PCTrajNode> LocalOptimizer::optimized_path(const PCTrajNode& start, const PCTrajNode& goal)
{
	// initialization
	vector<RRT_Node> path = refined_path(start, goal);
	vector<PCTrajNode> p;
	if(path.empty())
	{
		inProcess = false;
		return p;
	}
	bool op_down = false;
	vector<RRT_Node>::reverse_iterator riter;
	for (riter = path.rbegin(); riter != path.rend(); riter++)
	{
		p.push_back((*riter).node);
	}

	vector<double> del(p.size(), delta_max);
	double bh = delta_min;
	int cnt = 0;
	cout<<"begin optimization!"<<endl;
	pcl::StopWatch time;
	do
	{
		// check distance between nodes
		cout << "optimization: " << ++cnt << endl;
		cout << "check distance between nodes" << endl;
		auto id = del.begin();
		for (auto i= p.begin(); i != p.end()-1; i++, id++)
		{
			double dist = (*i).get_dist(*(i+1));
			if (dist > din_max)
			{
				if (i + 1 != p.end() - 1)
				{
					cout << "add nodes" << endl;
					VectorXd pat = PCTrajNode::get_connect(*i, *(i + 1));
					if (pat[4]==0)
					{
						continue;
					}
					Vector4d path_seg;// = forward(init, x);
					double seg = pat(4)/2;
					double seg2 = seg*seg;
					double seg3 = seg*seg2;
					double seg4 = seg*seg3;
					double seg5 = seg*seg4;
					path_seg[0] = seg;
					path_seg[1] = pat(0)*seg2/2+ pat(1)*seg3/6+pat(2)*seg4/12+pat(3)*seg5/20;
					path_seg[2] = pat(0)*seg+ pat(1)*seg2/2+pat(2)*seg3/3+pat(3)*seg4/4;
					path_seg[3] = pat(0) + pat(1)*seg+pat(2)*seg2+pat(3)*seg3;
					Matrix4d mid_T = (*i).Trans(path_seg.head(3));
					PCTrajNode mid_node(mid_T, path_seg[3]);
					i = p.insert(i + 1, mid_node);
					i--;
					id = del.insert(id + 1, *id);
					id--;
				}
			}
			else if (dist < din_min)
			{
				if (i + 1 != p.end() - 1 && i + 2 != p.end() - 1)
				{
					cout << "erase nodes" << endl;
					id = del.erase(id + 1);
					i = p.erase(i + 1);
					id = del.erase(id);
					i = p.erase(i);
					id--;
					i--;
					VectorXd pat = PCTrajNode::get_connect(*i, *(i + 1));
					Vector4d path_seg;// = forward(init, x);
					if (pat[4]==0)
					{
						continue;
					}
					double seg = pat(4)/2;
					double seg2 = seg*seg;
					double seg3 = seg*seg2;
					double seg4 = seg*seg3;
					double seg5 = seg*seg4;
					path_seg[0] = seg;
					path_seg[1] = pat(0)*seg2/2+ pat(1)*seg3/6+pat(2)*seg4/12+pat(3)*seg5/20;
					path_seg[2] = pat(0)*seg+ pat(1)*seg2/2+pat(2)*seg3/3+pat(3)*seg4/4;
					path_seg[3] = pat(0) + pat(1)*seg+pat(2)*seg2+pat(3)*seg3;
					Matrix4d mid_T = (*i).Trans(path_seg.head(3));
					PCTrajNode mid_node(mid_T, path_seg[3]);
					i = p.insert(i + 1, mid_node);
					i--;
					id = del.insert(id + 1, *id);
					id--;
				}
			}
			(*i).connect(*(i + 1));
			if ((*i).t == VectorXd::Zero(5))
			{
				cout << "something wrong! check the path!" << endl;
			}
			
			shadow temp;
			temp.s[0] = *i;
			temp.delta = *id;
			Graph.push_back(temp);
		}
		// deal with start and goal
		shadow temp;
		temp.s[0] = p.back();
		temp.s[1] = p.back();
		temp.s[2] = p.back();
		temp.delta = delta_min;
		Graph.push_back(temp);
		Graph[0].s[1] = Graph[0].s[2] = Graph[0].s[0];
		Graph[0].delta = delta_min;

		// separate
		cout << "separate" << endl;
		separate();
		
		// dijkstra
		cout << "dijkstra" << endl;
		vector<int> opath = dijk();
		op_down = true;
		for (size_t i = 1; i < opath.size() - 1; i++)
		{
			if (opath[i] != 0)
			{
				if (del[i] > delta_min)
					del[i] -= 0.01;
				op_down = false;
				p[i] = Graph[i].s[opath[i]];
			}
		}
		Graph.clear();
		if (cnt>=100)
			break;
	} while (!op_down && any_of(del.begin(), del.end(), [bh](double i) {return i > bh; }));
	cout<<"optimization done, iteration = "<<cnt<<"      time consume = "<<time.getTime()/1000<<"s"<<endl;
	visualizePath(p);

	return p;
}

void LocalOptimizer::separate(void)
{
	// generate left and right subnodes
	for (size_t i = 1; i < Graph.size() - 1; i++)
	{
		shadow& sh = Graph[i];
		Matrix4d left = sh.s[0].Trans({ 0,-sh.delta,0 });
		Matrix4d right = sh.s[0].Trans({ 0,sh.delta,0 });
		sh.s[1] = PCTrajNode(left);
		sh.s[2] = PCTrajNode(right);
	}

	// special node, 1
	shadow& sh = Graph[1];
	pair<double, double> gl = get_direct(sh.s[1].projection(Graph[0].s[0].get_pos()), \
		sh.s[1].projection(Graph[2].s[0].get_pos()));
	sh.s[1].change_T({ 0,0,gl.second });
	sh.s[1].kappa = gl.first;
	pair<double, double> gr = get_direct(sh.s[2].projection(Graph[0].s[0].get_pos()), \
		sh.s[2].projection(Graph[2].s[0].get_pos()));
	sh.s[2].change_T({ 0,0,gr.second });
	sh.s[2].kappa = gr.first;

	// special node, last-1
	shadow& sh1 = Graph[Graph.size()-2];
	gl = get_direct(sh1.s[1].projection(Graph.back().s[0].get_pos()), \
		sh1.s[1].projection(Graph[Graph.size() - 3].s[0].get_pos()));
	sh1.s[1].change_T({ 0,0,gl.second });
	sh1.s[1].kappa = gl.first;
	gr = get_direct(sh1.s[2].projection(Graph.back().s[0].get_pos()), \
		sh1.s[2].projection(Graph[Graph.size() - 3].s[0].get_pos()));
	sh1.s[2].change_T({ 0,0,gr.second });
	sh1.s[2].kappa = gr.first;

	// othors
	for (size_t i = 2; i < Graph.size() - 2; i++)
	{
		shadow& sh = Graph[i];
		for (int k = 1; k < 3; k++)
		{
			pair<double, double> gd{ 0,0 };
			for (int j = 0; j < 3; j++)
			{
				pair<double,double> temp = get_direct(sh.s[k].projection(Graph[i - 1].s[j].get_pos()), \
					sh.s[k].projection(Graph[i + 1].s[j].get_pos()));
				gd.first += temp.first;
				gd.second += temp.second;
			}
			gd.first /= 3.0;
			gd.second /= 3.0;
			sh.s[k].change_T({ 0,0,gd.second });
			sh.s[k].kappa = gd.first;
		}
	}
}

double LocalOptimizer::get_op_cost(size_t index, int from, int to)
{
	PCTrajNode from_node = Graph[index].s[from];
	PCTrajNode to_node = Graph[index + 1].s[to];
	VectorXd par = PCTrajNode::get_connect(from_node, to_node);

	if (par == VectorXd::Zero(5))
	{
		//cout << "cubic" << endl;
		return std::numeric_limits<double>::infinity();
	}

	double cost = 0;
	
	// get tau
	double totau;
	if (to_node.isTraversable())
	{
		totau = to_node.tau;
	}
	else
	{
		//cout << "traverse" << endl;
		return std::numeric_limits<double>::infinity();
	}

	// get kappa
	double kappa = 0;
	for (double dist = 0.0; dist < par[4]; dist += 0.01)
	{
		double kap = 0;
		for (int j = 3; j >= 0; j--)
		{
			kap *= dist;
			kap += par[j];
		}
		if (kap > PCTrajNode::PR.kappa_max)
		{
			//cout << "kappa = " << kap << " to high!" << endl;
			return std::numeric_limits<double>::infinity();
		}
		else if (kap > kappa)
		{
			kappa = kap;
		}
	}

	// get dist
	double d = Graph[index].s[from].get_dist(Graph[index + 1].s[to]);
	d = (d - din_min) / (din_max - din_min);

	return w_len * d + w_trav * (1 - totau) + w_curv * kappa / PCTrajNode::PR.kappa_max;
}

vector<int> LocalOptimizer::dijk(void)
{
	vector<int> result;
	vector<PCTrajNode> map;
	vector<bool> collected;
	vector<int> parent;
	vector<double> dist;
	
	map.push_back(Graph[0].s[0]);
	collected.push_back(false);
	parent.push_back(-1);
	dist.push_back(0);

	for (size_t i = 1; i < Graph.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			map.push_back(Graph[i].s[j]);
			collected.push_back(false);
			parent.push_back(-1);
			dist.push_back(numeric_limits<double>::infinity());
		}
	}

	while (1)
	{
		double d = numeric_limits<double>::infinity();
		int index = -1;
		for (size_t i = 0; i < map.size(); i++)
		{
			if (!collected[i] && dist[i] < d)
			{
				d = dist[i];
				index = i;
			}
		}
		if (index == -1 || (index - 1) / 3 == Graph.size() - 2)
		{
			cout << "cost is: " << dist[index] << endl;
			break;
		}
		collected[index] = true;
		for (int i = 0; i < 3; i++)
		{
			int nei = index - (index-1) % 3 + i + 3;
			if (index == 0)
			{
				nei = 1;
			}
			if (!collected[nei])
			{
				double c = dist[index];
				if (index % 3 == 0)
				{
					c += get_op_cost(index / 3, 2, i);
				}
				else
				{
					c += get_op_cost(index / 3 + 1, index % 3 - 1, i);
				}
				if (dist[nei] > c)
				{
					dist[nei] = c;
					parent[nei] = index;
				}
			}
		}
	}
	result.push_back(0);
	int i = map.size() - 1;
	while (parent[i] != 0)
	{
		result.push_back((parent[i] - 1) % 3);
		i = parent[i];
	}
	result.push_back(0);
	reverse(result.begin(), result.end());

	double ocost = 0;
	for (size_t i = 0; i < Graph.size() - 1; i++)
	{
		ocost += get_op_cost(i, 0, 0);
	}
	cout << "ocost=" << ocost << endl;
	if (ocost>1e7)
	{
		for (auto p:result)
			p=0;
	}
	return result;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "LocalOpitizater_node");
	ROS_INFO("Planner started!");
	LocalOptimizer planner;
	ros::spin();
}