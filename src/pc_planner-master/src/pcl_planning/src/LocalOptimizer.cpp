#include "LocalOptimizer.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "config.hpp"
extern bool mapReceived;
extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_filter;
    void LocalOptimizer::draw_path_line(vector<PCTrajNode> p)
    {
        int id = 0;
        double sc = 0.1;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "map";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
		int i=0;
				for (auto node : p)
			{
				double x0=node.T(0,3);
				double y0=node.T(1,3);
				pt.x = x0;
				pt.y = y0;
				pt.z = 0.0;
				line_strip.points.push_back(pt);
				i++;
			}

        line_pub.publish(line_strip);
    }
void LocalOptimizer::triggerCallBack(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	
	// trgger_for_debug=false;
	// return;
    ofstream file;
    std::ofstream outfile;
    file.open("/home/xulong/drving_on_pc_muti_layer/data/drive_on_pc_traj_start_end.txt", std::ofstream::out);
    outfile.open("/home/xulong/drving_on_pc_muti_layer/data/drive_on_pc_traj.txt", std::ofstream::out);
    outfile.clear();
    plan_time.open("/home/xulong/drving_on_pc_muti_layer/data/drive_on_pc_plan_time.txt", std::ofstream::out);
	plan_time.clear();
    access_time.open("/home/xulong/drving_on_pc_muti_layer/data/drive_on_pc_access_time.txt", std::ofstream::out);
	access_time.clear();
    traj_length.open("/home/xulong/drving_on_pc_muti_layer/data/drive_on_pc_traj_length.txt", std::ofstream::out);
	traj_length.clear();
	avg_cur.open("/home/xulong/drving_on_pc_muti_layer/data/drive_on_pc_average_curvature.txt", std::ofstream::out);
	avg_cur.clear();

    
    string data_line;
    int n = 0;
    unsigned seed;
    seed = time(0);
    srand(seed);
    while(n<1500)
    {
					// while(!trgger_for_debug)
					// {
					// 	ros::spinOnce();
					// }
		
        // string data;
        // vector<string> datas;
        // stringstream line(data_line);
        // while(line >> data)
        //     datas.push_back(data);

        // std::vector<int> pointIdxNKNSearch(10);
        // std::vector<float> pointNKNSquaredDistance(10);
        // pcl::PointXYZI searchPoint, end_pt;
        // searchPoint.x = stod( datas[0] );
        // searchPoint.y = stod( datas[1] );
        // searchPoint.z = stod( datas[2] );

        // end_pt.x = stod( datas[3] );
        // end_pt.y = stod( datas[4] );
        // end_pt.z = stod( datas[5] );

        std::vector<int> pointIdxNKNSearch(10);
        std::vector<float> pointNKNSquaredDistance(10);
        pcl::PointXYZI searchPoint, end_pt;
		Vector3d sample_start=sample();
		Vector3d sample_end=sample();
        searchPoint.x = sample_start(0);
        searchPoint.y = sample_start(1);
        searchPoint.z = sample_start(2);
publishSamplePoint_2(sample_start);
        end_pt.x = sample_end(0);
        end_pt.y = sample_end(1);
        end_pt.z = sample_end(2);
publishSamplePoint(sample_end);
		double dist_tmp_sample_s_e=(sample_start-sample_end).norm();
		// cout<<"dist_tmp_sample_s_e "<<dist_tmp_sample_s_e<<endl;
		if(dist_tmp_sample_s_e<1)
			continue;
		// cout<<"searchPoint "<<searchPoint.x<< searchPoint.y<<searchPoint.z<<endl;
		// cout<<"end_pt "<<end_pt.x<< end_pt.y<<end_pt.z<<endl;
		size_t start_idx ;size_t end_idx;

		// if((searchPoint.z>10) && (end_pt.z>10))
		// 	continue;
		// if((searchPoint.z>10) && (end_pt.z<=10))
		// 	searchPoint.z=end_pt.z;
		// if((end_pt.z>10) && (searchPoint.z<=10))
		// 	end_pt.z=searchPoint.z;


        // kdtree_filter.nearestKSearch(searchPoint, 10, pointIdxNKNSearch, pointNKNSquaredDistance);
		// start_idx= pointIdxNKNSearch[0];
        // kdtree_filter.nearestKSearch(end_pt, 10, pointIdxNKNSearch, pointNKNSquaredDistance);
		// end_idx = pointIdxNKNSearch[0];
		bool start_is_traversable=false;
		bool end_is_traversable=false;
		bool pc_start_traj_lib_is_ok=false;
		bool pcl_goal_traj_lib_is_ok=false;
		int iii=0,jjj=0;
		int start_index_traj_id=0,end_index_traj_id=0;
		for(iii=0;iii<10;iii++)
		{
			//std::cout<<"iii "<<iii<<std::endl;
			kdtree_filter.nearestKSearch(searchPoint, 10, pointIdxNKNSearch, pointNKNSquaredDistance);
			start_idx= pointIdxNKNSearch[iii];

			Matrix4d start_mat = Matrix4d::Identity();
			start_mat(0, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].x;
			start_mat(1, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].y;
			start_mat(2, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].z;
			PCTrajNode pcl_start(start_mat);

			if(pcl_start.isTraversable())
			{
				start_is_traversable=true;
				for(start_index_traj_id=0;start_index_traj_id<17;start_index_traj_id++){
					Matrix4d T_i;
					T_i = pcl_start.Trans(traj_lib[start_index_traj_id]);
					PCTrajNode pc_new(T_i);
					if(pc_new.isTraversable()){
						pc_start_traj_lib_is_ok=true;
						cout<<"pc_start_traj_lib_is_ok "<<start_index_traj_id<<endl;
						break;
					}
				}
				break;
			}
			else{
				start_is_traversable=false;
				cout<<"pcl_start.isTraversable() "<<pcl_start.isTraversable()<<endl;
			}
				
		}
		if((!pc_start_traj_lib_is_ok) || (!start_is_traversable))
			continue;
		for(jjj=0;jjj<10;jjj++)
		{
			//std::cout<<"jjj "<<jjj<<std::endl;
			kdtree_filter.nearestKSearch(end_pt, 10, pointIdxNKNSearch, pointNKNSquaredDistance);
			end_idx = pointIdxNKNSearch[jjj];

			Matrix4d goal_mat = Matrix4d::Identity();

			goal_mat(0, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].x;
			goal_mat(1, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].y;
			goal_mat(2, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].z;
			PCTrajNode pcl_goal(goal_mat);

			if(pcl_goal.isTraversable() )
			{
				end_is_traversable=true;
				for(end_index_traj_id=0;end_index_traj_id<17;end_index_traj_id++){
					Matrix4d T_i;
					T_i = pcl_goal.Trans(traj_lib_back[end_index_traj_id]);
					PCTrajNode pc_new(T_i);
					if(pc_new.isTraversable()){
						pcl_goal_traj_lib_is_ok=true;
						// cout<<"pcl_goal_traj_lib_is_ok "<<end_index_traj_id<<endl;
						break;
					}
				}
				break;
			}
			else{
				end_is_traversable=false;
				// cout<<"pcl_goal.isTraversable() "<<pcl_goal.isTraversable()<<endl;
			}
				
		}
		if((!end_is_traversable) ||(!pcl_goal_traj_lib_is_ok))
			continue;
		// cout<<"iii "<<iii<<endl;
		// cout<<"jjj "<<jjj<<endl;
		// cout<<"start_index_traj_id "<<start_index_traj_id<<endl;
		// cout<<"end_index_traj_id "<<end_index_traj_id<<endl;
        ROS_WARN("start=%f, %f, %f", PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].x, PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].y,PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].z);
        ROS_WARN("end=%f, %f, %f", PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].x, PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].y,PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].z);
		
		Vector3d p(PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].x, PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].y,PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].z);
		cout<<"p "<<p<<endl;
		// publishSamplePoint(p);
		Vector3d p2(PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].x, PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].y,PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].z);
		cout<<"p2 "<<p2<<endl;
		// publishSamplePoint_2(p2);
	
//--------------

		//TODO
		
			Matrix4d start_mat = Matrix4d::Identity();
			Matrix4d goal_mat = Matrix4d::Identity();
			start_mat(0, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].x;
			start_mat(1, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].y;
			start_mat(2, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].z;

			goal_mat(0, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].x;
			goal_mat(1, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].y;
			goal_mat(2, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].z;
			cout<<"start_mat "<<start_mat(0, 3)<<"  "<<start_mat(1, 3)<<"  "<<start_mat(2, 3)<<endl;
			cout<<"goal_mat "<<goal_mat(0, 3)<<"  "<<goal_mat(1, 3)<<"  "<<goal_mat(2, 3)<<endl;
			PCTrajNode pcl_goal(goal_mat), pcl_start(start_mat);
		if (!pcl_goal.isTraversable())
		{
			ROS_WARN("goal is not traversable");
			//std::cout<<"traversibility "<<pcl_goal.tau<<std::endl;
			// return;
		}
		if (!pcl_start.isTraversable())
		{
			ROS_WARN("start is not traversable");
			//std::cout<<"traversibility "<<pcl_start.tau<<std::endl;
			// return;
		}
		vector<PCTrajNode> result_node;
		result_node=optimized_path(pcl_start, pcl_goal);
		// visualizeTrees();
		int i=0;
		double all_length=0;
		double average_curvature=0;
		if(result_node.size()==0)
			continue;
		file<<start_mat(0, 3)<<" "<<start_mat(1, 3)<<" "<<start_mat(2, 3)<<" "<<goal_mat(0, 3)<<" "<<goal_mat(1, 3)<<" "<<goal_mat(2, 3)<<" "<<std::endl;
		
		int count=0;
			for (auto node : result_node)
		{
			double x0=node.T(0,3);
			double y0=node.T(1,3);
			double theta0=atan2(node.T(1,0),node.T(0,0));
			double k0=node.t[0];
			double a=node.t[1];
			double b=node.t[2];
			double c=node.t[3];
			double sf=node.t[4];

			if(i<result_node.size()-1){
			PCTrajNode node1=result_node[i];
			PCTrajNode node2=result_node[i+1];
			double dist = (node1).get_dist(node2);
			// cout<<"dist"<<dist<<endl;
			all_length+=dist;
			}

			if(i<result_node.size()-2 && i!=1){
			PCTrajNode node1=result_node[i];
			PCTrajNode node2=result_node[i+1];
			PCTrajNode node3=result_node[i+2];
			double dist = (node1).get_dist(node2);
			// cout<<"dist"<<dist<<endl;
			// all_length+=dist;
			// double theta1=atan2(node1.T(1,0),node1.T(0,0));
			// double theta2=atan2(node2.T(1,0),node2.T(0,0));
			Vector2d tmp_1((node2.T(0,3)-node1.T(0,3)),(node2.T(1,3)-node1.T(1,3)));
			Vector2d tmp_2((node3.T(0,3)-node2.T(0,3)),(node3.T(1,3)-node2.T(1,3)));
			double cos_value=tmp_2.dot(tmp_1)/(tmp_1.norm()*tmp_2.norm());
			double angle_Rad=acos(cos_value);
			average_curvature+=abs(angle_Rad)/dist;
			count++;
			}

			if (i==result_node.size()-1)                   
				outfile<<x0<<" "<<y0<<" "<<theta0<<" "<<k0<<" "<<a<<" "<<b<<" "<<c<<" "<<sf<<std::endl;
			else
				outfile<<x0<<" "<<y0<<" "<<theta0<<" "<<k0<<" "<<a<<" "<<b<<" "<<c<<" "<<sf<<"  ";
			i++;
		}
		all_average_curvature+=average_curvature/count;
		traj_length_all+=all_length;

		access_time<<pre_time<<std::endl;
		traj_length<<all_length<<std::endl;
		avg_cur<<average_curvature/count<<std::endl;
		count++;
		if(result_node.size()==0)
			outfile<<"may failed result_node.size()==0"<<std::endl;
		trgger_for_debug=false;
		n++;

		
    }
	traj_length<<"traj_length_all "<<traj_length_all<<std::endl;
	plan_time<<"plan_time_all "<<plan_time_all<<std::endl;
	avg_cur<<"avg_cur_all "<<all_average_curvature<<std::endl;

    outfile.close();
    plan_time.close();
	access_time.close();
	traj_length.close();
}
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
	//std::cout<<"---------------------------------------------------------------------------------------------------------path.size() "<<path.size()<<std::endl;
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
	trgger_for_debug=false;
	//std::cout<<"in LocalOptimizer goal cb"<<std::endl;
	// if(!first_in_cb)
		// return;
	if (mapReceived)
	{	
		// first_in_cb=false;
		// tf::Quaternion q;
		// tf::quaternionMsgToTF(goal_msg.pose.orientation, q);
		// tf::Matrix3x3 R(q);

		// Matrix4d goal_mat = Matrix4d::Identity();
		// for (int i = 0; i < 3; i++)
		// {
		// 	for (int j = 0; j < 3; j++)
		// 	{
		// 		goal_mat(i, j) = R[i][j];
		// 	}
		// }
		// goal_mat(0, 3) = goal_msg.pose.position.x;
		// goal_mat(1, 3) = goal_msg.pose.position.y;
		// goal_mat(2, 3) = goal_msg.pose.position.z;

		// PCTrajNode pcn(goal_mat), pseudoOdom(Matrix4d::Identity());
		// if (!pcn.isTraversable())
		// {
		// 	ROS_WARN("goal is not traversable");
		// 	return;
		// }

		// optimized_path(pseudoOdom, pcn);
		// visualizeTrees();
//-----------------------------------------------------------------------------
		//TODO

			// Matrix4d start_mat = Matrix4d::Identity();
			// Matrix4d goal_mat = Matrix4d::Identity();
			// start_mat(0, 3) = -30.25;
			// start_mat(1, 3) = 76.75;
			// start_mat(2, 3) = 4.25;
  
			// goal_mat(0, 3) = -25.75;
			// goal_mat(1, 3) = 87.25;
			// goal_mat(2, 3) = 4.25;
			// PCTrajNode pcl_goal(goal_mat), pcl_start(start_mat);
			// optimized_path(pcl_start, pcl_goal);
			// visualizeTrees();
//-----------------------------------------------------------------------------  
   		std::vector<int> pointIdxNKNSearch(10);
        std::vector<float> pointNKNSquaredDistance(10);
        pcl::PointXYZI searchPoint, end_pt;//-18 13 0.3
        searchPoint.x = 19.600142; 
        searchPoint.y = 17.399998;
        searchPoint.z = 0;
        // searchPoint.x = 0;
        // searchPoint.y = 0;
        // searchPoint.z = 0;
		end_pt.x = goal_msg.pose.position.x;
		end_pt.y = goal_msg.pose.position.y;
		end_pt.z = goal_msg.pose.position.z;           
		// end_pt.x = 45.799965;
		// end_pt.y = -30.800041;
		// end_pt.z = 0.200000; 
//-----------------
		// size_t start_idx ;size_t end_idx;
        // kdtree_filter.nearestKSearch(searchPoint, 10, pointIdxNKNSearch, pointNKNSquaredDistance);
		// start_idx= pointIdxNKNSearch[0];
        // kdtree_filter.nearestKSearch(end_pt, 10, pointIdxNKNSearch, pointNKNSquaredDistance);
		// end_idx = pointIdxNKNSearch[0];

		// for(int j=0;j<10;j++)
		// {
		// 	kdtree_filter.nearestKSearch(searchPoint, 10, pointIdxNKNSearch, pointNKNSquaredDistance);
		// 	start_idx= pointIdxNKNSearch[j];
		// 	kdtree_filter.nearestKSearch(end_pt, 10, pointIdxNKNSearch, pointNKNSquaredDistance);
		// 	end_idx = pointIdxNKNSearch[j];

		// 	Matrix4d start_mat = Matrix4d::Identity();
		// 	Matrix4d goal_mat = Matrix4d::Identity();
		// 	start_mat(0, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].x;
		// 	start_mat(1, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].y;
		// 	start_mat(2, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].z;

		// 	goal_mat(0, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].x;
		// 	goal_mat(1, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].y;
		// 	goal_mat(2, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].z;
		// 	PCTrajNode pcl_goal(goal_mat), pcl_start(start_mat);
		// 	bool pc_start_traj_lib_is_ok=false;
		// 	bool pcl_goal_traj_lib_is_ok=false;
		// 	if(pcl_goal.isTraversable() && pcl_start.isTraversable())
		// 	{
		// 		for(int index=0;index<17;index++){
		// 			Matrix4d T_i;
		// 			T_i = pcl_start.Trans(traj_lib[index]);
		// 			PCTrajNode pc_new(T_i);
		// 			if(pc_new.isTraversable()){
		// 				pc_start_traj_lib_is_ok=true;
		// 				cout<<"pc_start_traj_lib_is_ok "<<index<<endl;
		// 				break;
		// 			}
		// 		}
		// 		for(int index=0;index<17;index++){
		// 			Matrix4d T_i;
		// 			T_i = pcl_goal.Trans(traj_lib_back[index]);
		// 			PCTrajNode pc_new(T_i);
		// 			if(pc_new.isTraversable()){
		// 				pcl_goal_traj_lib_is_ok=true;
		// 				cout<<"pcl_goal_traj_lib_is_ok "<<index<<endl;
		// 				break;
		// 			}
		// 		}
		// 		if(pc_start_traj_lib_is_ok && pcl_goal_traj_lib_is_ok)
		// 			break;
		// 	}
				
		// }
//-----------------
		// Vector3d p(PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].x, PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].y,PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].z);
		// cout<<"p "<<p<<endl;
		// publishSamplePoint(p);
		// Vector3d p2(PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].x, PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].y,PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].z);
		// cout<<"p2 "<<p2<<endl;
		// publishSamplePoint_2(p2);
		// // Vector3d p(8,39,5);
		// // publishSamplePoint(p);
		// // Vector3d p2(-19,-23,3);
		// // publishSamplePoint_2(p2);
			Matrix4d start_mat = Matrix4d::Identity();
			Matrix4d goal_mat = Matrix4d::Identity();
			// start_mat(0, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].x;
			// start_mat(1, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].y;
			// start_mat(2, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[start_idx].z;

			// goal_mat(0, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].x;
			// goal_mat(1, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].y;
			// goal_mat(2, 3) = PCTrajNode::PCMap_fiter_rho_vis->points[end_idx].z;
	
			start_mat(0, 3) = searchPoint.x;
			start_mat(1, 3) = searchPoint.y;
			start_mat(2, 3) = searchPoint.z;
//哎 没用下面的
			// Vector2d tmp_goal_yaw(goal_mat(0,0),goal_mat(1,1));
			// Vector2d tmp_start_end((goal_mat(0, 3)-start_mat(0, 3)),(goal_mat(1, 3)-start_mat(1, 3)));
			// double tmp_cos=tmp_goal_yaw.dot(tmp_start_end);
			// if(tmp_cos<0)
			// {
			// 	goal_mat(0,0)=-goal_mat(0,0);
			// 	goal_mat(1,1)=-goal_mat(1,1);
			// }
//哎 没用上面的 


double xR_dot_xW=Eigen::Vector3d(-1,-1,0).dot(Eigen::Vector3d(1,0,0));
double yR_dot_xW=Eigen::Vector3d(1,-1,0).dot(Eigen::Vector3d(1,0,0));
goal_mat(0, 0)=xR_dot_xW;
goal_mat(0, 1)=yR_dot_xW;
double xR_dot_yW=Eigen::Vector3d(-1,-1,0).dot(Eigen::Vector3d(0,1,0));
double yR_dot_yW=Eigen::Vector3d(1,-1,0).dot(Eigen::Vector3d(0,1,0));
goal_mat(1, 0)=xR_dot_yW;
goal_mat(1, 1)=yR_dot_yW;


			goal_mat(0, 3) = end_pt.x;
			goal_mat(1, 3) = end_pt.y;
			goal_mat(2, 3) = end_pt.z;

			cout<<"start_mat "<<start_mat(0, 3)<<"  "<<start_mat(1, 3)<<"  "<<start_mat(2, 3)<<endl;
			cout<<"goal_mat "<<goal_mat(0, 3)<<"  "<<goal_mat(1, 3)<<"  "<<goal_mat(2, 3)<<endl;
		 PCTrajNode pcl_goal(goal_mat), pcl_start(start_mat);
			//std::cout<<"mapReceived optimized_path"<<std::endl;





			Vector3d pos(pcl_goal.T.topRightCorner(3, 1));
			Vector3d grad_tmp;grad_tmp(0)=pcl_goal.T(0,2);grad_tmp(1)=pcl_goal.T(1,2);grad_tmp(2)=pcl_goal.T(2,2);

			// Vector3d pos(goal_mat(0, 3),goal_mat(1, 3),goal_mat(2, 3));

			// pcl::PointXYZI tmp;tmp.x=pos(0);tmp.y=pos(1);tmp.z=pos(2);
			// tmp.intensity=-1;//PCMap_fiter_rho_vis  kdtree_filter
			// Vector3d grad_tmp=PCTrajNode::calculate_rho_and_normal(tmp,PCTrajNode::PCMap,kdtree);
			// double cos_vis=grad_tmp.dot(Vector3d(0,0,1))/grad_tmp.norm();
			// 	if (cos_vis<0)//如果不同向
			// {
			// 	grad_tmp=-grad_tmp;
			// }   
			// cos_vis=grad_tmp.dot(Vector3d(0,0,1))/grad_tmp.norm();
			// //std::cout<<"lcoal cos_vis "<<cos_vis<<std::endl;
			publishSamplePoint_2(pos);
			vis_grad(pos,grad_tmp,1);

			if (!pcl_goal.isTraversable())
			{
				ROS_WARN("goal is not traversable");
				return;
			}

			optimized_path(pcl_start, pcl_goal);
			// visualizeTrees();
			

	}
	else
	{
		ROS_WARN("No map received yet! Can't plan now.");
	}
}

//calculate ka, theta
static pair<double, double> get_direct(Vector2d v1, Vector2d v3)
{
	double d1 = -(v1[0] * v1[0] + v1[1] * v1[1]);
	double d2 = v3[0] * v3[0] + v3[1] * v3[1];
	double fm = 2 * (v3[0] * v1[1] - v3[1] * v1[0]);
	double ka,theta;
	//fm=0就是快共线 这里挂的nan 
	if(fm<10e-5)
	{
		ka=0;
		theta=atan2(v3[1]-v1[1], v3[0]-v1[0]);
		return  { ka,theta };
	}
	Vector2d o((v3[1] * d1 + v1[1] * d2) / fm, (-v1[0] * d2 - v3[0] * d1) / fm);
	ka = 1.0 / o.norm();
	if(!pcl_isfinite(o.norm()))
	{
		//std::cout<<"v1 "<<v1<<std::endl;
		//std::cout<<"v3 "<<v3<<std::endl;
		//std::cout<<"d1 "<<d1<<std::endl;
		//std::cout<<"d2 "<<d2<<std::endl;
		//std::cout<<"fm"<<fm<<std::endl;
		//std::cout<<"(v3[1] * d1 + v1[1] * d2) / fm "<<(v3[1] * d1 + v1[1] * d2) / fm<<std::endl;
		//std::cout<<"(-v1[0] * d2 - v3[0] * d1) / fm "<<(-v1[0] * d2 - v3[0] * d1) / fm<<std::endl;
		//std::cout<<"o.norm() "<<o.norm()<<std::endl;
	}

	theta = atan2(o[1], o[0]);
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
{//actually rrt connect
	// initialization
	ros::Time t1 = ros::Time::now();
	update_cost_time_all=0;
	find_nearest_time_all=0;
	get_extend_time_all=0;
	connect_time_all=0;
	sample_time_all=0;
	sample_rrt_time_all=0;
	vector<RRT_Node> path = refined_path(start, goal);
	ROS_WARN("all_plan cost_before_local = %lf ms", (ros::Time::now() - t1).toSec() * 1000);
	ROS_ERROR(" in localoptimize");
	vector<PCTrajNode> p;
	if(path.empty())
	{
		inProcess = false;
		ROS_ERROR(" RRT* faileddddddddddddddddddddd");
				p.clear();
				Graph.clear();
				return p;
	}
	for (int i = path.size() - 1; i>1; i--)
	{
		// cout<<"-----------------------"<<endl;
		PCTrajNode from_node = path[i].node;
		PCTrajNode to_node = path[i -1].node;
		VectorXd par = PCTrajNode::get_connect(from_node, to_node);
		// //std::cout<<"						index   "<<i<<std::endl;
		// //std::cout<<"par[4]				"<<par[4]<<std::endl;
		// //std::cout<<"get_cost "<<from_node.get_cost()<<std::endl;
		// //cout << from_node.get_pos()[0] << " " << from_node.get_pos()[1] << " " << from_node.get_pos()[2] << endl;
		// //cout << to_node.get_pos()[0] << " " << to_node.get_pos()[1] << " " << to_node.get_pos()[2] << endl;
	}
	bool op_down = false;
	vector<RRT_Node>::reverse_iterator riter;
	for (riter = path.rbegin(); riter != path.rend(); riter++)
	{
		p.push_back((*riter).node);
	}

    ROS_WARN("all_plan cost = %lf ms", (ros::Time::now() - t1).toSec() * 1000);
	// cout<<"update_cost_time_all "<<update_cost_time_all<<endl;
	cout<<"find_nearest_time_all "<<find_nearest_time_all<<endl;
	// cout<<"get_extend_time_all "<<get_extend_time_all<<endl;
	// cout<<"connect_time_all "<<connect_time_all<<endl;
	// cout<<"sample_time_all "<<sample_time_all<<endl;
	cout<<"	sample_rrt_time_all"<<	sample_rrt_time_all<<endl;
	// for (size_t i = 0; i < p.size() - 1; i++)
	// {
	// 	cout<<"-----------------------"<<endl;
	// 	PCTrajNode from_node = p[i];
	// 	PCTrajNode to_node = p[i + 1];
	// 	VectorXd par = PCTrajNode::get_connect(from_node, to_node);
	// 	//std::cout<<"						index   "<<i<<std::endl;
	// 	//std::cout<<"par[4]				"<<par[4]<<std::endl;
	// 	//cout << from_node.get_pos()[0] << " " << from_node.get_pos()[1] << " " << from_node.get_pos()[2] << endl;
	// 	//cout << to_node.get_pos()[0] << " " << to_node.get_pos()[1] << " " << to_node.get_pos()[2] << endl;
	// }

	// return p;
	
	vector<double> del(p.size(), PCTrajNode::config.local_param.delta_max);
	double bh = PCTrajNode::config.local_param.delta_min;
	int cnt = 0;
	cout<<"begin optimization!----------------------------------------------------------"<<endl;



	pcl::StopWatch time;
	do
	{
		// check distance between nodes
		cout << "optimization: " << ++cnt << endl;
		//cout << "check distance between nodes" << endl;
		auto id = del.begin();
		for (auto i= p.begin(); i != p.end()-1; i++, id++)
		{
			double dist = (*i).get_dist(*(i+1));
			//cout << "dist"<<dist << endl;
			if (dist > PCTrajNode::config.local_param.din_max)
			{
				if (i + 1 != p.end() - 1)
				{
					//cout << "add nodes" << endl;
					VectorXd pat = PCTrajNode::get_connect(*i, *(i + 1));//k a b c sf
					if ( pat[4]==INFINITY || pat[4]==0)//pat[4]==0
					{
						//cout << "--------------------add nodes get_connect infinity" << endl;
						continue;
					}


					// Vector4d path_seg;// = forward(init, x);
					// double seg = pat(4)/2;
					// double seg2 = seg*seg;
					// double seg3 = seg*seg2;
					// double seg4 = seg*seg3;
					// double seg5 = seg*seg4;
					// path_seg[0] = seg;
					// path_seg[1] = pat(0)*seg2/2+ pat(1)*seg3/6+pat(2)*seg4/12+pat(3)*seg5/20;
					// path_seg[2] = pat(0)*seg+ pat(1)*seg2/2+pat(2)*seg3/3+pat(3)*seg4/4;
					// path_seg[3] = pat(0) + pat(1)*seg+pat(2)*seg2+pat(3)*seg3;
					// //cout << "path_seg " <<path_seg<< endl;

					// Matrix4d mid_T = (*i).Trans(path_seg.head(3));
					// PCTrajNode mid_node(mid_T, path_seg[3]);
					

					double ds=pat(4)/2;double k0=pat(0);double a=pat(1);double b=pat(2);double c=pat(3);
					double theta= k0*ds+a*ds*ds/2+b*ds*ds*ds/3+c*ds*ds*ds*ds/4 ;
					double kf= k0 + a*ds+b*ds*ds+c*ds*ds*ds;
					Vector2d result= forward(0,0,0,k0,a,b,c,ds);
					double y =result[1];//y
					double x =result[0];//x
					Vector3d tmp_trans(x,y,theta);
					//cout << "tmp_trans " <<tmp_trans<< endl;
					Matrix4d mid_T = (*i).Trans(tmp_trans);
					PCTrajNode mid_node(mid_T, kf);

					
					if(!mid_node.isTraversable()){
						//cout << "--------------------add nodes mid_node.is not Traversable()"<< endl;
						continue;
					}
					double cost=PCTrajNode::get_connect((*i), mid_node)[4];
					if(cost==INFINITY){
						//cout << "--------------------add nodes cost==INFINITY"<< endl;
							continue;
					}
					cost=PCTrajNode::get_connect(mid_node,(*(i+1)))[4];
					if(cost==INFINITY){
						//cout << "--------------------add nodes cost==INFINITY"<< endl;
							continue;
					}
					i = p.insert(i + 1, mid_node);
					i--;
					id = del.insert(id + 1, *id);
					id--;
				}
			}
			else if (dist < PCTrajNode::config.local_param.din_min)
			{
						// //cout << "(*i)"<< (*i).T(0,3)<<" "<< (*i).T(1,3)<<" "<< (*i).T(2,3)<<endl;
						// //cout << "(*(i+1))"<< (*(i+1)).T(0,3)<<" "<< (*(i+1)).T(1,3)<<" "<< (*(i+1)).T(2,3)<<endl;
						// //cout << "(*(i+2))"<< (*(i+2)).T(0,3)<<" "<< (*(i+2)).T(1,3)<<" "<< (*(i+2)).T(2,3)<<endl;
						// //cout << "(*(i+3))"<< (*(i+3)).T(0,3)<<" "<< (*(i+3)).T(1,3)<<" "<< (*(i+3)).T(2,3)<<endl;
				if (i + 1 != p.end() - 1 && i + 2 != p.end() - 1)
				{
					bool N1_N3_yes=true;
					bool N1_mid_yes=true;
					bool mid_N3_yes=true;
					VectorXd pat = PCTrajNode::get_connect(*i, *(i + 3));//k a b c sf

					if (pat[4]==INFINITY|| pat[4]==0)//pat[4]==0 
					{
						N1_N3_yes=false;
						//cout << "--------------------N1_N3_yes=false"<< endl;
					}

					MatrixXd T_b = (*i).T.inverse()*((*(i+3)).T);
					double xf = T_b.row(0)[3];
					double yf = T_b.row(1)[3];
					double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);
					Vector3d tmp_trans(xf/2,yf/2,thetaf/2);

					Matrix4d mid_T = (*i).Trans(tmp_trans);
					PCTrajNode mid_node(mid_T);
					double cost=PCTrajNode::get_connect((*i), mid_node)[4];
					if(cost==INFINITY){
						N1_mid_yes=false;	
						//cout << "--------------------N1_mid_yes=false"<< endl;
					}
					cost=PCTrajNode::get_connect(mid_node,(*(i+3)))[4];
					if(cost==INFINITY){
						mid_N3_yes=false;	
						//cout << "--------------------mid_N3_yes=false"<< endl;
					}

					if((N1_N3_yes==false) && ((N1_mid_yes==false) || (mid_N3_yes==false))){
						//cout << "--------------------dont earse and insert do nothing"<< endl;
						
					}
					if((N1_N3_yes==true) && ((N1_mid_yes==false) || (mid_N3_yes==false)))
					{//其实还不太能直接连,因为可能长度太长,虽然两端角度合适 有解,但中间的点却会不可通行,导致后面add node的时候挂掉
						if(pat[4]>PCTrajNode::config.local_param.din_max)
						{
							Vector4d path_seg;
							double seg = pat(4)/2;
							double seg2 = seg*seg;
							double seg3 = seg*seg2;
							double seg4 = seg*seg3;
							double seg5 = seg*seg4;
							path_seg[0] = seg;
							path_seg[1] = pat(0)*seg2/2+ pat(1)*seg3/6+pat(2)*seg4/12+pat(3)*seg5/20;
							path_seg[2] = pat(0)*seg+ pat(1)*seg2/2+pat(2)*seg3/3+pat(3)*seg4/4;
							path_seg[3] = pat(0) + pat(1)*seg+pat(2)*seg2+pat(3)*seg3;
							//cout << "path_seg " <<path_seg<< endl;

							Matrix4d mid_T_tmp = (*i).Trans(path_seg.head(3));
							PCTrajNode mid_node_tmp(mid_T_tmp, path_seg[3]);
							if(!mid_node.isTraversable())
								cout << "N1_N3_yes,N1_mid or mid_N3 failed, but N1_N3 too long ,and add node failed, do nothing"<< endl;
							else{
								//cout << "N1_N3_yes,N1_mid or mid_N3 failed"<< endl;
								//cout << "--------------------just earse"<< endl;
								id = del.erase(id + 1);
								i = p.erase(i + 1);
								id = del.erase(id);
								i = p.erase(i);
								id--;
								i--;
							}
						}
						else{
							//cout << "N1_N3_yes,N1_mid or mid_N3 failed"<< endl;
							//cout << "--------------------just earse"<< endl;
							id = del.erase(id + 1);
							i = p.erase(i + 1);
							id = del.erase(id);
							i = p.erase(i);
							id--;
							i--;
						}						
					}
					if((N1_mid_yes==true) && (mid_N3_yes==true)){
						//cout << "-------------------- earse and insert"<< endl;
						id = del.erase(id + 1);
						i = p.erase(i + 1);
						id = del.erase(id);
						i = p.erase(i);
						id--;
						i--;
						
						i = p.insert(i + 1, mid_node);
						i--;
						id = del.insert(id + 1, *id);
						id--;
						
					}
				}
				// if (i + 1 != p.end() - 1 && i + 2 != p.end() - 1)
				// {
				// 		//cout << "(*i)"<< (*i).T(0,3)<<" "<< (*i).T(1,3)<<" "<< (*i).T(2,3)<<endl;
				// 		//cout << "(*(i+1))"<< (*(i+1)).T(0,3)<<" "<< (*(i+1)).T(1,3)<<" "<< (*(i+1)).T(2,3)<<endl;
				// 		//cout << "(*(i+2))"<< (*(i+2)).T(0,3)<<" "<< (*(i+2)).T(1,3)<<" "<< (*(i+2)).T(2,3)<<endl;
				// 		//cout << "(*(i+3))"<< (*(i+3)).T(0,3)<<" "<< (*(i+3)).T(1,3)<<" "<< (*(i+3)).T(2,3)<<endl;
				// 	//cout << "--------------------erase nodes" << endl;
				// 	id = del.erase(id + 1);
				// 	i = p.erase(i + 1);
				// 	//cout << "(*i)"<< (*i).T(0,3)<<" "<< (*i).T(1,3)<<" "<< (*i).T(2,3)<<endl;
				// 	id = del.erase(id);
				// 	i = p.erase(i);
				// 	//cout << "(*i)"<< (*i).T(0,3)<<" "<< (*i).T(1,3)<<" "<< (*i).T(2,3)<<endl;
				// 	id--;
				// 	i--;
				// 	//cout << "(*i)"<< (*i).T(0,3)<<" "<< (*i).T(1,3)<<" "<< (*i).T(2,3)<<endl;
				// 	//cout << "(*(i+1))"<< (*(i+1)).T(0,3)<<" "<< (*(i+1)).T(1,3)<<" "<< (*(i+1)).T(2,3)<<endl;

				// 	VectorXd pat = PCTrajNode::get_connect(*i, *(i + 1));//k a b c sf
				// 	Vector4d path_seg;// = forward(init, x);
				// 	double dist = (*i).get_dist((*(i+1)));
				// 	//cout << "dist 13 "<<dist << endl;

				// 	if (pat[4]==INFINITY|| pat[4]==0)//pat[4]==0 
				// 	{
				// 		//cout << "--------------------erase nodes at[4]==INFINITY|| pat[4]==0"<< endl;
				// 		continue;
				// 	}

				// 	// double seg = pat(4)/2;
				// 	// double seg2 = seg*seg;
				// 	// double seg3 = seg*seg2;
				// 	// double seg4 = seg*seg3;
				// 	// double seg5 = seg*seg4;
				// 	// path_seg[0] = seg;
				// 	// path_seg[1] = pat(0)*seg2/2+ pat(1)*seg3/6+pat(2)*seg4/12+pat(3)*seg5/20;
				// 	// path_seg[2] = pat(0)*seg+ pat(1)*seg2/2+pat(2)*seg3/3+pat(3)*seg4/4;
				// 	// path_seg[3] = pat(0) + pat(1)*seg+pat(2)*seg2+pat(3)*seg3;
				// 	// //cout << "path_seg " <<path_seg<< endl;

				// 	// Matrix4d mid_T = (*i).Trans(path_seg.head(3));
				// 	// PCTrajNode mid_node(mid_T, path_seg[3]);//x y theta

				// 	// double ds=pat(4)/2;double k0=pat(0);double a=pat(1);double b=pat(2);double c=pat(3);
				// 	// double theta= k0*ds+a*ds*ds/2+b*ds*ds*ds/3+c*ds*ds*ds*ds/4 ;
				// 	// double kf= k0 + a*ds+b*ds*ds+c*ds*ds*ds;
				// 	// Vector2d result= forward(0,0,0,k0,a,b,c,ds);
				// 	// double y =result[1];//y
				// 	// double x =result[0];//x
				// 	// Vector3d tmp_trans(x,y,theta);
				// 	// //cout << "tmp_trans " <<tmp_trans<< endl;

				// 	MatrixXd T_b = (*i).T.inverse()*((*(i+1)).T);
				// 	double xf = T_b.row(0)[3];
				// 	double yf = T_b.row(1)[3];
				// 	double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);
				// 	Vector3d tmp_trans(xf/2,yf/2,thetaf/2);

				// 	Matrix4d mid_T = (*i).Trans(tmp_trans);
				// 	PCTrajNode mid_node(mid_T);


				// 	if(!mid_node.isTraversable())
				// 		continue;
				// 	double cost=PCTrajNode::get_connect((*i), mid_node)[4];
				// 	if(cost==INFINITY){
				// 		//cout << "(*i)"<< (*i).T(0,3)<<" "<< (*i).T(1,3)<<" "<< (*i).T(2,3)<<endl;
				// 		//cout << "(*(i+1))"<< (*(i+1)).T(0,3)<<" "<< (*(i+1)).T(1,3)<<" "<< (*(i+1)).T(2,3)<<endl;
				// 		//cout << "mid_node"<< mid_node.T(0,3)<<" "<< mid_node.T(1,3)<<" "<< mid_node.T(2,3)<<endl;
				// 		double dist = (*i).get_dist(mid_node);
				// 		//cout << "dist 12 "<<dist << endl;
				// 		dist = (mid_node).get_dist((*(i+1)));
				// 		//cout << "dist 23 "<<dist << endl;

				// 	Vector3d pt;
				// 	pt(0)=(*i).T(0,3);
				// 	pt(1)=(*i).T(1,3);
				// 	pt(2)=(*i).T(2,3);
					
				// 	publishSamplePoint(pt);
					
				// 	pt(0)=(*(i+1)).T(0,3);
				// 	pt(1)=(*(i+1)).T(1,3);
				// 	pt(2)=(*(i+1)).T(2,3);
					
				// 	publishSamplePoint_2(pt);
				// 	vector<Eigen::Vector3d> x_y_theta;
				// 	Eigen::Vector3d tmp(mid_node.T(0,3),mid_node.T(1,3),atan2(mid_node.T.row(1)[0], mid_node.T.row(0)[0]));
				// 	x_y_theta.push_back(tmp);
				// 	pub_csd(x_y_theta);
				// 		// pt(0)=mid_node.T(0,3);
				// 		// pt(1)=mid_node.T(1,3);
				// 		// pt(2)=mid_node.T(2,3);
				// 		// publishSamplePoint_2(pt);
						
				// 		//cout << "------------------1--erase nodes cost==INFINITY"<< endl;
				// 		// return p;
				// 			continue;
						
				// 	}
				// 	cost=PCTrajNode::get_connect(mid_node,(*(i+1)))[4];
				// 	if(cost==INFINITY){

				// 		//cout << "(*i)"<< (*i).T(0,3)<<" "<< (*i).T(1,3)<<" "<< (*i).T(2,3)<<endl;
				// 		//cout << "(*(i+1))"<< (*(i+1)).T(0,3)<<" "<< (*(i+1)).T(1,3)<<" "<< (*(i+1)).T(2,3)<<endl;
				// 		//cout << "mid_node"<< mid_node.T(0,3)<<" "<< mid_node.T(1,3)<<" "<< mid_node.T(2,3)<<endl;
				// 		double dist = (*i).get_dist(mid_node);
				// 		//cout << "dist 12 "<<dist << endl;

				// 		dist = (mid_node).get_dist((*(i+1)));
				// 		//cout << "dist 23 "<<dist << endl;
						
				// 	Vector3d pt;
				// 	pt(0)=(*i).T(0,3);
				// 	pt(1)=(*i).T(1,3);
				// 	pt(2)=(*i).T(2,3);
					
				// 	publishSamplePoint(pt);
					
				// 	pt(0)=(*(i+1)).T(0,3);
				// 	pt(1)=(*(i+1)).T(1,3);
				// 	pt(2)=(*(i+1)).T(2,3);
					
				// 	publishSamplePoint_2(pt);
				// 	vector<Eigen::Vector3d> x_y_theta;
				// 	Eigen::Vector3d tmp(mid_node.T(0,3),mid_node.T(1,3),atan2(mid_node.T.row(1)[0], mid_node.T.row(0)[0]));
				// 	x_y_theta.push_back(tmp);
				// 	pub_csd(x_y_theta);

				// 		//cout << "------------------2--erase nodes cost==INFINITY"<< endl;
				// 			continue;
				// 	}
				// 	i = p.insert(i + 1, mid_node);
				// 	i--;
				// 	id = del.insert(id + 1, *id);
				// 	id--;
				// }
			}
			(*i).connect(*(i + 1));
						// //cout << "(*i)"<< (*i).T(0,3)<<" "<< (*i).T(1,3)<<" "<< (*i).T(2,3)<<endl;
						// //cout << "(*(i+1))"<< (*(i+1)).T(0,3)<<" "<< (*(i+1)).T(1,3)<<" "<< (*(i+1)).T(2,3)<<endl;
						// //cout << "(*(i+2))"<< (*(i+2)).T(0,3)<<" "<< (*(i+2)).T(1,3)<<" "<< (*(i+2)).T(2,3)<<endl;
			if ((*i).t[4] == INFINITY)
			{
				ROS_ERROR("something wrong! check the path!")  ;
			}


			shadow temp;
			temp.s[0] = *i;
			if(!pcl_isfinite(temp.s[0].T(0,3)))
			{
				//cout << "-----------temp.s[0]-------------" << endl;
				cout<<temp.s[0].T<< endl;
			}
			temp.delta = *id;
			Graph.push_back(temp);
		}
		// deal with start and goal
		shadow temp;
		temp.s[0] = p.back();
		temp.s[1] = p.back();
		temp.s[2] = p.back();
		temp.delta = PCTrajNode::config.local_param.delta_min;
		Graph.push_back(temp);
		Graph[0].s[1] = Graph[0].s[2] = Graph[0].s[0];
		Graph[0].delta = PCTrajNode::config.local_param.delta_min;

		// separate

	//ocost=inf是work的？
	// //cout << "separate-----------------------------------------------------------" << endl;
	// for (size_t i = 0; i < Graph.size() - 1; i++)
	// {
	// 	PCTrajNode from_node = Graph[i].s[0];
	// 	PCTrajNode to_node = Graph[i + 1].s[0];
		
	// 	VectorXd par = PCTrajNode::get_connect(from_node, to_node);
	// 	//std::cout<<"						index   "<<i<<std::endl;
	// 	//std::cout<<"par[4]				"<<par[4]<<std::endl;

	// 	//cout << from_node.get_pos()[0] << " " << from_node.get_pos()[1] << " " << from_node.get_pos()[2] << endl;
	// 	//cout << to_node.get_pos()[0] << " " << to_node.get_pos()[1] << " " << to_node.get_pos()[2] << endl;
	// }
		separate();

	//ocost=inf是work的？
		// dijkstra
		//cout << "dijkstra" << endl;
		//之前的dijkstra会遇到的问题是原本那条中间的路就不work，delta加横向偏移量之后还不work，还都是inf的情况，根本搜不到终点去就退出掉了，然后挂掉
		//现在是搜不到终点就退出的话，就直接退出优化
		vector<int> opath = dijk();
		op_down = true;
		//std::cout<<"after dijk,traj.size()"<<opath.size()<<std::endl;
		bool dijk_failed=false;
		bool dijk_is_worked_and_result_is_center=true;
		if(opath.size()!=Graph.size())//扔进去dijk的path是add和或者erase之后的
		{
			op_down = false;
			dijk_failed=true;
			dijk_is_worked_and_result_is_center=false;
    		ROS_WARN("dijk can't find work path,even if the initial path is not work");
			if(cnt==1){
				ROS_ERROR("WHY the rrtstar can't work ,");
				p.clear();
				Graph.clear();
				return p;
			}
			
		}	
		p.clear();//挂
		//本来p是不清的,我发现有的时候p的个数和opath的个数不一致,讲道理比较奇怪,可能是dijk里面有问题,但是我强行把p清理了.其实应该del跟着动的,但测了下好像ok就先不管了
		// for (size_t i = 1; i < opath.size() - 1; i++)
		// for (size_t i = 1; i < opath.size(); i++)//挂
		for (size_t i = 0; i < opath.size(); i++)//挂
		{
			// //std::cout<<"opath[i] "<<opath[i]<<std::endl;
			// //std::cout<<"p1 "<<Graph[i].s[opath[i]].T(0,3)<<" "<<Graph[i].s[opath[i]].T(1,3)<<" "<<Graph[i].s[opath[i]].T(2,3)<<std::endl;
			if(dijk_failed==false)//dijk_is_worked
			{
				// if (opath[i] != 0)
				// {
					if (del[i] > PCTrajNode::config.local_param.delta_min)
						del[i] -= 0.01;
					op_down = false;
					dijk_is_worked_and_result_is_center=false;
					// p[i] = Graph[i].s[opath[i]];//挂
					p.push_back(Graph[i].s[opath[i]]);
					if(!pcl_isfinite(p[i].T(0,3)))
					{
						
						cout<<p[i].T<< endl;
					}
				// }

			}
			else{//dijk如果失败的话就还是按照上一回的中心位置来，然后继续优化
				//std::cout<<"dijk failed "<<std::endl;
				dijk_is_worked_and_result_is_center=false;
				// p[i] = Graph[i].s[0];
				p.push_back(Graph[i].s[0]);
				if (del[i] > PCTrajNode::config.local_param.delta_min)
					del[i] -= 0.01;
			}
			// //std::cout<<"p2 "<<p[i].T(0,3)<<" "<<p[i].T(1,3)<<" "<<p[i].T(2,3)<<std::endl;
		}
		for(int i = 0; i < p.size(); i++)
		{
			// //std::cout<<"p3 "<<p[i].T(0,3)<<" "<<p[i].T(1,3)<<" "<<p[i].T(2,3)<<std::endl;
		}
		if(dijk_is_worked_and_result_is_center)
		{
			//cout << "优化完都处于初始中心位置" << endl;
			op_down=true;
		}
			
		Graph.clear();
		if (cnt>=10)
			break;
	} while (!op_down && any_of(del.begin(), del.end(), [bh](double i) {return i > bh; }));
	//如果不是优化完都处于初始中心位置（即还可以通过小偏移而减小cost）   且  每个del都大于最小的dela_min，则继续优化
	cout<<"optimization done, iteration = "<<cnt<<"      time consume = "<<time.getTime()/1000<<"s"<<endl;
	plan_time<<(ros::Time::now() - t1).toSec() * 1000<<std::endl;
	plan_time_all+=(ros::Time::now() - t1).toSec() * 1000;
	visualizePath(p);
    ROS_WARN("all_plan cost = %lf ms", (ros::Time::now() - t1).toSec() * 1000);

	draw_path_line(p);
	// draw_path_by_intergrate(p);
	Graph.clear();
	return p;
}

void LocalOptimizer::draw_path_by_intergrate(vector<PCTrajNode> p){
		int i=0;
		vector<Eigen::Vector3d> x_y_theta_all;
		//std::cout<<"p.size()"<<p.size()<<std::endl;
		Eigen::Vector3d x_y_theta;double x;double y;double theta;Vector2d result;
			for (auto node : p)
		{
			double x0=node.T(0,3);
			double y0=node.T(1,3);
			double theta0=atan2(node.T(1,0),node.T(0,0));
			double k0=node.t[0];
			double a=node.t[1];
			double b=node.t[2];
			double c=node.t[3];
			double sf=node.t[4];	
			// //std::cout<<"a "<<a<<std::endl;
			// //std::cout<<"sf "<<sf<<std::endl;
			for(double ds=0;ds<sf;ds+=0.01)
			{
				theta= theta0+k0*ds+a*ds*ds/2+b*ds*ds*ds/3+c*ds*ds*ds*ds/4 ;
				result= forward(x0,y0,theta0,k0,a,b,c,ds);
				y =result[1];//y
				x =result[0];//x
				x_y_theta(0)=x;x_y_theta(1)=y;x_y_theta(2)=theta;
				x_y_theta_all.push_back(x_y_theta);
			}
			i++;
		}
		pub_csd(x_y_theta_all);
}	
void LocalOptimizer::pub_csd(vector<Eigen::Vector3d> x_y_theta)
{
    nav_msgs::Odometry pst;
    pst.header.frame_id= "map";
    //std::cout<<"x_y_theta.size()"<<x_y_theta.size()<<std::endl;
    for (size_t i = 0; i < x_y_theta.size(); i++)
    {
        pst.header.stamp = ros::Time::now();
    	// //std::cout<<"			x"<< x_y_theta[i](0) <<std::endl;
	    // //std::cout<<"			        y"<< x_y_theta[i](1) <<std::endl;
        pst.pose.pose.position.x = x_y_theta[i](0);
        pst.pose.pose.position.y = x_y_theta[i](1);
        pst.pose.pose.position.z = 0.7;
        Eigen::AngleAxisd t_v(x_y_theta[i](2),Vector3d(0,0,1));
        Eigen::Quaterniond rot(t_v);
        pst.pose.pose.orientation.w=rot.w();
        pst.pose.pose.orientation.x=rot.x();
        pst.pose.pose.orientation.y=rot.y();
        pst.pose.pose.orientation.z=rot.z();
        csd_pub.publish(pst);
        ros::Duration(0.005).sleep();
    }

}
static double fx(double s, void * params) {
	double* alpha = (double *)params;
	double a = alpha[0];
	double b = alpha[1];
	double c = alpha[2];
	double kappa = alpha[3];
	double theta = alpha[4];
	double f = theta + kappa * s + a * s*s / 2 + b * pow(s, 3) / 3 + c * pow(s, 4) / 4;

	return cos(f);
}

// integrate function for gsl
static double fy(double s, void * params) {
	double* alpha = (double *)params;
	double a = alpha[0];
	double b = alpha[1];
	double c = alpha[2];
	double kappa = alpha[3];
	double theta = alpha[4];
	double f = theta + kappa * s + a * s*s / 2 + b * pow(s, 3) / 3 + c * pow(s, 4) / 4;

	return sin(f);
}

Vector2d LocalOptimizer::forward(double x0,double y0,double theta0,double k0,double a, double b,double c,double s_)
{

	Vector2d y = Vector2d::Zero();
	double result, error;
	double alpha[5];

	alpha[0] = a; alpha[1] = b; alpha[2] = c; alpha[3] = k0; alpha[4] = theta0;

	gsl_function F;

	gsl_integration_workspace * wx= gsl_integration_workspace_alloc(1000);
	gsl_integration_workspace * wy= gsl_integration_workspace_alloc(1000);
	
	// integrate x
	F.function = &fx;
	F.params = (void*)alpha;

	//ab是积分上下界 ,按照&fx： 把cos(theta(s))从a积到b的值写到result
	// fuction_f, a , b , epsabs , epsrel , limit , * workspace , * result , * abserr ) 
	int state = gsl_integration_qags(&F, 0, s_, 0, 1e-7, 1e-7,  \
		wx, &result, &error);
	if (state==GSL_ESING)
	{
		//std::cout<<"state==GSL_ESING x"<<std::endl;
		y(0) = std::numeric_limits<double>::infinity();
		y(1) = 0;
		gsl_integration_workspace_free(wx);
		gsl_integration_workspace_free(wy);
		return y;
	}
	

	y(0) = x0 + result ;

	F.function = &fy;
	state = gsl_integration_qags(&F, 0, s_, 0, 1e-7, 1e-7,  \
		wx, &result, &error);
	if (state==GSL_ESING)
	{
		//std::cout<<"state==GSL_ESING y"<<std::endl;
		y(0) = std::numeric_limits<double>::infinity();
		y(1) = 0;
		gsl_integration_workspace_free(wy);
		gsl_integration_workspace_free(wy);
		return y;
	}
	y(1) = y0 + result ;
	gsl_integration_workspace_free(wx);
	gsl_integration_workspace_free(wy);

	return y;
}

void LocalOptimizer::separate(void)
{
	// generate left and right subnodes
	//std::cout<<"              Graph.size()"<<Graph.size()<<std::endl;
	for (size_t i = 1; i < Graph.size() - 1; i++)
	{
		shadow& sh = Graph[i];
		Matrix4d left = sh.s[0].Trans({ 0,-sh.delta,0 });
		Matrix4d right = sh.s[0].Trans({ 0,sh.delta,0 });
		//可以解释了，sh.s[0].T 是旋转那里是nan，因此trans之后会导致平移的也nan
		//然后导致new一个新node的时候searchpoint输入的xyz是nan就挂掉
		//至于为什么会出现旋转nan,应该是之前算方向角，gd.second算成nan了
			if(!pcl_isfinite(sh.s[0].T(3,0)))
			{
				//std::cout<<"sh.s[0].T 旋转挂掉"<<sh.s[0].T<<std::endl;
				//std::cout<<"left"<<left<<std::endl;
			}

		sh.s[1] = PCTrajNode(left);
		sh.s[2] = PCTrajNode(right);

	}
	
	// special node, 1
	shadow& sh = Graph[1];
	//sh.s[1]现在是N1的左子结点，get_direct是在求这个左子结点的曲率和theta
	//论文里说的是求三组然后均值，这里之所以不均是因为是起始点和终止点
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
			//是这里gd.second算出来一个nan导致graph里有节点的旋转是nan
			if(!pcl_isfinite(sh.s[0].T(0,0)))
			{
					//std::cout<<"000 test graph		i"<<"   k=0"<<std::endl;
					//std::cout<<"      sh.s[0].T"<<std::endl;
					//std::cout<<sh.s[0].T<<std::endl;
			}
			if(!pcl_isfinite(sh.s[k].T(0,0)))
			{
					//std::cout<<"gd.second"<<gd.second<<std::endl;
					//std::cout<<"111 test graph		i"<<i<<"  k "<<k<<std::endl;
					//std::cout<<"      sh.s[k].T"<<std::endl;
					//std::cout<<sh.s[k].T<<std::endl;
			}

			sh.s[k].change_T({ 0,0,gd.second });
			sh.s[k].kappa = gd.first;

			if(!pcl_isfinite(sh.s[k].T(0,0)))
			{		
					//std::cout<<"gd.second"<<gd.second<<std::endl;
					//std::cout<<"222 test graph		i"<<i<<"  k "<<k<<std::endl;
					//std::cout<<"      sh.s[k].T"<<std::endl;
					//std::cout<<sh.s[k].T<<std::endl;
			}

		}
	}

}

double LocalOptimizer::get_op_cost(size_t index, int from, int to)
{
	PCTrajNode from_node = Graph[index].s[from];
	PCTrajNode to_node = Graph[index + 1].s[to];
	VectorXd par = PCTrajNode::get_connect(from_node, to_node);
	if (par == VectorXd::Zero(5)|| par[4]==INFINITY )
	{
		//cout << "get_op_cost cubic error" << endl;
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
		//cout << "get_op_cost not traverse" << endl;
		return std::numeric_limits<double>::infinity();
	}

	// get kappa
	double kappa = 0;
	// for (double dist = 0.0; dist < par[4]; dist += 0.01)
	// {
	// 	double kap = 0;
	// 	for (int j = 3; j >= 0; j--)
	// 	{
	// 		kap *= dist;
	// 		kap += par[j];
	// 	}
	// 	if (kap > PCTrajNode::PR.kappa_max)
	// 	{
	// 		//cout << "get_op_cost kap big"<< endl;
	// 		return std::numeric_limits<double>::infinity();
	// 	}
	// 	else if (kap > kappa)
	// 	{
	// 		kappa = kap;
	// 	}
	// }

	// get dist
	double d = Graph[index].s[from].get_dist(Graph[index + 1].s[to]);
	// cout<<"d  "<<d<<endl;
	d = (d - PCTrajNode::config.local_param.din_min) / (PCTrajNode::config.local_param.din_max - PCTrajNode::config.local_param.din_min);
	// cout<<"d_after  "<<d<<endl;
	return PCTrajNode::config.local_param.w_len * d + PCTrajNode::config.local_param.w_trav * (1 - totau) + PCTrajNode::config.local_param.w_curv * kappa / PCTrajNode::PR.kappa_max;
}


vector<int> LocalOptimizer::dijk(void)
{	
	ros::Time time_1 = ros::Time::now();

	vector<int> result;
	double ocost = 0;
	//cout << "-----------------Graph.size: " << Graph.size()<< endl;	
	for (size_t i = 0; i < Graph.size() - 1; i++)
	{
		ocost += get_op_cost(i, 0, 0);
	}
	//cout << "-------------------old cost=" << ocost << endl;
	//ocost=inf是work的？



	djk_nodePtr ** djk_nodeMap;
	
	djk_nodeMap= new djk_nodePtr * [Graph.size()];
	//init 
	djk_nodeMap[0] = new djk_nodePtr[1];
	djk_nodeMap[0][0]=new djk_node(0,0);
	for (size_t i = 1; i < Graph.size()-1; i++)
	{
		djk_nodeMap[i]=new djk_nodePtr[3];
		for (int j = 0; j < 3; j++)
		{
			djk_nodeMap[i][j]=new djk_node(i, j);
		}
	}
	djk_nodeMap[Graph.size()-1] = new djk_nodePtr[1];
	djk_nodeMap[Graph.size()-1][0]=new djk_node(Graph.size()-1,0);

	std::multimap<double, djk_nodePtr> openSet;
   	djk_nodePtr startPtr= djk_nodeMap[0][0];
   	djk_nodePtr endPtr= djk_nodeMap[Graph.size()-1][0];
	openSet.clear();
	djk_nodePtr currentPtr  = NULL;
    djk_nodePtr neighborPtr = NULL;
	djk_nodePtr terminatePtr = NULL;
    startPtr -> fScore = 0;
	startPtr -> state = 1;
	openSet.insert( make_pair(startPtr -> fScore, startPtr) );
	vector<djk_nodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
	while (!openSet.empty())
	{
		currentPtr=openSet.begin()->second;
        currentPtr->state=-1;
        openSet.erase(openSet.begin());
		// //std::cout<<"			currentPtr->id_in_graph "<<currentPtr->id_in_graph<<std::endl;
		// //std::cout<<"			currentPtr->id_in_node "<<currentPtr->id_in_node<<std::endl;
		if( currentPtr->id_in_graph == Graph.size()-1 ){
			//std::cout<<"--------------------actual find end"<<std::endl;
			break;
		}

		neighborPtrSets.clear();
		edgeCostSets.clear();
		//get neighbor:next node 
		if(currentPtr->id_in_graph<Graph.size()-2)
		{
			for (int i = 0; i < 3; i++)
			{

				djk_nodePtr neighborPtr = djk_nodeMap[currentPtr->id_in_graph+1][i];

				//get_op_cost 输入的是 在graph里的node的id ，from这个node的左位还是右位还是中位，to 左中右
				double distance=get_op_cost(currentPtr->id_in_graph, currentPtr->id_in_node, i);
				if(distance<1e7)
				{
					neighborPtrSets.push_back(neighborPtr);
					edgeCostSets.push_back(distance);
				}

			}
		}
		else if(currentPtr->id_in_graph==Graph.size()-2)//倒数第二个点
		{
			djk_nodePtr neighborPtr = djk_nodeMap[currentPtr->id_in_graph+1][0];
			double distance=get_op_cost(currentPtr->id_in_graph, currentPtr->id_in_node, 0);
			if(distance<1e7)
			{
				neighborPtrSets.push_back(neighborPtr);
				edgeCostSets.push_back(distance);
			}
		}
		for(int i = 0; i < (int)neighborPtrSets.size(); i++)
		{
			neighborPtr=neighborPtrSets[i];
			// //std::cout<<"neighborPtr->id_in_graph "<<neighborPtr->id_in_graph<<std::endl;
			// //std::cout<<"neighborPtr->id_in_node "<<neighborPtr->id_in_node<<std::endl;
			if(neighborPtr -> state ==0){//如果这个neighbor和还没有被访问，那就把它pushback到openset里面
				neighborPtr-> fScore=currentPtr-> fScore+edgeCostSets[i];
				neighborPtr->cameFrom=currentPtr;
				neighborPtr -> state =1;
				openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
				continue;
			}
			else 
			{
				if(neighborPtr-> fScore  >  currentPtr->fScore+edgeCostSets[i])
				{
				neighborPtr-> fScore=currentPtr->fScore+edgeCostSets[i];
				neighborPtr->cameFrom=currentPtr;
				}
				continue;
			}
		}


	}
	djk_nodePtr stoptr=currentPtr;
	//要么是找不到终点就退出了，要么是到终点了
	//std::cout<<"			finally "<<std::endl;
	//std::cout<<"			currentPtr->id_in_graph "<<currentPtr->id_in_graph<<std::endl;
	//std::cout<<"			currentPtr->id_in_node "<<currentPtr->id_in_node<<std::endl;
	while (stoptr->fScore!=0)
	{ 	
		result.push_back(stoptr->id_in_node);
		stoptr=stoptr->cameFrom;
	}
	result.push_back(0);
	reverse(result.begin(), result.end());

	if (ocost>1e7)
	{
		for (auto p:result)
			p=0;
	}

	ros::Time time_2 = ros::Time::now();
    // ROS_WARN("Time consume in dijk path finding is %f", (time_2 - time_1).toSec() );
	return result;
}

// vector<int> LocalOptimizer::dijk(void)
// {
// 	ros::Time time_1 = ros::Time::now();
// 	vector<int> result;
// 	vector<PCTrajNode> map;
// 	vector<bool> collected;
// 	vector<int> parent;
// 	vector<double> dist;
// 	//cout << "Graph.size: " << Graph.size()<< endl;	
// 	// //cout << "end 0" << Graph[Graph.size()-1].s[0].T<< endl;	
// 	// //cout << "end 1" << Graph[Graph.size()-1].s[1].T<< endl;	
// 	// //cout << "end 2" << Graph[Graph.size()-1].s[2].T<< endl;	
// 	map.push_back(Graph[0].s[0]);
// 	collected.push_back(false);
// 	parent.push_back(-1);
// 	dist.push_back(0);
// 	for (size_t i = 1; i < Graph.size()-1; i++)
// 	{
// 		for (int j = 0; j < 3; j++)
// 		{
// 			map.push_back(Graph[i].s[j]);
// 			collected.push_back(false);
// 			parent.push_back(-1);
// 			dist.push_back(numeric_limits<double>::infinity());
// 		}
// 	}
// 	map.push_back(Graph[Graph.size()-1].s[0]);
// 	collected.push_back(false);
// 	parent.push_back(-1);
// 	dist.push_back(numeric_limits<double>::infinity());
// 	//cout << "map graph.size: " << map.size()<< endl;	
	
// 	double ocost = 0;
// 	for (size_t i = 0; i < Graph.size() - 1; i++)
// 	{
// 		ocost += get_op_cost(i, 0, 0);
// 	}
// 	//cout << "ocost=" << ocost << endl;
// 	//ocost=inf是work的？
// 	while (1)
// 	{
// 		double d = numeric_limits<double>::infinity();
// 		int index = -1;
// 		//index是弹出还没被拓展过的最小的节点，顺序就是正确的那种
// 		//之所以报错死掉，是因为出现了原本那条中间的路就不work，delta加横向偏移量之后还不work，还都是inf的情况，根本搜不到终点去就退出掉了
// 		//然后又从终点回溯，自然就是回溯到终点的父节点是-1，对应出来最后result是0 -2 0  直接挂掉
// 		for (size_t i = 0; i < map.size(); i++)
// 		{
// 			// //cout << "i " << i << endl;
// 			// //cout << "collected[i] " << collected[i]<< endl;
// 			// //cout << "dist[i] is: " << dist[i]<< endl;
// 			if (!collected[i] && dist[i] < d)
// 			{
// 				d = dist[i];
// 				index = i;
// 			}
// 		}
// 		//cout << "------------------"<< endl;
// 		//dist就是cost
// 		//index=-1 说明所有的节点都拓展过了,或者没拓展的dist都是inf
// 		//index是 0 1 2 ...
// 		// //cout << "           index is: " << index<< endl;
// 		if (index == -1 || (index - 1) / 3 == Graph.size() - 2)
// 		{
// 			// //cout << "cost is: " << dist[index] << endl;
// 			break;
// 		}

// 		collected[index] = true;
// 		//这个拓展邻居是要在下一个node1里面的三个里选
// 		//get_op_cost 输入的是 node的id ，from这个node的左位还是右位还是中位，to 左中右
// 		for (int i = 0; i < 3; i++)
// 		{
// 			int nei = index - (index-1) % 3 + i + 3;
			
// 			if (index == 0)//为什么只连N1中间那个
// 			{
// 				nei = 1;
// 			}
// 			// //std::cout<<"              nei"<<nei<<std::endl;
// 			if (!collected[nei])
// 			{
// 				double c = dist[index];
// 				if (index % 3 == 0)
// 				{	
// 					// //std::cout<<"get_op_cost(index / 3, 2, i)"<<get_op_cost(index / 3, 2, i)<<std::endl;
// 					c += get_op_cost(index / 3, 2, i);
// 				}
// 				else
// 				{
// 					// //std::cout<<"get_op_cost(index / 3, 2, i)"<<get_op_cost(index / 3, 2, i)<<std::endl;
// 					c += get_op_cost(index / 3 + 1, index % 3 - 1, i);
// 				}
// 				if (dist[nei] > c)
// 				{
// 					dist[nei] = c;
// 					parent[nei] = index;
// 				}
// 			}
// 		}
// 	}
// 	result.push_back(0);
// 	int i = map.size() - 1;
// 	while (parent[i] != 0)
// 	{
// 		// //std::cout<<"parent[i]"<<parent[i]<<std::endl;
// 		// //std::cout<<"(parent[i] - 1) % 3"<<(parent[i] - 1) % 3<<std::endl;
// 		//比如9 是N4的第三位，序号就是2，(9-1)%3=2
// 		result.push_back((parent[i] - 1) % 3);
// 		i = parent[i];
// 	}
// 	result.push_back(0);
// 	reverse(result.begin(), result.end());

// 	for (auto p:result)
// 			//std::cout<<"1  p   "<<p<<std::endl;
// 	if (ocost>1e7)
// 	{
// 		for (auto p:result)
// 			p=0;
// 	}
// 	for (auto p:result)
// 			//std::cout<<"2  p   "<<p<<std::endl;
// 	ros::Time time_2 = ros::Time::now();
//     ROS_WARN("Time consume in dijk path finding is %f", (time_2 - time_1).toSec() );
// 	return result;
// }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "LocalOpitizater_node");
	ROS_WARN("Planner started!");

	Config config;
	LocalOptimizer planner;
	ros::spin();
}