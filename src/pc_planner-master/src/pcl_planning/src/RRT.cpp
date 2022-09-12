#include "RRT.h"
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_filter;
voxel_map::VoxelMap::Ptr voxelMapptr;
SDFMap::Ptr sdf_ptr;
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
	// std::cout<<"---------------------------------------------------------------------------------------------------------path.size() "<<path.size()<<std::endl;
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
			// std::cout<<"goal_tree "<<goal_tree.size()<<std::endl;
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
		// std::cout<<"	end tree cost "<<node.cost<<std::endl;
		// std::cout<<"						node.parent.second "<<node.parent.second<<std::endl;
		// if (node.parent.second != -1)
		// {
		// 	point_msg1.x = point(0);
		// 	point_msg1.y = point(1);
		// 	point_msg1.z = point(2);
		// 	point_parent = goal_tree[node.parent.second].node.get_T().block<3, 1>(0, 3);
		// 	point_msg2.x = point_parent(0);
		// 	point_msg2.y = point_parent(1);
		// 	point_msg2.z = point_parent(2);
		// 	std::cout<<"	point_parent "<<point_parent<<std::endl;
		// 	std::cout<<"	point "<<point<<std::endl;
		// 	edges.points.push_back(point_msg1);
		// 	edges.points.push_back(point_msg2);
		// }
	}

	tree_node_pub.publish(msg);
	tree_edge_pub.publish(edges);
}
pcl::PointCloud<pcl::PointXYZI> RRT::snowProj(const sensor_msgs::PointCloud2& globalmap)
{

    double RADIUS = 1;
    double MAX_D  = 0.5;
    pcl::PointCloud<pcl::PointXYZI> snow_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZI> tmp_kdtree;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_global_pc;
    tmp_global_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(globalmap, *tmp_global_pc);
    tmp_kdtree.setInputCloud(tmp_global_pc);
    Vector3d c_point;
    Vector3d n_point;
    pcl::PointXYZI c_pt;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    bool flag_success;
    int N = tmp_global_pc -> points.size();
    for(int i = 0 ; i < N; i++)
    {
        cout<<" snow Proj : "<< (i*100)/N << " %"<<endl;
        c_point(0) = tmp_global_pc -> points[i].x;
        c_point(1) = tmp_global_pc -> points[i].y;
        c_point(2) = tmp_global_pc -> points[i].z;
        c_pt = tmp_global_pc -> points[i];
        flag_success = true;
        if ( tmp_kdtree.radiusSearch (c_pt, RADIUS, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            for(int j = 0 ; j < pointIdxRadiusSearch.size() ; j++)
            {
                n_point(0) = tmp_global_pc -> points[ pointIdxRadiusSearch[j] ].x;
                n_point(1) = tmp_global_pc -> points[ pointIdxRadiusSearch[j] ].y;
                n_point(2) = tmp_global_pc -> points[ pointIdxRadiusSearch[j] ].z;
                if( n_point(2) > c_point(2) )
                {
                    double dis_xy = ( n_point.head(2) - c_point.head(2) ).norm() ;
                    double ang    = atan2( n_point(2) - c_point(2) , dis_xy );
                    if( ang > MAX_D ) { flag_success = false; break;}
                }

            }
        }
        if( flag_success ){
            snow_cloud.points.push_back( c_pt );
        }
    }
    return snow_cloud;
}
void RRT::pclMapCallback(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg)
{
	// pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
	if (!mapReceived)
	{
		const Eigen::Vector3i xyz((PCTrajNode::config.mapBound[1] - PCTrajNode::config.mapBound[0]) / PCTrajNode::config.resolution,
									(PCTrajNode::config.mapBound[3] - PCTrajNode::config.mapBound[2]) / PCTrajNode::config.resolution,
									(PCTrajNode::config.mapBound[5] - PCTrajNode::config.mapBound[4]) / PCTrajNode::config.resolution);

		const Eigen::Vector3d offset(PCTrajNode::config.mapBound[0], PCTrajNode::config.mapBound[2], PCTrajNode::config.mapBound[4]);
		std::cout<<"xyz "<<xyz<<std::endl;
		std::cout<<"offset "<<offset<<std::endl;
		std::cout<<"mconfig.resolution "<<PCTrajNode::config.resolution<<std::endl;

		voxelMapptr.reset(new voxel_map::VoxelMap(xyz, offset, PCTrajNode::config.resolution));
		sdf_ptr.reset(new SDFMap(PCTrajNode::config.esdf_vis_slice_height_));


		pcl::PointCloud<pcl::PointXYZI> cloud_afer_snow_proj=snowProj(*laserCloudMsg);
		mapReceived = true;

		std::cout<<" rrt  pcl CB"<<std::endl;
		ROS_WARN("Callback called!");

//-----------------------
			ros::Time t1 = ros::Time::now();
            for (size_t i = 0; i < cloud_afer_snow_proj.size(); i++)
            {
				Eigen::Vector3d tmp(cloud_afer_snow_proj[i].x,cloud_afer_snow_proj[i].y,cloud_afer_snow_proj[i].z);
				
            	voxelMapptr->setOccupied(tmp);
            }
			double time1=(ros::Time::now() - t1).toSec() * 1000;
			// std::cout<<" time1 "<<time1<<std::endl;
			cout<<"voxelMapptr->all_size "<<voxelMapptr->all_size<<endl;
//-----------------------
//-----------------------
            // size_t cur = 0;
            // const size_t total = laserCloudMsg->data.size() / laserCloudMsg->point_step;
            // float *fdata = (float *)(&laserCloudMsg->data[0]);
			// std::cout<<" laserCloudMsg->point_step "<<laserCloudMsg->point_step<<std::endl;
			// std::cout<<" laserCloudMsg->data.size() "<<laserCloudMsg->data.size()<<std::endl;
			// std::cout<<" total "<<total<<std::endl;
			// int all=0;
			std::vector<Eigen::Vector4d> your_Vector4d;
			// ros::Time t1 = ros::Time::now();
            // for (size_t i = 0; i < total; i++)
            // {
			// 	Eigen::Vector3d tmp(fdata[cur + 0],fdata[cur + 1],fdata[cur + 2]);
				
            //     cur = laserCloudMsg->point_step / sizeof(float) * i;

            //     if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
            //         std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
            //         std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
            //     {
            //         continue;
            //     }
			// 	all++;
            // 	voxelMapptr->setOccupied(Eigen::Vector3d(fdata[cur + 0],
            //                                              fdata[cur + 1],
            //                                              fdata[cur + 2]));
            // }
			// double time1=(ros::Time::now() - t1).toSec() * 1000;
			// // std::cout<<" time1 "<<time1<<std::endl;
			// cout<<"voxelMapptr->all_size "<<voxelMapptr->all_size<<endl;
			// std::cout<<" all "<<all<<std::endl;

//-----------------------
      	std::vector<Eigen::Vector3d> end_feasible_set;
        for(int x=0;x<=(PCTrajNode::config.mapBound[1] - PCTrajNode::config.mapBound[0]) / PCTrajNode::config.resolution;x++)
        for(int y=0;y<=(PCTrajNode::config.mapBound[3] - PCTrajNode::config.mapBound[2]) / PCTrajNode::config.resolution;y++)
        for(int z=0;z<=(PCTrajNode::config.mapBound[5] - PCTrajNode::config.mapBound[4]) / PCTrajNode::config.resolution;z++)
        {
            Eigen::Vector3i id(x,y,z);
            Eigen::Vector3d pos_tmp=voxelMapptr->posI2D(id);
			if(voxelMapptr->query(id)==true)
			{
				// if(abs(z-PCTrajNode::config.esdf_vis_slice_height_)<4 )
					end_feasible_set.push_back(pos_tmp);
			}
            	
        }
        std::cout<<"end_feasible_set.size()"<<end_feasible_set.size()<<std::endl;
		for(int zzz=0;zzz<10;zzz++){
			visualize(end_feasible_set,1);
			// pub_cloud(your_Vector3d);
			ros::Duration(0.3).sleep();
		}
		
        //     sdf_ptr->setEnvironment(voxelMapptr);
		// 	sdf_ptr->updateESDF3d();

        //     voxelMapptr->dilate();
        //     std::vector<Eigen::Vector3d> surface_points;
        //     voxelMapptr->getSurf(surface_points);
		// 	std::cout<<"surface_points.size()"<<surface_points.size()<<std::endl;
		// 	 visualize(surface_points,0);
//-----------------------

		//每一个点云都要有一个dobs,也就是传感器得来的观测方向，这里我用esdf的来代替，resolution还最好得跟原地图一致不然get surface有点难搞
		//而且为了防止地板只有一层，我需要get_surface,这样才能一个地板有两个方向的信息，才有厚度
		//也就是以足够的分辨率（得让地板可以分成两层才行）去update esdf ，再get surface ，过滤掉不在表面上的点云，
		//也就是说给定一个点云xyz，从voxel_map里面拿到他的法线的方向信息dobs
		// pcl::fromROSMsg(*laserCloudMsg, *PCTrajNode::PCMap);//laserCloudMsg
		*PCTrajNode::PCMap=cloud_afer_snow_proj;
		pcl::fromROSMsg(*laserCloudMsg, *PCTrajNode::debug);
		kdtree.setInputCloud(PCTrajNode::PCMap);
		int n = PCTrajNode::PCMap->points.size();
		std::cout<<"PCMap"<<n<<std::endl;
		pcl::StopWatch time;
		ros::Time t2 = ros::Time::now();

		Eigen::Vector4d Vector4d_tmp;
		// for (int i = 0; i < surface_points.size(); i++)
		for (int i = 0; i < n; i++)
		{
			if(i%100==0){
				std::cout<<"progress "<<double(i)/double(n)<<std::endl;
				// pub_cloud(your_Vector4d);
			}
				
		// 	pcl::PointXYZI tmp;tmp.x=surface_points[i](0);tmp.y=surface_points[i](1);tmp.z=surface_points[i](2);
		// 	tmp.intensity=-1;
		// 	Vector3d grad_tmp=PCTrajNode::calculate_rho(tmp,PCTrajNode::PCMap,kdtree);
		// 	Vector3d Pt,grad_esdf;
		// 	Pt(0)			=tmp.x;
		// 	Pt(1)			=tmp.y;
		// 	Pt(2)			=tmp.z;
		// 	sdf_ptr->getDistWithGradTrilinear(Pt, grad_esdf);
        //     double cos_value=grad_tmp.dot(grad_esdf)/(grad_tmp.norm()*grad_esdf.norm());
        //     if (cos_value<0)//如果不同向
        //    {
		// 	grad_tmp=-grad_tmp;
        //    }   
		//    double cos_vis=grad_tmp.dot(Vector3d(0,0,1))/grad_tmp.norm();

		//    Vector4d_tmp(0)=Pt(0);
		//    Vector4d_tmp(1)=Pt(1);
		//    Vector4d_tmp(2)=Pt(2);
		//    Vector4d_tmp(3)=cos_vis;
		//    your_Vector4d.push_back(Vector4d_tmp);
		   
        	// vis_grad(Pt, grad_tmp,i);
			// if()


		// 	ros::Duration(0.005).sleep();

			PCTrajNode::PCMap->points[i].intensity = -1;
			Vector3d grad_tmp=PCTrajNode::calculate_rho_and_normal(PCTrajNode::PCMap->points[i],PCTrajNode::PCMap,kdtree);
			// Vector3d Pt,grad_esdf;
			// Pt(0)			=PCTrajNode::PCMap->points[i].x;
			// Pt(1)			=PCTrajNode::PCMap->points[i].y;
			// Pt(2)			=PCTrajNode::PCMap->points[i].z;
			// sdf_ptr->getDistWithGradTrilinear(Pt, grad_esdf);
            // double cos_value=grad_tmp.dot(grad_esdf)/(grad_tmp.norm()*grad_esdf.norm());
			// 	if (cos_value<0)//如果不同向
			// {
			// 	grad_tmp=-grad_tmp;
			// }   
			double cos_vis=grad_tmp.dot(Vector3d(0,0,1))/grad_tmp.norm();
				if (cos_vis<0)//如果不同向
			{
				grad_tmp=-grad_tmp;
			}   


			// publishSamplePoint_2(pos);

			cos_vis=grad_tmp.dot(Vector3d(0,0,1))/grad_tmp.norm();
			// std::cout<<"cos_vis "<<cos_vis<<std::endl;
			pair<bool,double> temp = PCTrajNode::get_rho(PCTrajNode::PCMap->points[i],PCTrajNode::PCMap,kdtree);
			pcl::PointXYZI Point;

			if(temp.second<= PCTrajNode::PR.rho_max && cos_vis>PCTrajNode::PR.normal_max)
			{
				Point.intensity	=-1;
				Point.x			=PCTrajNode::PCMap->points[i].x;
				Point.y			=PCTrajNode::PCMap->points[i].y;
				Point.z			=PCTrajNode::PCMap->points[i].z;
				Point.intensity	=cos_vis;
				PCTrajNode::PCMap_fiter_rho_vis->push_back(Point);
				Vector3d Pt;
				Pt(0)			=PCTrajNode::PCMap->points[i].x;
				Pt(1)			=PCTrajNode::PCMap->points[i].y;
				Pt(2)			=PCTrajNode::PCMap->points[i].z;
				voxelMapptr->set_normal_for_rrt_is_ok(Pt);
			}	
			else{
				// std::cout<<"rho "<<temp.second<<std::endl;
			}
		}
		// for(int zzz=0;zzz<10;zzz++){
		// 	pub_cloud(your_Vector4d);
		// 	ros::Duration(0.3).sleep();
		// }

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

//-----
	// // visualize tree nodes
	// geometry_msgs::PoseArray msg;
	// msg.header.frame_id = "map";
	// msg.header.stamp = ros::Time::now();

	// //visualize tree edges
	// visualization_msgs::Marker edges;
	// edges.header.frame_id = "map";
	// edges.header.stamp = ros::Time::now();
	// edges.ns = "RRT";
	// edges.id = 0;
	// edges.type = visualization_msgs::Marker::LINE_LIST;
	// edges.scale.x = 0.2;
	// edges.color.b = 1;
	// edges.color.a = 1;

	// geometry_msgs::Pose pose;
	// tf::Matrix3x3 R;
	// Matrix4d T;
	// Vector3d point, point_parent;
	// geometry_msgs::Point point_msg1, point_msg2;

//-----
// 		std::vector<Eigen::Vector3d> after_fp_Set;
// 		for(int i = 0; i < PCTrajNode::debug->points.size(); i++)
// 		{
// 			if(i%100==0){
// 				std::cout<<"progress "<<double(i)/double(PCTrajNode::debug->points.size())<<std::endl;
// 			}

// 		// Vector3d TMP(PCTrajNode::PCMap->points[i].x,PCTrajNode::PCMap->points[i].y,PCTrajNode::PCMap->points[i].z);
// 		// if(voxelMapptr->query_if_normal_for_rrt_ok(TMP))
// 		// {
// 		// 	Vector4d_tmp(0)=TMP(0);
// 		// 	Vector4d_tmp(1)=TMP(1);
// 		// 	Vector4d_tmp(2)=TMP(2);
// 		// 	your_Vector4d.push_back(Vector4d_tmp);
// 		// }

// 			Matrix4d tmp_mat = Matrix4d::Identity();
// 			tmp_mat(0, 3) = PCTrajNode::debug->points[i].x;
// 			tmp_mat(1, 3) = PCTrajNode::debug->points[i].y;
// 			tmp_mat(2, 3) = PCTrajNode::debug->points[i].z;
// 			PCTrajNode temp_node(tmp_mat);

// 			// Vector3d pos(temp_node.T.topRightCorner(3, 1));
// 			// Vector3d direct_tmp;direct_tmp(0)=temp_node.T(0,2);direct_tmp(1)=temp_node.T(1,2);direct_tmp(2)=temp_node.T(2,2);
// 			// vis_grad(pos,direct_tmp,i);
			
// 			if(temp_node.isTraversable())
// 			{
// 				// Vector3d after_fp(temp_node.T(0,3),temp_node.T(1,3),temp_node.T(2,3));
// 				// after_fp_Set.push_back(after_fp);
// 				Vector4d_tmp(0)=temp_node.T(0,3);
// 				Vector4d_tmp(1)=temp_node.T(1,3);
// 				Vector4d_tmp(2)=temp_node.T(2,3);
// 				Vector4d_tmp(3)=temp_node.get_tau();
// 				your_Vector4d.push_back(Vector4d_tmp);

// //rviz死了...不pub这个线了 没必要...
// 						// point_msg1.x = Vector4d_tmp(0);
// 						// point_msg1.y = Vector4d_tmp(1);
// 						// point_msg1.z = Vector4d_tmp(2);

// 						// point_msg2.x = after_fp(0);
// 						// point_msg2.y = after_fp(1);
// 						// point_msg2.z = after_fp(2);
// 						// edges.points.push_back(point_msg1);
// 						// edges.points.push_back(point_msg2);

// 						// std::cout<<"----------------------------------------------"<<std::endl;

				

// 			}



// 		}
// 			for(int zzz=0;zzz<10;zzz++){
// 				// visualize(after_fp_Set,1);
// 				pub_cloud(your_Vector4d);
// 				// tree_edge_pub.publish(edges);
// 				ros::Duration(0.3).sleep();
// 			}
//-----

		// n = PCTrajNode::PCMap_fiter_rho_vis->points.size();
		// std::cout<<"PCMap_Size"<<n<<std::endl;
		pre_time=(ros::Time::now() - t2).toSec() * 1000;
		//cout << " calculate_all_rho consume_FINISHED:" << time.getTime() << "ms" << std::endl;
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
			// ROS_WARN("goal is not traversable");
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
	ros::Time nearest_time=ros::Time::now();
	int nearest = -1;
	double dist = std::numeric_limits<double>::infinity();
	// if((direct.first)==0)
	// 	std::cout<<"   在start_tree上找最近的"<<std::endl;
	// else{
	// 	std::cout<<"   在end_tree上找最近的"<<std::endl;
	// }
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
	double time1=(ros::Time::now() - nearest_time).toSec() * 1000;
	find_nearest_time_all+=time1;
	return nearest;
}

pair<bool, int> RRT::extend(Vector3d p)
{
					// while(trgger_for_debug)
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
	////cout << "extend: " << debug++ << endl;
	//在已有的rrt树上找到一个离采样点最近的
	
	int nearest = find_nearest(p);

	// //cout << "------------------------------nearest: " << nearest << endl;
	if (nearest == -1) //which suggests that the tree is empty now
	{	
		// //cout << "find_nearest=-1 may tree empty" << endl;
		return {true, -1}; // second is -1 means that it's invalid
	}

	Matrix4d T_new = Matrix4d::Identity();
	double dist = std::numeric_limits<double>::infinity();
	int new_index = -1;

	// find nearist extend traj
	//no use是指这个轨迹还长不长，轨迹库里有13个轨迹嘛
	Vector3d pt_nearest;
	pt_nearest(0)=(*direct.second)[nearest].node.T(0,3);
	pt_nearest(1)=(*direct.second)[nearest].node.T(1,3);
	pt_nearest(2)=(*direct.second)[nearest].node.T(2,3);
	Vector3d vector_near_sample=p-pt_nearest;
	Vector3d vector_near_new;
	double max_cos_value_for_alignment=-1.1;
	for (auto index : (*direct.second)[nearest].no_use)
	{
		Matrix4d T_i;
		// std::cout<<"direct.first "<<direct.first<<std::endl;
		if (direct.first)
		{
			T_i = (*direct.second)[nearest].node.Trans(traj_lib[index]);
		}
		else
		{
			T_i = (*direct.second)[nearest].node.Trans(traj_lib_back[index]);
		}
//--------
		// double d = (p - T_i.topRightCorner(3, 1)).norm();
		// //T_i是以nearest的node为原点展开的平面上按照轨迹库某一条去trans后得到的T_i
		// //找一条距离采样点最近的 xxxx
		// if (d < dist)
		// {
		// 	dist = d;
		// 	new_index = index;
		// 	// std::cout<<"traj_lib[index]"<<traj_lib[index]<<std::endl;
		// 	T_new = T_i;
		// }
//--------
		Vector3d new_;
		new_(0)=T_i(0,3);
		new_(1)=T_i(1,3);
		new_(2)=T_i(2,3);
		//找一条与near-sample线对齐的
		vector_near_new=new_-pt_nearest;
		double cos_tmp=vector_near_new.dot(vector_near_sample)/(vector_near_new.norm()*vector_near_sample.norm());
		// std::cout<<"cos_tmp "<<cos_tmp<<std::endl;
		if (cos_tmp > max_cos_value_for_alignment)
		{
			max_cos_value_for_alignment = cos_tmp;
			new_index = index;
			T_new = T_i;
		}
//--------
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

			// Vector3d pos(pc_new.T.topRightCorner(3, 1));
			// Vector3d grad_tmp;grad_tmp(0)=pc_new.T(0,2);grad_tmp(1)=pc_new.T(1,2);grad_tmp(2)=pc_new.T(2,2);

			// vis_grad(pos,grad_tmp,1);



	trgger_for_debug=true;


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
	// 	// std::cout<<"pt1   "<<pt<<std::endl;
	// // }
	// 	// Vector3d pt;
	// 	pt(0)=(*direct.second)[nearest].node.T(0,3);
	// 	pt(1)=(*direct.second)[nearest].node.T(1,3);
	// 	pt(2)=(*direct.second)[nearest].node.T(2,3);
	// 	publishSamplePoint_2(pt);
		// std::cout<<"pt2   "<<pt<<std::endl;
		// 		point_msg2.x = pt(0);
		// 		point_msg2.y = pt(1);
		// 		point_msg2.z = pt(2);
		// 		edges.points.push_back(point_msg2);
		// 		tree_edge_pub.publish(edges);
	if (pc_new.isTraversable())//判断其父节点和它之间会不会在z轴上有很大的差距，用0.5好了
	{
							MatrixXd T_b = ((*direct.second)[nearest].node.T.inverse())*(pc_new.T);
							double xf = T_b.row(0)[3];
							double yf = T_b.row(1)[3];
							double zf = T_b.row(2)[3];
							
							// std::cout<<"zf extend "<<zf<<endl;
							if(abs(zf)>0.5)
							{
								// ROS_WARN("Not traversable between nearest and new");
								return {direct.first, -1}; // not traversable, invalid
							}
		// pc_new.set_sf(traj_cost[new_index]);//现在可是放进去p_new和父节点的sf值
		RRT_Node p_new(pc_new, {direct.first, nearest});
		p_new.cost = (*direct.second)[nearest].cost + traj_cost[new_index];
		p_new.traj_idx = new_index;
		// (*direct.second)[nearest].node.set_sf(traj_cost[new_index]);//父节点的cost给set一下，但是其实一个父节点有好多个子节点，sf会被覆盖的 这里是有问题的
		// Vector3d pt;
		// pt(0)=(*direct.second)[nearest].node.T(0,3);
		// pt(1)=(*direct.second)[nearest].node.T(1,3);
		// pt(2)=(*direct.second)[nearest].node.T(2,3);
		// publishSamplePoint_2(pt);


		// 		point_msg2.x = pt(0);
		// 		point_msg2.y = pt(1);
		// 		point_msg2.z = pt(2);
		// 		edges.points.push_back(point_msg2);
		// std::cout<<"direct.first    start or end 					"<<direct.first<<std::endl;
		// std::cout<<"extending				    father 				"<<nearest<<std::endl;
		// std::cout<<"extending    				father   cost 		"<<(*direct.second)[nearest].cost<<std::endl;
		// std::cout<<"new_index "<<new_index<<std::endl;

		
		(*direct.second).push_back(p_new);
		
		pair<bool, int> p(direct.first, (*direct.second).size() - 1);
		if((*direct.second).size() - 1>50000)
			std::cout<<"超过50000个了"<<std::endl;
		// std::cout<<"p.first  "<<p.first<<"p.second  "<<p.second<<std::endl;
		set_tree_dist(p);



		// visualizeTrees();
		// tree_edge_pub.publish(edges);
		return {direct.first, (*direct.second).size() - 1};
		
	}
	else
	{
		// ROS_WARN("Not traversable");
		return {direct.first, -1}; // not traversable, invalid
	}
}
void RRT::set_tree_dist(pair<bool, int> p){
	if(p.first==0)//end tree的p
	{
		for (size_t i = 0; i < (start_tree).size(); i++) // traverse the other tree
		{
			pair<bool, int> p_start; p_start.first=1;p_start.second=i;
			double dist =goal_tree[p.second].node.get_dist((start_tree)[p_start.second].node);
			//p.second不会是-1把
			if(dist<PCTrajNode::config.rrt_param.trees_merge_thresh)
				Euclidean.insert(p.second,p_start.second)=dist;//行(end tree),列(start tree) 距离
		}

	}
	else{//start tree的p
		//cout << "(goal_tree).size() "<<(goal_tree).size()<< endl;
		for (size_t i = 0; i < (goal_tree).size(); i++) // traverse the other tree
		{
			pair<bool, int> p_goal; p_goal.first=0;p_goal.second=i;
			double dist =start_tree[p.second].node.get_dist((goal_tree)[p_goal.second].node);
			//p.second不会是-1把
			if(dist<PCTrajNode::config.rrt_param.trees_merge_thresh)
				Euclidean.insert(p_goal.second,p.second)=dist;//行(end tree),列(start tree) 距离
		}
	}
}
void RRT::update_cost(pair<bool, int> p)
{
	
	for (size_t i = 0; i < start_tree.size(); i++)
	{
		// std::cout<<"	Trees  这个树 cost "<<start_tree[i].node.get_cost()<<std::endl;
		// std::cout<<"	Trees  这个树 parent "<<start_tree[i].parent.second<<std::endl; // 连父节点和这个子
		// if(!start_tree[i].parent.second==-1)
		// 	std::cout<<"	Trees  这个树 parent cost"<<start_tree[start_tree[i].parent.second].node.get_cost()<<std::endl;
		if (start_tree[i].parent == p) // 父节点是p的
		{
			double c;
			// c = start_tree[p.second].node.get_cost();
			// if (c==0)//
			// {
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
			// }
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
bool RRT::connect_single(pair<bool, int> p_Start,pair<bool, int> p_end){
			vector<PCTrajNode> path_pc;
			pair<bool, int> tp = {true, p_Start.second};
			pair<bool, int> srt_end = {false, p_end.second};
			double dist=(start_tree)[p_Start.second].node.get_dist((goal_tree)[p_end.second].node) ;
			// cout<<"p_Start.first	"<<p_Start.first<<endl;
			// cout<<"p_end.first	"<<p_end.first<<endl;
			// cout<<"connect_single dist	"<<dist<<endl;
			path_pc = PCTrajNode::SRT_Generate((start_tree)[p_Start.second].node, (goal_tree)[p_end.second].node);

			if (path_pc.size()==1)//挂
			{
				// cout<<"connect_single path_pc.size()==1	"<<endl;
				return false;
			}
			if (path_pc.empty())
			{
				// cout<<"connect_single path_pc.empty()"<<endl;
				return false;
			}
			else
			{
///*
				// cout<<"add SRT nodes(中间的，不带始末) to start tree!"<<endl;
				// cout<<"		SRT出来的 size(包含SRT的fromnode和tonode)="<<path_pc.size()<<endl;

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
				// cout<<"		srt_end_node.cost    "<<srt_end_node.cost<<endl;
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
				ros::Time at1=ros::Time::now();
				update_cost({true, 0});//这个操作有问题 挂
				double time1=(ros::Time::now() - at1).toSec() * 1000;
				update_cost_time_all+=time1;
				return true;
//*/
/*
				cout<<"add SRT nodes(中间的，不带始末) to start tree!"<<endl;
				cout<<"		SRT出来的 size(包含SRT的fromnode和tonode)="<<path_pc.size()<<endl;

				for (size_t j = 1; j < path_pc.size() - 1; j++)
				{
						RRT_Node temp(path_pc[j], tp);
						temp.cost = get_Node(tp).cost + path_pc[j].get_cost();
						start_tree.push_back(temp);
						tp = {true, start_tree.size() - 1};
				}

				// merge goal tree
				// cout<<"merge goal tree!"<<endl;
				RRT_Node &srt_end_node = get_Node(srt_end);
				pair<bool, int> pg = srt_end_node.parent;
				srt_end_node.parent = tp;
				srt_end_node.cost = get_Node(tp).cost + path_pc[path_pc.size() - 1].get_cost();
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
				return true;
*/
			}

	return false;
}
bool RRT::try_connect(pair<bool, int> p)
{
	// cout<<"try connect!"<<endl;
	// cout<<"try_connect   start_tree.size()"<<start_tree.size()<<endl;
	// for (size_t j = 0; j <=start_tree.size() - 1; j++)
	// {
		// cout<<"		start_tree_j "<<j<<" start_tree(i).cost "<<start_tree[j].cost<<endl;
	// }

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
cout<<"(*direct.second)[i].node.get_dist((*tree)[p.second].node)    "<<(*direct.second)[i].node.get_dist((*tree)[p.second].node) <<endl;



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
			if (path_pc.size()==1)//挂
			{
				cout<<"		path_pc.size()  "<<path_pc.size()<<endl;
				continue;
			}
			if (path_pc.empty())
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
					// while(trgger_for_debug)
					// {
					// 	ros::spinOnce();
					// }
			Vector3d pos1(path_pc[j].T.topRightCorner(3, 1));
			Vector3d grad_tmp1;grad_tmp1(0)=path_pc[j].T(0,2);grad_tmp1(1)=path_pc[j].T(1,2);grad_tmp1(2)=path_pc[j].T(2,2);
			vis_grad(pos1,grad_tmp1,0);
			// publishSamplePoint(pos1);
					RRT_Node temp(path_pc[j], tp);
// cout<<"		path_pc[j] get_T"<<endl;
// cout<<path_pc[j].get_T()<<endl;
// cout<<"		get_Node(tp).cost  "<<get_Node(tp).cost<<endl;
// cout<<"		path_pc[j - 1].get_cost()   "<<path_pc[j - 1].get_cost()<<endl;

					temp.cost = get_Node(tp).cost + path_pc[j - 1].get_cost();
				// cout<<"		temp.cost    "<<temp.cost<<endl;
					start_tree.push_back(temp);

			Vector3d pos(temp.node.T.topRightCorner(3, 1));
			Vector3d grad_tmp;grad_tmp(0)=temp.node.T(0,2);grad_tmp(1)=temp.node.T(1,2);grad_tmp(2)=temp.node.T(2,2);
			vis_grad(pos,grad_tmp,1);
			// publishSamplePoint_2(pos);
			trgger_for_debug=true;
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
	// cout<<"can't connect!"<<endl;
	return false;
}

vector<RRT_Node> RRT::get_path()
{
	vector<RRT_Node> result;
	pair<bool, int> g = {true, goal_idx};
	//cout << "==============================get_path=============================="<< endl;
	//cout << "RRT  trajectory cost=" << get_Node(g).cost << endl;
	//cout << "g.second" << g.second << endl;
	while (g.second != -1)
	{	
		RRT_Node &p = get_Node(g);
		if(!p.node.isTraversable())
			cout<<"is Traversable   		"<<p.node.isTraversable()<<p.node.get_pos()[0] << " " << p.node.get_pos()[1] << " " << p.node.get_pos()[2]<<endl;
		if(g.second != 0)
		{
			PCTrajNode to_node = p.node;
			PCTrajNode  from_node= get_Node(p.parent).node;
			// cout<<"get connect in get path"<<endl;
			VectorXd par = PCTrajNode::get_connect(from_node, to_node);
			if(par[4]>10e7)
			{
				ROS_ERROR("get connect error");
				std::cout<<"	from_node "<<from_node.get_pos()[0]<<from_node.get_pos()[1]<<from_node.get_pos()[2]<<std::endl;
				std::cout<<"	to_node "<<to_node.get_pos()[0]<< " " <<to_node.get_pos()[1]<< " " <<to_node.get_pos()[2]<<std::endl;
			}
		}

		// std::cout<<"	get_path_from_goal cost "<<p.cost<<std::endl;
		// std::cout<<"						node.parent.second "<<p.parent.second<<std::endl;
		// //cout << p.node.get_pos()[0] << " " << p.node.get_pos()[1] << " " << p.node.get_pos()[2] << endl;
		result.push_back(p);
		g = p.parent;
	}
	inProcess = false;

	visualizePath(result);
	// ROS_WARN("RRT path nodes number: %d", result.size());
	//cout << "==============================get_path finished======================"<< endl;
	return result;
}
vector<RRT_Node> RRT::find_path(const PCTrajNode &start, const PCTrajNode &goal)
{
	inProcess = true;

	// initialization
	vector<RRT_Node> result;
	// if(clear_start_tree)
	// {
		start_tree.clear();
	// }
	// 
	Euclidean.setZero();
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
	//cout << "RRT begin!" << endl;
	pcl::StopWatch time;
	int z=0;
	for (int i = 0; i < PCTrajNode::config.rrt_param.max_iter; i++)
	{
		z++;
		// std::cout<<"iter "<<i<<" trgger_for_debug "<<trgger_for_debug<<std::endl;



		// sample, extend this tree
		// //cout << "iteration: " << i << endl;


		// if (i % 100 == 0)
		// {
			// ROS_WARN("Iteration: %d", i);

			// visualizeTrees();

		// }

		// std::cout<<"在pcmap的kdtree上rand"<<std::endl;

//------------------------
		// Vector3d p = sample();
		// pair<bool, int> p_new = extend(p);//extend return的是方向direct.first和 (*direct.second).size() - 1
		// if (p_new.second != -1) // if it's valid
		// {
		// 	// change, and try connect with that tree
			
		// 	change_direction();		//
		// 	std::cout<<"换方向,拿对面每个点跟p_new连"<<std::endl;
		// 	if (try_connect(p_new)) // path found
		// 	{   //cout << "try connect p_new yes "<< endl;
		// 		//cout << "get path in iteration: " << i << endl;
		// 		//cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
		// 		goal_idx = start_tree.size() - 1;
		// 		return get_path();
		// 	}
			
		// 	// can't connect, from that tree extend to p_new, try connect continuely
		// 	else
		// 	{
		// 		std::cout<<"p_new连不动"<<std::endl;
		// 		Vector3d p_new_pos = get_Node(p_new).node.get_pos();
		// 		// publishSamplePoint_2(p_new_pos);
		// 		pair<bool, int> p_t = extend(p_new_pos);
		// 		int cnt = 0;
		// 		//论文是要尝试连接两个树上所有能相互连（<3*rep）的点
		// 		while (p_t.second != -1)
		// 		{
		// 			std::cout<<"p_new连不动 在循环了"<<std::endl;
		// 			cnt++;
		// 			if (cnt >= PCTrajNode::config.rrt_param.heuristic_straight_thresh)
		// 			{
		// 				//cout << "cnt >= PCTrajNode::config.rrt_param.heuristic_straight_thresh break"<< endl;
		// 				break;
		// 			}
		// 			change_direction();
		// 			if (try_connect(p_t))
		// 			{
		// 				//cout << "try connect from other yes "<< endl;
		// 				//cout << "get path in  iteration: " << i << endl;
		// 				//cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
		// 				goal_idx = start_tree.size() - 1;
		// 				return get_path();
		// 			}
		// 			change_direction();
		// 			// publishSamplePoint_2(p_new_pos);
		// 			p_t = extend(p_new_pos);
		// 		}

		// 	}
		// }
//------------------------
				// visualizeTrees();
				make_direct_tree_start();				

				ros::Time sample_time1=ros::Time::now();
				Vector3d p_sample_start = sample();
				double tmp_sample=(ros::Time::now() - sample_time1).toSec() * 1000;
				sample_time_all+=tmp_sample;

				// publishSamplePoint(p_sample_start);
					
				ros::Time extend_time=ros::Time::now();
				pair<bool, int> p_new_start = extend(p_sample_start);
				double tmp_extend_1=(ros::Time::now() - extend_time).toSec() * 1000;
				get_extend_time_all+=tmp_extend_1;

				if(p_new_start.second != -1 ){
					// cout<<"p_new_start.first "<<p_new_start.first <<endl;
					// Vector3d pt;
					// pt(0)=(start_tree)[p_new_start.second].node.T(0,3);
					// pt(1)=(start_tree)[p_new_start.second].node.T(1,3);
					// pt(2)=(start_tree)[p_new_start.second].node.T(2,3);
					// publishSamplePoint(pt);
					if(connect_both_tree())
					{
						//cout << "iter_times "<<z << endl;
						return get_path();
					}
				}

				// visualizeTrees();
				make_direct_tree_end();

				ros::Time sample_time2=ros::Time::now();
				Vector3d p_sample_end = sample();
				double tmp_sample2=(ros::Time::now() - sample_time2).toSec() * 1000;
				sample_time_all+=tmp_sample2;

				// publishSamplePoint(p_sample_end);
				ros::Time extend_time2=ros::Time::now();
				pair<bool, int> p_new_end = extend(p_sample_end);
				double tmp_extend_2=(ros::Time::now() - extend_time2).toSec() * 1000;
				get_extend_time_all+=tmp_extend_2;

				if(p_new_end.second != -1 ){
					// cout<<"p_new_end.first "<<p_new_end.first <<endl;
					// pt(0)=(goal_tree)[p_new_end.second].node.T(0,3);
					// pt(1)=(goal_tree)[p_new_end.second].node.T(1,3);
					// pt(2)=(goal_tree)[p_new_end.second].node.T(2,3);
					// publishSamplePoint_2(pt);
					
					if(connect_both_tree())
					{
						//cout << "iter_times "<<z << endl;
						return get_path();
					}

				}
				
//------------------------
	}

	// can't find path
	//cout << "can't find path!!!" << endl;
	//cout << "iter_times "<<z << endl;
	std::cout<<"PCTrajNode::config.rrt_param.max_iter"<<PCTrajNode::config.rrt_param.max_iter<<std::endl;
	ROS_WARN("start tree size: %d, goal tree size: %d", start_tree.size(), goal_tree.size());
	// visualizeTrees();
	inProcess = false;
	return result;
}

//论文是要尝试连接两个树上所有能相互连（<3*rep）的点
inline bool RRT::connect_both_tree()
{
				// ros::Time connect_time=ros::Time::now();
				

//------------				
				// ros::Time at1=ros::Time::now();
				// make_direct_tree_end();
				// //cout << "(start_tree).size() "<<(start_tree).size()<< endl;
				// for (size_t i = 0; i < (start_tree).size(); i++) // traverse the other tree
				// {
				// 	//direct.first和 (*direct.second).size() - 1
				// 	pair<bool, int> p_start; p_start.first=1;p_start.second=i;

				// 	Vector3d pt;
				// 	pt(0)=(start_tree)[p_start.second].node.T(0,3);
				// 	pt(1)=(start_tree)[p_start.second].node.T(1,3);
				// 	pt(2)=(start_tree)[p_start.second].node.T(2,3);
				// 	// publishSamplePoint(pt);

				// 	if (try_connect(p_start))
				// 	{
				// 		//cout << "try connect p_start yes "<< endl;
				// 		//cout << "get path in  iteration: " << i << endl;
				// 		goal_idx = start_tree.size() - 1;
				// 		return true;
				// 	}
				// }
				// make_direct_tree_start();
				// //cout << "(goal_tree).size() "<<(goal_tree).size()<< endl;
				// for (size_t i = 0; i < (goal_tree).size(); i++) // traverse the other tree
				// {
				// 	//direct.first和 (*direct.second).size() - 1
				// 	pair<bool, int> p_end; p_end.first=0;p_end.second=i;
					
				// 	Vector3d pt;
				// 	pt(0)=(goal_tree)[p_end.second].node.T(0,3);
				// 	pt(1)=(goal_tree)[p_end.second].node.T(1,3);
				// 	pt(2)=(goal_tree)[p_end.second].node.T(2,3);
				// 	// publishSamplePoint_2(pt);
				// 	if (try_connect(p_end))
				// 	{
				// 		//cout << "try connect from other yes "<< endl;
				// 		//cout << "get path in  iteration: " << i << endl;
				// 		goal_idx = start_tree.size() - 1;
				// 		return true;
				// 	}
				// }
				// double time1=(ros::Time::now() - at1).toSec() * 1000;
				// //如果tryconnect是不满足长度直接略过去了,50乘以50 那就是0.004381ms级别,很快
				// //如果tryconnect 没被略过去,就会花很多时间
				// //而且还要抛弃掉已经连接过的试验
				////而且我煞笔了.start tree上每个点连对面每个点之后不需要再反过来 end tree连start tree每个点了...
				// std::cout<<"		 time1 "<<time1<<std::endl;
				// return false;
//------------	
				ros::Time at1=ros::Time::now();
				//cout << "非零元素的数量 "<<Euclidean.nonZeros()<< endl;
				Euclidean.prune(0.0);
				// //cout << "非零元素的数量 "<<Euclidean.nonZeros()<< endl;
				for(int k=0;k<Euclidean.outerSize();++k)
				{
					for(SparseMatrix<double>::InnerIterator it(Euclidean,k);it;++it)
					{
						// if(it.value()==0)
						// 	continue;
						pair<bool, int> p_Start,p_end;
						p_Start.first=1;p_Start.second=it.col();
						p_end.first=0;p_end.second=it.row();
							// Vector3d pt;
							// pt(0)=(goal_tree)[p_end.second].node.T(0,3);
							// pt(1)=(goal_tree)[p_end.second].node.T(1,3);
							// pt(2)=(goal_tree)[p_end.second].node.T(2,3);
							// publishSamplePoint_2(pt);
							// pt(0)=(start_tree)[p_Start.second].node.T(0,3);
							// pt(1)=(start_tree)[p_Start.second].node.T(1,3);
							// pt(2)=(start_tree)[p_Start.second].node.T(2,3);
							// publishSamplePoint(pt);
						//cout << "value "<<it.value()<< endl;
						if(connect_single(p_Start,p_end))
						{
							//cout << "connect_single yes "<< endl;
							goal_idx = start_tree.size() - 1;
							return true;
						}
						else{
							//直接赋0没法删除非零元素,并不会释放它,还是会被迭代器迭到,因此非零系数的数量是相同的
							//使用 makeCompressed() 无济于事
							//可以通过调用显式删除它们A.prune(0.0),这将触发剩余列条目的昂贵内存副本
							//可以直接it.value()==0 continue
							Euclidean.coeffRef(it.row(),it.col())=0;
						}
					}
				}
//--------------------------
				double time1=(ros::Time::now() - at1).toSec() * 1000;
				//如果tryconnect是不满足长度直接略过去了,50乘以50 那就是0.004381ms级别,很快
				//如果tryconnect 没被略过去,就会花很多时间
				//而且还要抛弃掉已经连接过的试验
				// std::cout<<"		                                                                                              time1 "<<time1<<std::endl;
				// double tmp_connect_=(ros::Time::now() - connect_time).toSec() * 1000;
				// connect_time_all+=tmp_connect_;
				return false;
}
// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "RRT_node");
// 	ROS_WARN("Planner started!");
// 	RRT planner;
// 	ros::spin();
// }