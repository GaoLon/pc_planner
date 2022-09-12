#include "PCTrajNode.h"
#include "config.hpp"
#include "voxel_map.hpp"
#include "sdf_map.h"
extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_filter;
extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

extern voxel_map::VoxelMap::Ptr voxelMapptr;
extern SDFMap::Ptr sdf_ptr;

Config PCTrajNode::config;
param_restrain PCTrajNode::PR;
pcl::PointCloud<pcl::PointXYZI>::Ptr PCTrajNode::PCMap(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr PCTrajNode::debug(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr PCTrajNode::PCMap_fiter_rho_vis(new pcl::PointCloud<pcl::PointXYZI>);

PCTrajNode::PCTrajNode(const Matrix4d& Tr, double _kap) :T(MatrixXd::Identity(4, 4)), \
tau(0.0), kappa(_kap), t(VectorXd::Zero(5))
{
	//pcl::StopWatch time;
	/*pcl::KdTreeFLANN<pcl::PointXYZI> kd;
	pcl::PointXYZI searchPoint;
	searchPoint.x = Tr.col(3)[0];
	searchPoint.y = Tr.col(3)[1];
	searchPoint.z = Tr.col(3)[2];
	std::vector<int> Idx_res;
	std::vector<float> Dist_res;
	kdtree.radiusSearch(searchPoint, 1.5, Idx_res, Dist_res);
	pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap(new pcl::PointCloud<pcl::PointXYZI>);
	kdmap->is_dense = false;
	kdmap->points.resize(Idx_res.size());
	for (size_t i = 0; i < kdmap->points.size(); ++i)
	{
		kdmap->points[i] = PCTrajNode::PCMap->points[Idx_res[i]];
	}
	kd.setInputCloud(kdmap);*/

	// if (fp(Tr, PCMap_fiter_rho_vis, kdtree_filter))
	// {
	// 	tau = ft(PCMap, kdtree);
	// }
	// else
	// {
	// 	// std::cout<<"fp false"<<std::endl;
	// 	tau = 0;
	// }

	if (fp(Tr, PCMap, kdtree))
	{
		tau = ft(PCMap, kdtree);
	}
	else
	{
		tau = 0;
	}
	
	//cout << "terrain assessment consume:" << time.getTime() << "ms" << std::endl;
}

PCTrajNode::PCTrajNode(const Matrix4d& Tr, bool simple) :T(Tr), \
tau(0.0), kappa(0.0), t(VectorXd::Zero(5))
{
	// if (!simple)
	// {
	// 	if (fp(Tr, PCMap_fiter_rho_vis, kdtree_filter))
	// 	{
	// 		tau = ft(PCMap, kdtree);
	// 		//vis
	// 		// this->pubProjectedPose();
	// 	}
	// 	else
	// 	{
	// 		// std::cout<<"fp false"<<std::endl;
	// 		tau = 0;
	// 	}
	// }


	if (!simple)
	{
		if (fp(Tr, PCMap, kdtree))
		{
			tau = ft(PCMap, kdtree);
			//vis
			// this->pubProjectedPose();
		}
		else
		{
			// std::cout<<"fp false"<<std::endl;
			tau = 0;
		}
	}
}

void PCTrajNode::ter_assess(void)
{ 

	if (fp(T, PCMap, kdtree))
		tau = ft(PCMap, kdtree);
	else 
		tau = 0; 

	// if (fp(T, PCMap, kdtree))
	// 	tau = ft(PCMap, kdtree);
	// else 
	// 	tau = 0; 
}
	bool PCTrajNode::isTraversable(void) { 
		// std::cout<<"PR.traversable_threshold "<<PR.traversable_threshold<<std::endl;
		// Vector3d pos(T(0,3),T(1,3),T(2,3));
		// Eigen::Vector3i id=voxelMapptr->posD2I(pos);
		// bool is_free=true;
		// if(voxelMapptr->query(id))
		// {
		// 	is_free=false;
		// }
		// else//如果不是被占据,那就查查是否是那种在层间但略有偏差的
		// {
		// 	for(int dz=-2;dz<=2;dz++)
		// 	{
		// 		Eigen::Vector3i id_tmp=id+Eigen::Vector3i(0,0,dz);
		// 		if(voxelMapptr->query(id_tmp))
		// 			is_free=false;
		// 	}
		// }
		// if(is_free==true)
		// 	return false;
		return tau > PR.traversable_threshold; 
		}
bool PCTrajNode::fp(const Matrix4d& Tr, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd, int K)
{
	// Initilization
	Matrix<double, 3, 1> X = Tr.col(0).head(3);
	Matrix<double, 3, 1> Y = Tr.col(1).head(3);
	Matrix<double, 3, 1> Z = Tr.col(2).head(3);
	Vector3d t = Tr.col(3).head(3);

	// get Z-axis Z_t
	/// get convariance matrix with KNN

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);


	pcl::PointXYZI searchPoint;
	searchPoint.x = t[0];
	searchPoint.y = t[1];
	searchPoint.z = t[2];

//------------------
	//  std::vector<int> pointIdxNKNSearch_tmp(1);
	//  std::vector<float> pointNKNSquaredDistance_tmp(1);
	//  kd.nearestKSearch(searchPoint,1, pointIdxNKNSearch_tmp, pointNKNSquaredDistance_tmp);
	//  Vector3d fp_tmp_pt(kdmap->points[pointIdxNKNSearch_tmp[0]].x, \
	// 		kdmap->points[pointIdxNKNSearch_tmp[0]].y, kdmap->points[pointIdxNKNSearch_tmp[0]].z);
	// searchPoint.x = fp_tmp_pt(0);
	// searchPoint.y = fp_tmp_pt(1);
	// searchPoint.z = fp_tmp_pt(2);
//------------------

	Matrix<double, 3, 1> mean_p = Vector3d::Zero();
	//get the nearest k neighbors of searchPoint
//这里searchpoint nan，就挂掉了
	if(!pcl_isfinite(searchPoint.x) || !pcl_isfinite(searchPoint.y)  || !pcl_isfinite(searchPoint.z) )
	{
		std::cout<<"Tr"<<Tr<<std::endl;
		std::cout<<"searchPoint.x"<<searchPoint.x<<std::endl;
		std::cout<<"searchPoint.y"<<searchPoint.y<<std::endl;
		std::cout<<"searchPoint.z"<<searchPoint.z<<std::endl;
	}
	kd.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
	// std::vector<int> pointIdxNKNSearch;
	// std::vector<float> pointNKNSquaredDistance;
	// kd.radiusSearch(searchPoint, PR.r_plane, pointIdxNKNSearch, pointNKNSquaredDistance);
	//有可能radiusSearch是空的

	/*for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
	{
		mean_p += Vector3d(PCTrajNode::PCMap->points[pointIdxNKNSearch[i]].x, \
			PCTrajNode::PCMap->points[pointIdxNKNSearch[i]].y, PCTrajNode::PCMap->points[pointIdxNKNSearch[i]].z);
	}*/

	for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
	{
		mean_p += Vector3d(kdmap->points[pointIdxNKNSearch[i]].x, \
			kdmap->points[pointIdxNKNSearch[i]].y, kdmap->points[pointIdxNKNSearch[i]].z);
	}
	mean_p /= pointIdxNKNSearch.size();
	// std::cout<<"pointIdxNKNSearch.size()"<<pointIdxNKNSearch.size()<<std::endl;
	// std::cout<<"mean_p"<<mean_p<<std::endl;
	Matrix3d cov = Matrix3d::Zero();
	/*for (int i = 0; i < pointIdxNKNSearch.size(); i++)
	{
		Matrix<double, 3, 1> v = Matrix<double, 3, 1>(kdmap->points[pointIdxNKNSearch[i]].x, \
			kdmap->points[pointIdxNKNSearch[i]].y, \
			kdmap->points[pointIdxNKNSearch[i]].z) - mean_p;
		cov += v * v.transpose();
	}*/
	for (int i = 0; i < pointIdxNKNSearch.size(); i++)
	{
		Matrix<double, 3, 1> v = Matrix<double, 3, 1>(kdmap->points[pointIdxNKNSearch[i]].x, \
			kdmap->points[pointIdxNKNSearch[i]].y, \
			kdmap->points[pointIdxNKNSearch[i]].z) - mean_p;
		cov += v * v.transpose();
	}

	/// opposite PCA

	EigenSolver<Matrix3d> es(cov);
	Matrix<double, 3, 1> D = es.pseudoEigenvalueMatrix().diagonal();// 特征值
	Matrix3d V = es.pseudoEigenvectors();    // 特征向量
	MatrixXd::Index evalsMax;
	D.minCoeff(&evalsMax);
	Matrix<double, 3, 1> Z_t = V.col(evalsMax);
// std::cout<<"pointIdxNKNSearch.size()  "<<pointIdxNKNSearch.size()<<std::endl;

	// if (X.dot(Z_t) < 0)//if (Z.dot(Z_t) < 0)
	// {
	// 	Z_t = -Z_t;
	// }

			double cos_vis=Z_t.dot(Vector3d(0,0,1))/Z_t.norm();
				if (cos_vis<0)//如果不同向
			{
				Z_t=-Z_t;
			}  

	// get T
	/// get rotation matrix
	Matrix<double, 3, 1> X_a = Y.cross(Z_t);
	Matrix<double, 3, 1> X_t = X_a / X_a.norm();
	Matrix3d R = Matrix3d::Zero();
	R.col(0) = X_t;
	R.col(1) = Z_t.cross(X_t);
	R.col(2) = Z_t;
	///get translate
	Vector3d t_t = t + Z_t.dot(mean_p - t) / Z.dot(Z_t)*Z;
	// std::cout<<"t_t"<<t_t<<std::endl;
	/// get final T
	T.topLeftCorner(3,3) = R;
	
	T.topRightCorner(3, 1) = Matrix<double, 3, 1>(t_t);

	// if(!voxelMapptr->query_if_normal_for_rrt_ok(t_t))
	// {
	// 	// cout <<"normal for rrt is not ok " << endl;
	// 	return 0;
	// }

	// std::cout<<"-.norm   "<<(t_t - mean_p).norm()<<std::endl;
	return (t_t - mean_p).norm() < 1;//0.5
	//cout << T << endl;
}

//get tao,travisibility
double PCTrajNode::ft(pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd)
{
	// std::cout<<"PR.w_rho"<<PR.w_rho<<std::endl;
	// std::cout<<"PR.w_roll"<<PR.w_roll<<std::endl;
	// std::cout<<"PR.w_pitch"<<PR.w_pitch<<std::endl;
	// std::cout<<"PR.rho_max"<<PR.rho_max<<std::endl;
	// std::cout<<"PR.roll_max"<<PR.roll_max<<std::endl;
	// std::cout<<"PR.rho_max"<<PR.rho_max<<std::endl;
	// get pitch and roll
	Matrix3d t33 = T.topLeftCorner(3, 3);
	/*Eigen::Quaternion<double> q(t33);
	auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
	double pitch = euler[1];
	double roll = euler[2];
	Vector3d ea = t33.eulerAngles(2, 0, 1);
	*/

//pitch和roll 以及法线都应该是用来约束fp之后的点,如果是约束fp之前的点,比如说tran.trajlib之后的点,很有可能在空中,比如SRT半路的点,也很有可能在空中
//但这里确实是ft,是proj之后的点欸
	//TODO
	double roll = atan2(t33.row(2)[1], hypot(t33.row(1)[1], t33.row(0)[1]));
	double pitch = atan2(t33.row(2)[0], hypot(t33.row(1)[0], t33.row(0)[0]));
	double pimin = pitch / PR.pitch_min;
	double pimax = pitch / PR.pitch_max;

	if (pitch<PR.pitch_min || pitch>PR.pitch_max || fabs(roll) > PR.roll_max)
	{
		// cout <<   " pitch or roll error" << endl;
		return 0;
	}

	// get rho_cub( see as ball )
	/// get box
	vector<double> rho;
	/*pcl::PointCloud<pcl::PointXYZI>::Ptr box{ new pcl::PointCloud<pcl::PointXYZI> };
	pcl::CropBox<pcl::PointXYZI> clipper;
	Matrix<double, 4, 1> leftdown(0, 0, -PR.d_rob[2], 1);
	Matrix<double, 4, 1> minp = T * leftdown;
	Vector4f min_point(minp.col(0)[0], minp.col(0)[1], minp.col(0)[2], minp.col(0)[3]);
	Matrix<double, 4, 1> mp = T * PR.d_rob;
	Vector4f max_point(mp.col(0)[0], mp.col(0)[1], mp.col(0)[2], mp.col(0)[3]);
	clipper.setMin(min_point);
	clipper.setMax(max_point);
	clipper.setInputCloud(kdmap);
	clipper.setNegative(false);
	clipper.filter(*box);*/
	
	Matrix<double, 4, 1> cent(PR.d_rob[0]/2, 0, 0, 1);
	Matrix<double, 4, 1> mp = T * cent;
	pcl::PointXYZI p;
	p.x = mp.col(0)[0];
	p.y = mp.col(0)[1];
	p.z = mp.col(0)[2];

	// pcl::PointXYZI ppp;ppp.x=T(0,3);ppp.y=T(1,3);ppp.z=T(2,3);

	// double angle_Rad=PCTrajNode::calculate_rho(ppp,PCTrajNode::PCMap,kdtree);
	// if(angle_Rad >= PCTrajNode::PR.normal_max)
	// {
	// 	cout <<   " angle_Rad error" << endl;
	// 	return 0;
	// }

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	kd.radiusSearch(p, PR.d_rob[0] / 2.0 * 1.7320508075, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	/// get rhos
	for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
	{
		//pcl::StopWatch ti;
		pair<bool,double> temp = get_rho(kdmap->points[pointIdxRadiusSearch[i]],kdmap, kd);
		// cout << "temp.second" << temp.second  << std::endl;
		// cout << "PR.rho_max" << PR.rho_max  << std::endl;
		// cout << "temp.first       " << temp.first  << std::endl;
		// if (temp.second > PR.rho_max)   // why ?? 挂
		// {
		// 	if (temp.first)
		// 		return 0;
		// 	else
		// 		temp.second = PR.rho_max;
		// }
		//算一个点可通行度的时候还要要求这个邻居点的normal是合理的，这个用一个栅格地图存是否work的01信息，如果1就不通行，扔掉，
		//至于为什么不在已经处理好的即满足rho又满足normal的filtermap上面去kdtree ，radiusSearch
		//因为这样的话，如果有一个点自己是work的但是box内的邻居非法，就还是会认为可以通行，在一些有墙的地方会穿过去。
		//但我这么做之后又出现新的问题,就是如果正常的路底下有柱子,柱子侧面是挂掉的那这块本来正常的路就挂了

		//
		// Vector3d TMP(kdmap->points[pointIdxRadiusSearch[i]].x,kdmap->points[pointIdxRadiusSearch[i]].y,kdmap->points[pointIdxRadiusSearch[i]].z);
		// if(!voxelMapptr->query_if_normal_for_rrt_ok(TMP))
		// {
		// 	// cout <<"normal for rrt is not ok " << endl;
		// 	return 0;
		// }
		if (temp.second > PR.rho_max)   
		{
			cout <<   "temp.second "<<temp.second  << endl;
			cout <<   "temp.second > PR.rho_max" << endl;
				return 0;
		}
		rho.push_back(temp.second);
	}
	double rho_cub = 0;
	if (!rho.empty())
	{
		rho_cub = accumulate(rho.begin(), rho.end(), 0.0);
		rho_cub /= (double)rho.size();
	}

	return 1 - PR.w_rho*rho_cub / PR.rho_max + PR.w_roll*fabs(roll) / PR.roll_max + PR.w_pitch*(pimin > pimax ? pimin : pimax);
}


//this problem is: known the SE2 and curv of (start and end),how to solve  cubic poly
//the trajectory lib is :draw a circle ,same distance sample on circle,
//the cur of the end is 0,solve the problem mentioned before  
//return kabcsf
VectorXd PCTrajNode::get_connect(const PCTrajNode& fromNode, const PCTrajNode& toNode)//现在用OBVP还是cubic都可以
{ 		

	// // through OBVP
	// VectorXd result = VectorXd::Zero(5);
	// // get boundary value
	// double kappa0 =fromNode.kappa;
	// //change tonode.t from map frame to fromnode.frame
	// MatrixXd T_b = fromNode.T.inverse()*toNode.T;
	// double xf = T_b.row(0)[3];
	// double yf = T_b.row(1)[3];
	// double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);//the yaw between tonode to fromnode
	// if (T_b.row(0)[3] < 0)
	// {//if tonode is not in the orientation of fromnode
	// 	ROS_ERROR("get_connect---------xf is nagetive!");
	// 	cout<<"T_b.row(0)[3] "<<T_b.row(0)[3]<<endl;
	// 	result[4]=INFINITY;
	// 	return result;
	// }

	// if (fabs(thetaf) > 1.57)
	// {//if the yaw of these two node is too big
	// 	ROS_ERROR("get_connect----delta theta is too high!");
	// 	result[4]=INFINITY;
	// 	return result;
	// }
	// double kappaf = toNode.kappa;

	// // close-formed solution
	// double T1 = xf;
	// double T2 = T1*T1;
	// double T3 = T2*T1;
	// double T4 = T3*T1;
	// double T5 = T4*T1;
	// result[0] = kappa0;
	// result[3] = 10*(kappaf*T2-6*thetaf*T1+12*yf)/T5;
	// result[2] = -6*(30*yf-14*T1*thetaf-T2*kappa0+2*T2*kappaf)/T4;
	// result[1] = 3*(20*yf-8*T1*thetaf-2*T2*kappa0+T2*kappaf)/T3;
	// result[4] = T1;
	// return result;
//--------------------------------------------------------------------
	// through cubic curvature
	
	VectorXd result = VectorXd::Zero(5);
	if (toNode.tau <= PR.traversable_threshold) 
	{
		// ROS_ERROR("get_connect  toNode 不可通行"); 
		result[4]=INFINITY;
		return result;
	}
		
	if (fromNode.tau <= PR.traversable_threshold) 
	{
		// ROS_ERROR("get_connect fromnode 不可通行"); 
		result[4]=INFINITY;
		return result;
	}
		
	
	// get boundary value
	double x0 = 0.0;
	double y0 = 0.0;
	double theta0 = 0.0;
	double kappa0 = fromNode.kappa;
	Vector4d init(x0, y0, theta0, kappa0);
	MatrixXd T_b = fromNode.T.inverse()*toNode.T;
	double xf = T_b.row(0)[3];
	double yf = T_b.row(1)[3];

	if (T_b.row(0)[3] < 0)
	{
		// ROS_ERROR("get_connect xf is nagetive!"); 
		result[4]=INFINITY;
		return result;
	}
	double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);
	if (fabs(thetaf) > 1.57)
	{
		// ROS_ERROR("get_connect delta theta is too high!"); 
		result[4]=INFINITY;
		return result;
	}

	double kappaf = toNode.kappa;
	Vector4d ends(xf, yf, thetaf, kappaf);
	//cout << "end=" << ends << endl;

	result[0] = kappa0;
	result.tail(4) = get_cubic_curvature(init, ends);
	if (result.tail(4) == Vector4d::Zero())
	{
		// cout << "cubic curvature failed!" << endl;
		result[4]=INFINITY;
		return result;
	}
	return result;
//--------------------------------------------------------------------
	// MatrixXd T_b = fromNode.T.inverse()*toNode.T;
	// double xf = T_b.row(0)[3];
	// double yf = T_b.row(1)[3];
	// double dsg = hypot(xf, yf);
	// VectorXd result = VectorXd::Zero(5);
	// result[4]=dsg;
	// return result;	


}

VectorXd PCTrajNode::connect(const PCTrajNode& toNode)
{
	t = get_connect(*this, toNode);
	return t;
}

//d1 maybe the max distance between tonode and fromnode it allow 
//caculate curve and check curvature constraint
vector<PCTrajNode> PCTrajNode::SRT_Generate(const PCTrajNode& fromNode, const PCTrajNode& toNode, double d1,double d2)
{
//-------------------------------------------------------------------------------------------------
// 	vector<PCTrajNode> result;
// 	MatrixXd T_b = fromNode.T.inverse()*toNode.T;
// 	double xf = T_b.row(0)[3];
// 	// std::cout<<"			xf "<<xf<<std::endl;
// 	double yf = T_b.row(1)[3];
// 	double dsg = hypot(xf, yf);
// 	// if(dsg <= d1)
// 	// {
// 	// 	return result;
// 	// }
// 	// 	if (T_b.row(0)[3] < 0)
// 	// {
// 	// 	// std::cout<<"			to node is behind the fromNode"<<std::endl;
// 	// 	return result;
// 	// }
// 	double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);
// 	if (fabs(thetaf) > 2)
// 	{
// 		// std::cout<<"			thetaf is over 90 degree"<<std::endl;
// 		return result;
// 	}
// 						// Vector4d srt;
// 						// double x0 = 0.0;
// 						// double y0 = 0.0;
// 						// double theta0 = 0.0;
// 						// double kappa0 = fromNode.kappa;
// 						// double kappaf = toNode.kappa;
// 						// Vector4d init(x0, y0, theta0, kappa0);
// 						// Vector4d ends(xf, yf, thetaf, kappaf);
// 						// srt = get_cubic_curvature(init, ends);
// 						// if (srt == Vector4d::Zero())
// 						// {
// 						// 	// ROS_ERROR("one cubic, but,get_cubic_curvature = 0");
// 						// 	result.clear();
// 						// 	return result;
// 						// }	

// 	PCTrajNode SRT_start(fromNode.T);//T kappa
// 	SRT_start.set_sf(PR.d_nom);
// 	result.push_back(SRT_start);

// 	for (double seg = PR.d_nom; seg < dsg; seg+=PR.d_nom)
// 	{
// 			Vector4d path_seg(seg*xf/dsg,seg*yf/dsg, atan2(yf, xf),0);//x y theta k
			
// 			Matrix4d T_seg = Matrix4d::Identity();
// 			T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
// 			T_seg.row(0)[1] = -sin(path_seg[2]);
// 			T_seg.row(1)[0] = sin(path_seg[2]);
// 			T_seg.row(0)[3] = path_seg[0];
// 			T_seg.row(1)[3] = path_seg[1];
// // cout<<"		fromNode.T"<<endl;
// // cout<<fromNode.T<<endl;
// // cout<<"		T_seg"<<endl;
// // cout<<T_seg<<endl;

// // std::cout<<"			path_seg  "<<path_seg<<std::endl;
// 			PCTrajNode temp(fromNode.T*T_seg);//T kappa
// 			temp.set_sf(PR.d_nom);
// // cout<<"		temp.get_T()"<<endl;
// // cout<<temp.get_T()<<endl;
// 			// temp.t(4)=PR.d_nom;
// 			if (temp.isTraversable())
// 			{
// 				result.push_back(temp);
				
// 			}
// 			else
// 			{
// 				result.clear();
// 				// ROS_WARN("one cubic,have some not traversable along this SRT");
// 				return result;
// 			}
// 	}
// 	PCTrajNode SRT_end(toNode.T);//最后一个点是连着树的，sf不知道
// 	result.push_back(SRT_end);
// 	return result;	
//-------------------------------------------------------------------------------------------------
//--------------
					// // through cubic curvature
					// // initilization
					// ROS_WARN("-------------------SRT");
					vector<PCTrajNode> result;
					double x0 = 0.0;
					double y0 = 0.0;
					double theta0 = 0.0;
					double kappa0 = fromNode.kappa;






					Vector4d init(x0, y0, theta0, kappa0);
					MatrixXd T_b = fromNode.T.inverse()*toNode.T;
					double xf = T_b.row(0)[3];
					double yf = T_b.row(1)[3];
					if (T_b.row(0)[3] < 0)
					{
						// ROS_ERROR("			to node is behind the fromNode");
						// std::cout<<"T_b.row(0)[3] "<<T_b.row(0)[3]<<std::endl;
						result.clear();
						return result;
					}

					double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);

					if (fabs(thetaf) > 1.57)
					{
						// ROS_ERROR("			thetaf is over 2");
						result.clear();
						return result;
					}

					double kappaf = toNode.kappa;
					Vector4d ends(xf, yf, thetaf, kappaf);
					double dsg = hypot(xf, yf);
					
					// if(dsg <= d1)
					// {
						Vector4d srt;
						srt = get_cubic_curvature(init, ends);
						if (srt == Vector4d::Zero())
						{
							// ROS_ERROR("one cubic, but,get_cubic_curvature = 0");
							result.clear();
							return result;
						}	
						// ROS_WARN("-------------------rewiring");
						Vector4d x(srt);
						for (double seg = 0.0; seg < srt[3]; seg+=PR.d_nom)
						{
							x[3] = seg;
							Vector4d path_seg = forward(init, x);
							Matrix4d T_seg = Matrix4d::Identity();
							T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
							T_seg.row(0)[1] = -sin(path_seg[2]);
							T_seg.row(1)[0] = sin(path_seg[2]);

								T_seg.row(0)[3] = path_seg[0];
								T_seg.row(1)[3] = path_seg[1];

							PCTrajNode temp(fromNode.T*T_seg, path_seg[3]);
							//----------------------------------------------
							// x[3] = seg+PR.d_nom;
							// path_seg = forward(init, x);
							// Matrix4d T_seg_next = Matrix4d::Identity();
							// T_seg_next.row(0)[0] = T_seg_next.row(1)[1] = cos(path_seg[2]);
							// T_seg_next.row(0)[1] = -sin(path_seg[2]);
							// T_seg_next.row(1)[0] = sin(path_seg[2]);
							// PCTrajNode temp_next(fromNode.T*T_seg_next, path_seg[3]);
							if (temp.isTraversable())//这些新的只考虑单个是否traversable 没考虑是否两两可以连
							{
								// double dist=(temp_next.T.topRightCorner(3, 1)-temp.T.topRightCorner(3, 1)).norm();
								// temp.set_sf(dist);
								// temp.set_sf(PR.d_nom);
								result.push_back(temp);
							}
							else
							{
								result.clear();
								// ROS_WARN("one cubic,have some not traversable along this SRT");
								return result;
							}
						}
						// sf是当前点到子节点的sf时候，toNode不set_sf  //sf是父节点到当前点的sf时候，fromnode不set_sf
						result.push_back(toNode);
						for(size_t i = 0; i < result.size()-1; i++)
						{
							
							double cost=PCTrajNode::get_connect(result[i], result[i+1])[4];	

							MatrixXd T_b = (result[i].T.inverse())*(result[i+1].T);
							double xf = T_b.row(0)[3];
							double yf = T_b.row(1)[3];
							double zf = T_b.row(2)[3];
							
							// std::cout<<"zf srt"<<zf<<endl;
							if(abs(zf)>0.5)
							{
								// ROS_ERROR("abs(zf>0.5)");
								result.clear();
								return result;
							}

							if(cost==INFINITY){
								// ROS_ERROR("SRT MAYBE ERROR!  clear and return");
								result.clear();
								return result;
							}
							result[i].set_sf(cost);
							// result[i+1].set_sf(cost);
						}
						// ROS_WARN("SRT SUCCESS");
						// cout<<"result.size() "<<result.size()<<endl;
						return result;

//-----------
					// }
					// else if(d1< dsg < d2){//两段
					// ROS_WARN("d1< dsg < d");

					// 			Vector4d srt1,srt2;
					// 			double x_mid = xf/2;
					// 			double y_mid = yf/2;
					// 			double theta_mid = thetaf/2 ;//这个就是瞎搞
					// 			double kappa_mid = 0;
					// 			Vector4d middle_waypoint(x_mid, y_mid, theta_mid, kappa_mid);
					// 			srt1 = get_cubic_curvature(init, middle_waypoint);
					// 			if (srt1 == Vector4d::Zero())
					// 			{
					// 				ROS_ERROR("two cubic, but first part get_cubic_curvature = 0");
					// 				result.clear();
					// 				return result;
					// 			}	
					// 			Vector4d x(srt1);
					// 			Matrix4d T_seg= Matrix4d::Identity();
					// 			for (double seg = 0.0; seg < srt1[3]; seg+=PR.d_nom)
					// 			{
					// 				x[3] = seg;
					// 				Vector4d path_seg = forward(init, x);
					// 				T_seg= Matrix4d::Identity();
					// 				T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
					// 				T_seg.row(0)[1] = -sin(path_seg[2]);
					// 				T_seg.row(1)[0] = sin(path_seg[2]);
					// 				T_seg.row(0)[3] = path_seg[0];
					// 				T_seg.row(1)[3] = path_seg[1];

					// 				PCTrajNode temp(fromNode.T*T_seg, path_seg[3]);
					// 				if (temp.isTraversable())
					// 				{
					// 					// temp.set_sf(PR.d_nom);
					// 					result.push_back(temp);
					// 				}
					// 				else
					// 				{
					// 					result.clear();
					// 					ROS_WARN("two cubic,have some not traversable along first part of SRT");
					// 					return result;
					// 				}
					// 			}
					// 			Matrix4d T_seg_tmp=T_seg;

					// 			srt2 = get_cubic_curvature(middle_waypoint,ends);//abcsf
					// 			if (srt2 == Vector4d::Zero())
					// 			{
					// 				ROS_ERROR("two cubic, but second part get_cubic_curvature = 0");
					// 				result.clear();
					// 				return result;
					// 			}	
					// 			x(0)=srt2(0);x(1)=srt2(1);x(2)=srt2(2);x(3)=srt2(3);
					// 			for (double seg = 0.0; seg < srt2[3]; seg+=PR.d_nom)
					// 			{
					// 				x[3] = seg;
					// 				Vector4d path_seg = forward(middle_waypoint, x);
					// 				T_seg = Matrix4d::Identity();
					// 				T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
					// 				T_seg.row(0)[1] = -sin(path_seg[2]);
					// 				T_seg.row(1)[0] = sin(path_seg[2]);
					// 				T_seg.row(0)[3] = path_seg[0];
					// 				T_seg.row(1)[3] = path_seg[1];

					// 				PCTrajNode temp(fromNode.T*T_seg_tmp*T_seg, path_seg[3]);
					// 				if (temp.isTraversable())
					// 				{
					// 					// temp.set_sf(PR.d_nom);
					// 					result.push_back(temp);
					// 				}
					// 				else
					// 				{
					// 					result.clear();
					// 					ROS_WARN("two cubic,have some not traversable along second part of SRT");
					// 					return result;
					// 				}
					// 			}
					// 	result.push_back(toNode);
					// 	for(size_t i = 0; i < result.size()-1; i++)
					// 	{
					// 		double cost=PCTrajNode::get_connect(result[i], result[i+1])[4];
					// 		if(cost==INFINITY){
					// 			ROS_ERROR("MAYBE ERROR!");
					// 		}
					// 		result[i].set_sf(cost);
					// 	}
					// 	return result;						
					// }
					// else{//曲直曲
					// 	ROS_WARN("曲直曲");
					// 	Vector4d srt1,srt2;
					// 	double x_second = PR.d_nom*xf/dsg;
					// 	double y_second = PR.d_nom*yf/dsg;
					// 	double theta_second = thetaf/2 ;//这个就是瞎搞
					// 	double kappa_second = 0;
					// 	Vector4d middle_Second_waypoint(x_second, y_second, theta_second, kappa_second);
					// 	srt1 = get_cubic_curvature(init, middle_Second_waypoint);

					// 	double x_last_second = xf-PR.d_nom*xf/dsg;
					// 	double y_last_second = yf-PR.d_nom*yf/dsg;
					// 	double theta_last_second = thetaf/2 ;//这个就是瞎搞
					// 	double kappa_last_second = 0;
					// 	Vector4d middle_last_waypoint(x_last_second, y_last_second, theta_last_second, kappa_last_second);
					// 	srt2 = get_cubic_curvature(middle_last_waypoint,ends);

						

					// 	if (srt1 == Vector4d::Zero())
					// 	{
					// 		ROS_ERROR("two cubic, but first part get_cubic_curvature = 0");
					// 		result.clear();
					// 		return result;
					// 	}	
					// 	Vector4d x(srt1);
					// 	Matrix4d T_seg= Matrix4d::Identity();
					// 	for (double seg = 0.0; seg < srt1[3]; seg+=PR.d_nom)
					// 	{
					// 		x[3] = seg;
					// 		Vector4d path_seg = forward(init, x);
					// 		T_seg = Matrix4d::Identity();
					// 		T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
					// 		T_seg.row(0)[1] = -sin(path_seg[2]);
					// 		T_seg.row(1)[0] = sin(path_seg[2]);
					// 		T_seg.row(0)[3] = path_seg[0];
					// 		T_seg.row(1)[3] = path_seg[1];

					// 		PCTrajNode temp(fromNode.T*T_seg, path_seg[3]);
					// 		if (temp.isTraversable())
					// 		{
					// 			// temp.set_sf(PR.d_nom);
					// 			result.push_back(temp);
					// 		}
					// 		else
					// 		{
					// 			result.clear();
					// 			// ROS_WARN("two cubic,have some not traversable along first part of SRT");
					// 			return result;
					// 		}
					// 	}
					// 	Matrix4d T_seg_tmp=T_seg;
					// 	for (double seg = srt1[3]; seg < dsg-srt2[3]; seg+=PR.d_nom)
					// 	{
					// 			Vector4d path_seg(seg*xf/dsg,seg*yf/dsg, thetaf/2,0);//x y theta k
								
					// 			T_seg = Matrix4d::Identity();
					// 			T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
					// 			T_seg.row(0)[1] = -sin(path_seg[2]);
					// 			T_seg.row(1)[0] = sin(path_seg[2]);
					// 			T_seg.row(0)[3] = path_seg[0];
					// 			T_seg.row(1)[3] = path_seg[1];

					// 			PCTrajNode temp(fromNode.T*T_seg_tmp*T_seg);//T kappa
					// 			// temp.set_sf(PR.d_nom);

					// 			if (temp.isTraversable())
					// 			{
					// 				// temp.set_sf(PR.d_nom);
					// 				result.push_back(temp);
									
					// 			}
					// 			else
					// 			{
					// 				result.clear();
					// 				return result;
					// 			}
					// 	}

					// 	if (srt2 == Vector4d::Zero())
					// 	{
					// 		ROS_ERROR("two cubic, but second part get_cubic_curvature = 0");
					// 		result.clear();
					// 		return result;
					// 	}	

					// 	Matrix4d T_seg_tmp2=T_seg;
					// 	x(0)=srt2(0);x(1)=srt2(1);x(2)=srt2(2);x(3)=srt2(3);
					// 	for (double seg = 0.0; seg < srt2[3]; seg+=PR.d_nom)
					// 	{
					// 		x[3] = seg;
					// 		Vector4d path_seg = forward(middle_last_waypoint, x);
					// 		T_seg = Matrix4d::Identity();
					// 		T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
					// 		T_seg.row(0)[1] = -sin(path_seg[2]);
					// 		T_seg.row(1)[0] = sin(path_seg[2]);
					// 		T_seg.row(0)[3] = path_seg[0];
					// 		T_seg.row(1)[3] = path_seg[1];

					// 		PCTrajNode temp(fromNode.T*T_seg_tmp*T_seg_tmp2*T_seg, path_seg[3]);
					// 		if (temp.isTraversable())
					// 		{
					// 			// temp.set_sf(PR.d_nom);
					// 			result.push_back(temp);
					// 		}
					// 		else
					// 		{
					// 			result.clear();
					// 			ROS_WARN("two cubic,have some not traversable along second part of SRT");
					// 			return result;
					// 		}
					// 	}
					// 	result.push_back(toNode);
					// 	for(size_t i = 0; i < result.size()-1; i++)
					// 	{
					// 		double cost=PCTrajNode::get_connect(result[i], result[i+1])[4];
					// 		if(cost==INFINITY){
					// 			ROS_ERROR("MAYBE ERROR!");
					// 		}
					// 		result[i].set_sf(cost);
					// 	}
					// 	return result;

					// }
	
	
//-------------------------------------------------------------------------------------------------



	// // through OBVP
	// // initilization
	// vector<PCTrajNode> result;
	// double kappa0 = fromNode.kappa;
	// MatrixXd T_b = fromNode.T.inverse()*toNode.T;
	// double xf = T_b.row(0)[3];
	// double yf = T_b.row(1)[3];

	// if (T_b.row(0)[3] < 0)//||fabs(yf/xf)>1)
	// {
	// 	cout<<"00000"<<endl;
	// 	return result;
	// }

	// double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);

	// if (fabs(thetaf) > 1.57)
	// {
	// 	cout<<"11111"<<endl;
	// 	return result;
	// }

	// double kappaf = toNode.kappa;
	// double dsg = hypot(xf, yf);//return sqrt(x*x+y*y)

	// if (dsg >= d1)
	// {
	// 	cout<<"22222"<<endl;
	// 	return result;
	// }
	
	// // close-formed solution
	// double T1 = xf;
	// double T2 = T1*T1;
	// double T3 = T2*T1;
	// double T4 = T3*T1;
	// double T5 = T4*T1;
	// Vector4d srt;
	// srt[2] = 10*(kappaf*T2-6*thetaf*T1+12*yf)/T5;
	// srt[1] = -6*(30*yf-14*T1*thetaf-T2*kappa0+2*T2*kappaf)/T4;
	// srt[0] = 3*(20*yf-8*T1*thetaf-2*T2*kappa0+T2*kappaf)/T3;
	// srt[3] = T1;

	// if (srt == Vector4d::Zero())
	// {
	// 	cout<<"33333"<<endl;
	// 	result.clear();
	// 	return result;
	// }

	// // check curvature constraint
	// double x0, x1,max_kappa=0;
	// int r = gsl_poly_solve_quadratic(srt[2]*3, srt[1]*2, srt[0],&x0,&x1 );
	// switch (r)
	// {
	// 	case 0: break;
	// 	case 1: 
	// 		if (x0>0 && x0<srt[3])
	// 		{
	// 			for (int j = 3; j >=0; j--)
	// 			{
	// 				max_kappa *= x0;
	// 				max_kappa += srt[j];
	// 			}
	// 			max_kappa = fabs(max_kappa);
	// 		}
	// 		break;
	// 	case 2:
	// 		if (x0>0 && x0<srt[3])
	// 		{
	// 			for (int j = 3; j >=0; j--)
	// 			{
	// 				max_kappa *= x0;
	// 				max_kappa += srt[j];
	// 			}
	// 		}
	// 		if (x1>0 && x1<srt[3])
	// 		{
	// 			double kap=0;
	// 			for (int j = 3; j >=0; j--)
	// 			{
	// 				kap *= x1;
	// 				kap += srt[j];
	// 			}
	// 			max_kappa = max(fabs(max_kappa), fabs(kap));
	// 		}
	// 		break;
	// 	default: max_kappa = 10; cout<<"gsl error!"<<endl;break;
	// }

	// if (max_kappa>PCTrajNode::PR.kappa_max)
	// {
	// 	result.clear();
	// 	return result;
	// }



	// // forward and cast
	// //不想用英文注释了。。还是用中文把..
	// //先用隆之算法连接前后俩node，下面是计算这个curve上各个点的状态，由于是以前一个node为frame，所以直接矩阵相乘，就能拿到中间的一些node了
	// //（但这些应该是在起始的那个node的平面上的）
	// //一旦有一个not isTraversable,就不oneshot了
	// Vector4d x(srt);
	// for (double seg = 0.0; seg < srt[3]; seg+=PR.d_nom)
	// {
	// 	x[3] = seg;
	// 	//x y theta kappa
	// 	Vector4d path_seg;// = forward(init, x);
	// 	double seg2 = seg*seg;
	// 	double seg3 = seg*seg2;
	// 	double seg4 = seg*seg3;
	// 	double seg5 = seg*seg4;
	// 	path_seg[0] = seg;
	// 	path_seg[1] = kappa0*seg2/2+ srt[0]*seg3/6+srt[1]*seg4/12+srt[2]*seg5/20;
	// 	path_seg[2] = kappa0*seg+ srt[0]*seg2/2+srt[1]*seg3/3+srt[2]*seg4/4;
	// 	path_seg[3] = kappa0 + srt[0]*seg+srt[1]*seg2+srt[2]*seg3;
	// 	Matrix4d T_seg = Matrix4d::Identity();
	// 	T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
	// 	T_seg.row(0)[1] = -sin(path_seg[2]);
	// 	T_seg.row(1)[0] = sin(path_seg[2]);
	// 	T_seg.row(0)[3] = path_seg[0];
	// 	T_seg.row(1)[3] = path_seg[1];

	// 	PCTrajNode temp(fromNode.T*T_seg, path_seg[3]);
	// 	if (temp.isTraversable())
	// 	{
	// 		result.push_back(temp);
	// 	}
	// 	else
	// 	{
	// 		result.clear();
	// 		return result;
	// 	}
	// }

	// result.push_back(toNode);

	// // connect and check whether kappa is valid
	// bool kap_ok = true;
	// for (size_t i = 0; i < result.size() - 1; i++)
	// {
	// 	VectorXd pol = result[i].connect(result[i + 1]);
	// 	if (pol == VectorXd::Zero(5))
	// 	{
	// 		result.clear();
	// 		return result;
	// 	}
	// 	double x0, x1,max_kappa=0;
	// 	int r = gsl_poly_solve_quadratic(pol[3]*3, pol[2]*2,pol[1],&x0,&x1 );
	// 	switch (r)
	// 	{
	// 		case 0: break;
	// 		case 1: 
	// 			if (x0>0 && x0<pol[4])
	// 			{
	// 				for (int j = 3; j >=0; j--)
	// 				{
	// 					max_kappa *= x0;
	// 					max_kappa += pol[j];
	// 				}
	// 				max_kappa = fabs(max_kappa);
	// 			}
	// 			break;
	// 		case 2:
	// 			if (x0>0 && x0<pol[4])
	// 			{
	// 				for (int j = 3; j >=0; j--)
	// 				{
	// 					max_kappa *= x0;
	// 					max_kappa += pol[j];
	// 				}
	// 			}
	// 			if (x1>0 && x1<pol[4])
	// 			{
	// 				double kap=0;
	// 				for (int j = 3; j >=0; j--)
	// 				{
	// 					kap *= x1;
	// 					kap += pol[j];
	// 				}
	// 				max_kappa = max(fabs(max_kappa), fabs(kap));
	// 			}
	// 			break;
	// 		default: max_kappa = 10; cout<<"gsl error!"<<endl;break;
	// 	}

	// 	if (max_kappa>PCTrajNode::PR.kappa_max)
	// 	{
	// 		result.clear();
	// 		return result;
	// 	}
	// 	// for (double dist = 0.0; dist < pol[4]; dist += 0.01)
	// 	// {
	// 	// 	double kap = 0;
	// 	// 	for (int j = 3; j >=0; j--)
	// 	// 	{
	// 	// 		kap *= dist;
	// 	// 		kap += pol[j];
	// 	// 	}
	// 	// 	if (fabs(kap) > PR.kappa_max)
	// 	// 	{
	// 	// 		//cout << "kappa = " << kap << " to high!" << endl;
	// 	// 		kap_ok = false;
	// 	// 		break;
	// 	// 	}
	// 	// }
	// 	// if (!kap_ok)
	// 	// {
	// 	// 	result.clear();
	// 	// 	return result;
	// 	// }
	// }

	// // cout<<"xf="<<T1<<endl;
	// // cout<<"yf="<<yf<<endl;
	return result;
}

//get one point's rho=|max(dik)-min(dik)|
Vector3d PCTrajNode::calculate_normal(pcl::PointXYZI& p, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd)
{
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	kd.radiusSearch(p, PR.r_plane, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	Matrix<double, 3, 1> mean_p = Vector3d::Zero();
	for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
	{
		mean_p += Vector3d(kdmap->points[pointIdxRadiusSearch[i]].x, \
			kdmap->points[pointIdxRadiusSearch[i]].y, kdmap->points[pointIdxRadiusSearch[i]].z);
	}
	mean_p /= pointIdxRadiusSearch.size();
	// get normal
	Matrix3d cov = Matrix3d::Zero();
	for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
	{
		Matrix<double, 3, 1> v = Matrix<double, 3, 1>(kdmap->points[pointIdxRadiusSearch[i]].x, \
			kdmap->points[pointIdxRadiusSearch[i]].y, \
			kdmap->points[pointIdxRadiusSearch[i]].z) - mean_p;
		cov += v * v.transpose();
	}
	/// opposite PCA
	EigenSolver<Matrix3d> es(cov);
	Matrix<double, 3, 1> D = es.pseudoEigenvalueMatrix().diagonal();// 特征值
	Matrix3d V = es.pseudoEigenvectors();    // 特征向量
	MatrixXd::Index evalsMax;
	D.minCoeff(&evalsMax);
	Matrix<double, 3, 1> n = V.col(evalsMax);
	//n是法线,是拟合平面出来的那个法线
	Vector3d tmp;tmp(0)=n(0,0);tmp(1)=n(1,0);tmp(2)=n(2,0);
	return tmp;
}
Vector3d PCTrajNode::calculate_rho_and_normal(pcl::PointXYZI& p, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd)
{
	// if (p.intensity != -1)
	// {
	// 	if (p.intensity < 0)
	// 		return { true,-p.intensity };
	// 	else
	// 		return { false,p.intensity };
	// }
	// get near nodes
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	kd.radiusSearch(p, PR.r_plane, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	Matrix<double, 3, 1> mean_p = Vector3d::Zero();
	for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
	{
		mean_p += Vector3d(kdmap->points[pointIdxRadiusSearch[i]].x, \
			kdmap->points[pointIdxRadiusSearch[i]].y, kdmap->points[pointIdxRadiusSearch[i]].z);
	}
	mean_p /= pointIdxRadiusSearch.size();
	// get normal
	Matrix3d cov = Matrix3d::Zero();
	for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
	{
		Matrix<double, 3, 1> v = Matrix<double, 3, 1>(kdmap->points[pointIdxRadiusSearch[i]].x, \
			kdmap->points[pointIdxRadiusSearch[i]].y, \
			kdmap->points[pointIdxRadiusSearch[i]].z) - mean_p;
		cov += v * v.transpose();
	}
	/// opposite PCA
	EigenSolver<Matrix3d> es(cov);
	Matrix<double, 3, 1> D = es.pseudoEigenvalueMatrix().diagonal();// 特征值
	Matrix3d V = es.pseudoEigenvectors();    // 特征向量
	MatrixXd::Index evalsMax;
	D.minCoeff(&evalsMax);
	Matrix<double, 3, 1> n = V.col(evalsMax);
	//n是法线,是拟合平面出来的那个法线
	Vector3d tmp;tmp(0)=n(0,0);tmp(1)=n(1,0);tmp(2)=n(2,0);

	// double cos_result=n.dot(Vector3d(0,0,1))/n.norm();
	// if(cos_result<0)
	// 	cos_result=-cos_result;
	// double angle_Rad=acos(cos_result);



	// get rho
	std::vector<int> Idx_res;
	std::vector<float> Dist_res;
	kd.radiusSearch(p, PR.r_res, Idx_res, Dist_res);
	vector<double> nears;
	// std::cout<<"Idx_res.size()"<<Idx_res.size()<<std::endl;
	if(Idx_res.size()==0)//挂
		return tmp;
	for (auto& i : Idx_res)
	{
		Matrix<double, 3, 1> pi = { kdmap->points[i].x, \
			kdmap->points[i].y, \
			kdmap->points[i].z };
		nears.push_back(n.transpose().dot(pi - mean_p));
	}
	sort(nears.begin(), nears.end());
	size_t a = static_cast<double>(nears.size()*PR.eta / 2.0);
	//a是离群用的
	double pd = n.transpose().dot(Matrix<double, 3, 1>{p.x, p.y, p.z} -mean_p);
	double d_min = nears[a];
	double d_max = nears[nears.size() - 1 - a];
	pair<bool, double> result{ false,fabs(d_max - d_min) };

	if (d_max <= pd || pd <= d_min)
	{
		result.first = true;
		p.intensity = -result.second;
		// std::cout<<"p.intensity "<<p.intensity<<std::endl;
		// p.intensity= p.intensity-angle_for_intensity;
		// std::cout<<"p.intensity "<<p.intensity<<std::endl;
	}
	else
	{
		p.intensity = result.second;
		// p.intensity= p.intensity+angle_for_intensity;
	}

	return tmp;
	// return angle_Rad;
	// return result;
}
pair<bool, double> PCTrajNode::get_rho(pcl::PointXYZI& p, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd)
{
	if (p.intensity != -1)//用了normal,解码之后可能有点隐患
	{
		if (p.intensity < 0)
			return { true,-p.intensity };
		else
			return { false,p.intensity };
	}
	// if (p.intensity != -1)//用了normal,解码之后可能有点隐患
	// {
	// 	if (p.intensity < 0)
	// 	{
	// 		std::cout<<"p.intensity "<<p.intensity<<std::endl;
	// 		std::cout<<"int(-p.intensity/10)*10 "<<int(-p.intensity/10)*10<<std::endl;
	// 		///比如-1793.1
	// 		return { true,-p.intensity+int(-p.intensity/10)*10 };
	// 	}

	// 	else///比如1793.1
	// 	{
	// 		return { false,p.intensity-int(p.intensity/10)*10 };
	// 	}

	// }
}
void pubProjectedPose(ros::Publisher *pubPtr, PCTrajNode node)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id="map";
	msg.header.stamp=ros::Time::now();
	Vector3d point=node.T.block<3,1>(0,3);
	msg.pose.position.x=point(0);
	msg.pose.position.y=point(1);
	msg.pose.position.z=point(2);
	tf::Matrix3x3 R;
	for(int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			R[i][j]=node.T(i,j);
		}
	}
	tfScalar yaw, pitch,row;
	R.getEulerYPR(yaw, pitch, row);
	msg.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(row, pitch, yaw);
	pubPtr->publish(msg);
}
