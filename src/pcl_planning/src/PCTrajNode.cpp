#include "PCTrajNode.h"

extern pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

param_restrain PCTrajNode::PR;

pcl::PointCloud<pcl::PointXYZI>::Ptr PCTrajNode::PCMap(new pcl::PointCloud<pcl::PointXYZI>);

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
	Matrix<double, 3, 1> mean_p = Vector3d::Zero();
	//get the nearest k neighbors of searchPoint
	kd.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
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
	if (Z.dot(Z_t) < 0)
	{
		Z_t = -Z_t;
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
	/// get final T
	T.topLeftCorner(3,3) = R;
	T.topRightCorner(3, 1) = Matrix<double, 3, 1>(t_t);

	return (t_t - mean_p).norm() < 0.5;
	//cout << T << endl;
}

double PCTrajNode::ft(pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd)
{
	// get pitch and roll
	Matrix3d t33 = T.topLeftCorner(3, 3);
	/*Eigen::Quaternion<double> q(t33);
	auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
	double pitch = euler[1];
	double roll = euler[2];
	Vector3d ea = t33.eulerAngles(2, 0, 1);
	*/


	//TODO
	double roll = atan2(t33.row(2)[1], hypot(t33.row(1)[1], t33.row(0)[1]));
	double pitch = atan2(t33.row(2)[0], hypot(t33.row(1)[0], t33.row(0)[0]));
	double pimin = pitch / PR.pitch_min;
	double pimax = pitch / PR.pitch_max;

	if (pitch<PR.pitch_min || pitch>PR.pitch_max || fabs(roll) > PR.roll_max)
	{
		// cout << pitch << "  " << roll  << endl;
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
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	kd.radiusSearch(p, PR.d_rob[0] / 2.0 * 1.7320508075, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	/// get rhos
	for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
	{
		//pcl::StopWatch ti;
		pair<bool,double> temp = get_rho(kdmap->points[pointIdxRadiusSearch[i]],kdmap, kd);
		//cout << "rho time consume: " << ti.getTime() << "ms" << std::endl;
		if (temp.second > PR.rho_max)
		{
			if (temp.first)
				return 0;
			else
				temp.second = PR.rho_max;
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

VectorXd PCTrajNode::get_connect(const PCTrajNode& fromNode, const PCTrajNode& toNode)
{
	// through OBVP
	VectorXd result = VectorXd::Zero(5);
	// get boundary value
	double kappa0 =fromNode.kappa;
	MatrixXd T_b = fromNode.T.inverse()*toNode.T;
	double xf = T_b.row(0)[3];
	double yf = T_b.row(1)[3];
	if (T_b.row(0)[3] < 0)
	{
		//cout << "xf is nagetive!" << endl;
		return result;
	}
	double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);
	if (fabs(thetaf) > 1)
	{
		//cout << "delta theta is too high!" << endl;
		return result;
	}
	double kappaf = toNode.kappa;

	// close-formed solution
	double T1 = xf;
	double T2 = T1*T1;
	double T3 = T2*T1;
	double T4 = T3*T1;
	double T5 = T4*T1;
	result[0] = kappa0;
	result[3] = 10*(kappaf*T2-6*thetaf*T1+12*yf)/T5;
	result[2] = -6*(30*yf-14*T1*thetaf-T2*kappa0+2*T2*kappaf)/T4;
	result[1] = 3*(20*yf-8*T1*thetaf-2*T2*kappa0+T2*kappaf)/T3;
	result[4] = T1;
	return result;

	// through cubic curvature
	// VectorXd result = VectorXd::Zero(5);
	// // get boundary value
	// double x0 = 0.0;
	// double y0 = 0.0;
	// double theta0 = 0.0;
	// double kappa0 = fromNode.kappa;
	// Vector4d init(x0, y0, theta0, kappa0);
	// MatrixXd T_b = fromNode.T.inverse()*toNode.T;
	// double xf = T_b.row(0)[3];
	// double yf = T_b.row(1)[3];
	// if (T_b.row(0)[3] < 0)
	// {
	// 	//cout << "xf is nagetive!" << endl;
	// 	return result;
	// }
	// double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);
	// if (fabs(thetaf) > 1)
	// {
	// 	//cout << "delta theta is too high!" << endl;
	// 	return result;
	// }
	// double kappaf = toNode.kappa;
	// Vector4d ends(xf, yf, thetaf, kappaf);
	// //cout << "end=" << ends << endl;

	// result[0] = kappa0;
	// result.tail(4) = get_cubic_curvature(init, ends);
	// if (result.tail(4) == Vector4d::Zero())
	// {
	// 	//cout << "cubic curvature failed!" << endl;
	// 	return VectorXd::Zero(5);
	// }
	// return result;
}

VectorXd PCTrajNode::connect(const PCTrajNode& toNode)
{
	t = get_connect(*this, toNode);
	return t;
}

vector<PCTrajNode> PCTrajNode::SRT_Generate(const PCTrajNode& fromNode, const PCTrajNode& toNode, double d1)
{
	// through cubic curvature
	// // initilization
	// vector<PCTrajNode> result;
	// double x0 = 0.0;
	// double y0 = 0.0;
	// double theta0 = 0.0;
	// double kappa0 = fromNode.kappa;
	// Vector4d init(x0, y0, theta0, kappa0);
	// MatrixXd T_b = fromNode.T.inverse()*toNode.T;
	// double xf = T_b.row(0)[3];
	// double yf = T_b.row(1)[3];

	// if (T_b.row(0)[3] < 0)
	// {
	// 	return result;
	// }

	// double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);

	// if (fabs(thetaf) > 1)
	// {
	// 	return result;
	// }

	// double kappaf = toNode.kappa;
	// Vector4d ends(xf, yf, thetaf, kappaf);
	// double dsg = hypot(xf, yf);

	// if (dsg >= d1)
	// {
	// 	return result;
	// }
	
	// //cout << "get_cubic_curvature" << endl;
	// Vector4d srt = get_cubic_curvature(init, ends);
	// if (srt == Vector4d::Zero())
	// {
	// 	result.clear();
	// 	return result;
	// }
	// Vector4d x(srt);
	// for (double seg = 0.0; seg < srt[3]; seg+=PR.d_nom)
	// {
	// 	x[3] = seg;
	// 	Vector4d path_seg = forward(init, x);
	// 	Matrix4d T_seg = Matrix4d::Identity();
	// 	T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
	// 	T_seg.row(0)[1] = -sin(path_seg[2]);
	// 	T_seg.row(1)[0] = sin(path_seg[2]);
	// 	/*if (T_b.row(0)[3] < 0)
	// 	{
	// 		T_seg.row(0)[3] = -path_seg[0];
	// 		T_seg.row(1)[3] = -path_seg[1];
	// 	}
	// 	else
	// 	{*/
	// 		T_seg.row(0)[3] = path_seg[0];
	// 		T_seg.row(1)[3] = path_seg[1];
	// 	//}
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



	// through OBVP
	// initilization
	vector<PCTrajNode> result;
	double kappa0 = fromNode.kappa;
	MatrixXd T_b = fromNode.T.inverse()*toNode.T;
	double xf = T_b.row(0)[3];
	double yf = T_b.row(1)[3];

	if (T_b.row(0)[3] < 0||fabs(yf/xf)>1)
	{
		return result;
	}

	double thetaf = atan2(T_b.row(1)[0], T_b.row(0)[0]);

	if (fabs(thetaf) > 1)
	{
		return result;
	}

	double kappaf = toNode.kappa;
	double dsg = hypot(xf, yf);

	if (dsg >= d1)
	{
		return result;
	}
	
	// close-formed solution
	double T1 = xf;
	double T2 = T1*T1;
	double T3 = T2*T1;
	double T4 = T3*T1;
	double T5 = T4*T1;
	Vector4d srt;
	srt[2] = 10*(kappaf*T2-6*thetaf*T1+12*yf)/T5;
	srt[1] = -6*(30*yf-14*T1*thetaf-T2*kappa0+2*T2*kappaf)/T4;
	srt[0] = 3*(20*yf-8*T1*thetaf-2*T2*kappa0+T2*kappaf)/T3;
	srt[3] = T1;

	if (srt == Vector4d::Zero())
	{
		result.clear();
		return result;
	}

	// check curvature constraint
	double x0, x1,max_kappa=0;
	int r = gsl_poly_solve_quadratic(srt[2]*3, srt[1]*2, srt[0],&x0,&x1 );
	switch (r)
	{
		case 0: break;
		case 1: 
			if (x0>0 && x0<srt[3])
			{
				for (int j = 3; j >=0; j--)
				{
					max_kappa *= x0;
					max_kappa += srt[j];
				}
				max_kappa = fabs(max_kappa);
			}
			break;
		case 2:
			if (x0>0 && x0<srt[3])
			{
				for (int j = 3; j >=0; j--)
				{
					max_kappa *= x0;
					max_kappa += srt[j];
				}
			}
			if (x1>0 && x1<srt[3])
			{
				double kap=0;
				for (int j = 3; j >=0; j--)
				{
					kap *= x1;
					kap += srt[j];
				}
				max_kappa = max(fabs(max_kappa), fabs(kap));
			}
			break;
		default: max_kappa = 10; cout<<"gsl error!"<<endl;break;
	}

	if (max_kappa>PCTrajNode::PR.kappa_max)
	{
		result.clear();
		return result;
	}

	// forward and cast
	Vector4d x(srt);
	for (double seg = 0.0; seg < srt[3]; seg+=PR.d_nom)
	{
		x[3] = seg;
		Vector4d path_seg;// = forward(init, x);
		double seg2 = seg*seg;
		double seg3 = seg*seg2;
		double seg4 = seg*seg3;
		double seg5 = seg*seg4;
		path_seg[0] = seg;
		path_seg[1] = kappa0*seg2/2+ srt[0]*seg3/6+srt[1]*seg4/12+srt[2]*seg5/20;
		path_seg[2] = kappa0*seg+ srt[0]*seg2/2+srt[1]*seg3/3+srt[2]*seg4/4;
		path_seg[3] = kappa0 + srt[0]*seg+srt[1]*seg2+srt[2]*seg3;
		Matrix4d T_seg = Matrix4d::Identity();
		T_seg.row(0)[0] = T_seg.row(1)[1] = cos(path_seg[2]);
		T_seg.row(0)[1] = -sin(path_seg[2]);
		T_seg.row(1)[0] = sin(path_seg[2]);
		T_seg.row(0)[3] = path_seg[0];
		T_seg.row(1)[3] = path_seg[1];

		PCTrajNode temp(fromNode.T*T_seg, path_seg[3]);
		if (temp.isTraversable())
		{
			result.push_back(temp);
		}
		else
		{
			result.clear();
			return result;
		}
	}

	result.push_back(toNode);

	// connect and check whether kappa is valid
	bool kap_ok = true;
	for (size_t i = 0; i < result.size() - 1; i++)
	{
		VectorXd pol = result[i].connect(result[i + 1]);
		if (pol == VectorXd::Zero(5))
		{
			result.clear();
			return result;
		}
		double x0, x1,max_kappa=0;
		int r = gsl_poly_solve_quadratic(pol[3]*3, pol[2]*2,pol[1],&x0,&x1 );
		switch (r)
		{
			case 0: break;
			case 1: 
				if (x0>0 && x0<pol[4])
				{
					for (int j = 3; j >=0; j--)
					{
						max_kappa *= x0;
						max_kappa += pol[j];
					}
					max_kappa = fabs(max_kappa);
				}
				break;
			case 2:
				if (x0>0 && x0<pol[4])
				{
					for (int j = 3; j >=0; j--)
					{
						max_kappa *= x0;
						max_kappa += pol[j];
					}
				}
				if (x1>0 && x1<pol[4])
				{
					double kap=0;
					for (int j = 3; j >=0; j--)
					{
						kap *= x1;
						kap += pol[j];
					}
					max_kappa = max(fabs(max_kappa), fabs(kap));
				}
				break;
			default: max_kappa = 10; cout<<"gsl error!"<<endl;break;
		}

		if (max_kappa>PCTrajNode::PR.kappa_max)
		{
			result.clear();
			return result;
		}
		// for (double dist = 0.0; dist < pol[4]; dist += 0.01)
		// {
		// 	double kap = 0;
		// 	for (int j = 3; j >=0; j--)
		// 	{
		// 		kap *= dist;
		// 		kap += pol[j];
		// 	}
		// 	if (fabs(kap) > PR.kappa_max)
		// 	{
		// 		//cout << "kappa = " << kap << " to high!" << endl;
		// 		kap_ok = false;
		// 		break;
		// 	}
		// }
		// if (!kap_ok)
		// {
		// 	result.clear();
		// 	return result;
		// }
	}

	// cout<<"xf="<<T1<<endl;
	// cout<<"yf="<<yf<<endl;
	return result;
}

pair<bool, double> PCTrajNode::get_rho(pcl::PointXYZI& p, pcl::PointCloud<pcl::PointXYZI>::Ptr kdmap, pcl::KdTreeFLANN<pcl::PointXYZI> kd)
{
	if (p.intensity != -1)
	{
		if (p.intensity < 0)
			return { true,-p.intensity };
		else
			return { false,p.intensity };
	}
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

	// get rho
	std::vector<int> Idx_res;
	std::vector<float> Dist_res;
	kd.radiusSearch(p, PR.r_res, Idx_res, Dist_res);
	vector<double> nears;
	for (auto& i : Idx_res)
	{
		Matrix<double, 3, 1> pi = { kdmap->points[i].x, \
			kdmap->points[i].y, \
			kdmap->points[i].z };
		nears.push_back(n.transpose().dot(pi - mean_p));
	}
	sort(nears.begin(), nears.end());

	size_t a = static_cast<double>(nears.size()*PR.eta / 2.0);
	double pd = n.transpose().dot(Matrix<double, 3, 1>{p.x, p.y, p.z} -mean_p);
	double d_min = nears[a];
	double d_max = nears[nears.size() - 1 - a];
	pair<bool, double> result{ false,fabs(d_max - d_min) };

	if (d_max <= pd || pd <= d_min)
	{
		result.first = true;
		p.intensity = -result.second;
	}
	else
	{
		p.intensity = result.second;
	}

	return result;
}

void pubProjectedPose(ros::Publisher *pubPtr, PCTrajNode node)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id="/map";
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
