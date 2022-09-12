#include <iostream>
// #include <conio.h>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "PCTrajNode.h"
#include "RRT.h"
// #include "RRT_star.h"
// #include "LocalOptimizer.h"
#include "Astarplanner.h"

using namespace std;
using namespace Eigen;

pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

int main()
{
	srand((unsigned)time(0));
	/*vector<int> a{ 1,2,3,4,5,5,6,7,7,9,10 };
	for (auto p = a.begin(); p != a.end() - 1; p++)
	{
		if (*p == 5)
		{
			p=a.insert(p+1, 11);
			p--;
			cout << *p << endl;
			cout << *(p + 1) << endl;
		}
	}*/
	/*Vector4d  init(0, 0, 0, 1.54694);
	Vector4d ends(0.430961,0.344747, 1.19971,0.277024);
	cout << get_cubic_curvature(init, ends) << endl;
	return 0;*/

	// read point cloud
	typedef struct tagPOINT_3D
	{
		double x;  //mm world coordinate x  
		double y;  //mm world coordinate y  
		double z;  //mm world coordinate z  
		double r; 
	}POINT_WORLD;

	int number_Txt;
	FILE *fp_txt;
	tagPOINT_3D TxtPoint;
	vector<tagPOINT_3D> m_vTxtPoints;
	fp_txt = fopen("test.txt", "r");
	int cnt = 0;
	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			cnt++;
			/*if (cnt > 1010)
			{
				break;
			}*/
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "txt���ݼ���ʧ�ܣ�" << endl;
	number_Txt = m_vTxtPoints.size();








	PCTrajNode::PCMap->width = number_Txt;
	PCTrajNode::PCMap->height = 1;
	PCTrajNode::PCMap->is_dense = false;
	PCTrajNode::PCMap->points.resize(PCTrajNode::PCMap->width * PCTrajNode::PCMap->height);
	for (size_t i = 0; i < PCTrajNode::PCMap->points.size(); ++i)
	{
		PCTrajNode::PCMap->points[i].x = m_vTxtPoints[i].x * 100;
		PCTrajNode::PCMap->points[i].y = m_vTxtPoints[i].y * 100;
		PCTrajNode::PCMap->points[i].z = m_vTxtPoints[i].z * 100;
		PCTrajNode::PCMap->points[i].intensity = -1;

	}
	
	kdtree.setInputCloud(PCTrajNode::PCMap);

	// terrain assessment
	/*vector<double> rho;
	for (size_t i = 0; i < PCTrajNode::PCMap->points.size(); i++)
	{
		pair<bool, double> temp = PCTrajNode::get_rho(PCTrajNode::PCMap->points[i], PCTrajNode::PCMap,kdtree);
		rho.push_back(temp.second);
	}
	for (auto& i : rho)
	{
		if (i > 0.08)
		{
			i = 0.08;
		}
		cout << i << endl;
	}
	double maxr = *max_element(rho.begin(), rho.end());
	double minr = *min_element(rho.begin(), rho.end());
	for (size_t i=0;i<PCTrajNode::PCMap->points.size();i++)
	{
		int a = (int)floor((maxr - rho[i]) / (maxr - minr) * 255.0);
		PCTrajNode::PCMap->points[i].intensity = a;
	}*/

	// init start and goal
	///rabbit
	Vector3d start(-7.7, 9.69, 3.663);
	Vector3d goal(2.075, 10.1, 4.528);
	//Vector3d goal(3.275, 5.977, 3.952);
	///dragon
	/*Vector3d start(-4.172, 13.63, 0.062);
	Vector3d goal(5.446, 12.07, -0.2122);*/
	Matrix4d st = Matrix4d::Identity();
	Matrix4d gt = Matrix4d::Identity();;
	/*gt << 0, 1, 0,0,\
		-1, 0, 0, 0,\
		0, 0, 1,0,\
		0,0,0,1;*/
	st.topRightCorner(3, 1) = Matrix<double, 3, 1>(start);
	gt.topRightCorner(3, 1) = Matrix<double, 3, 1>(goal);
	pcl::StopWatch time;
	PCTrajNode s(st);
	cout << "constructor once consume:" << time.getTime() << "ms" << std::endl;
	PCTrajNode g(gt);

	// launch planner
	//LocalOptimizer planner;
	Astarplanner planner;
	vector<PCTrajNode> path;
	path = planner.optimized_path(s, g);
	// 
	if (planner.AstarSearch(s, g))
	{
		path = planner.getPath();
	}
	

	// visulization
	/*pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(PCTrajNode::PCMap, "intensity");
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloud(PCTrajNode::PCMap,fildColor, "sample cloud");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.spin();
	system("pause");*/
	return (0);
}

//test gsl
//#include <iostream>
//#include <cmath>
//#include <gsl/gsl_sf.h>
//#include <gsl/gsl_integration.h>
//#include "PCTrajNode.h"
//#include "RRT.h"
//#include "RRT_star.h"
//#include "LocalOptimizer.h"
//pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
//
//double f(double x, void * params) {
//	double* alpha = (double *)params;
//	double a = alpha[0];
//	double b = alpha[1];
//	double c = alpha[2];
//	double kappa = alpha[3];
//	double theta = alpha[4];
//	double f = theta + kappa * x + a * x*x / 2 + b * pow(x, 3) / 3 + c * pow(x, 4) / 4;
//
//	return cos(f);
//}
//
//
//int main()
//
//{
//
//	gsl_integration_workspace * w
//		= gsl_integration_workspace_alloc(1000);
//
//	double result, error;
//	double expected = 0.290879242285901;
//	double alpha[5] = { 67.8973613365547,-292.8943271995715,292.9903274469080,0,0 };
//
//	gsl_function F;
//	F.function = &f;
//	F.params = (void*)alpha;
//
//	gsl_integration_qag(&F, 0, 0.38, 0, 1e-10, 1e-10, 1000, \
//		w, &result, &error);
//
//	printf("result          = % .18f\n", result);
//	printf("exact result    = % .18f\n", expected);
//	printf("estimated error = % .18f\n", error);
//	printf("actual error    = % .18f\n", result - expected);
//	printf("intervals =  %d\n", w->size);
//
//	return 0;
//
//}
