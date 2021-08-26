#pragma once
#include "PCTrajNode.h"

constexpr double inf = 1 >> 20;
struct AstarNode;
typedef AstarNode *AstarNodePtr;

struct AstarNode
{
	PCTrajNode node;
	double gScore{ inf }, fScore{ inf };
	AstarNodePtr cameFrom{ NULL };

	AstarNode() {}
	AstarNode(PCTrajNode n) :node(n) {}
	~AstarNode() {}
};

class NodeComparator
{
public:
	bool operator()(AstarNodePtr node1, AstarNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class Astarplanner
{
private:
	vector<AstarNodePtr> astar_path;
	vector<AstarNodePtr> astar_map;
	priority_queue<AstarNodePtr, vector<AstarNodePtr>, NodeComparator> openSet;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;
	double getEuclHeu(AstarNodePtr node1, AstarNodePtr node2) { return node1->node.get_dist(node2->node); }
	double getObvpHeu(AstarNodePtr node1, AstarNodePtr node2);
	inline double getHeu(AstarNodePtr node1, AstarNodePtr node2){ return tie_breaker_ * getObvpHeu(node1, node2); }
	Vector3d traj_lib[13] = { {0.6,0,0}, {0.599250156236980, 0.029987501562407,0.1}, \
	{0.599250156236980, -0.029987501562407, -0.1}, { 0.597002499166815  , 0.059900049988097 ,0.2},\
	{ 0.597002499166815, -0.059900049988097, -0.2}, { 0.593262646761625 ,  0.089662879484160 ,0.3},\
	{ 0.593262646761625, -0.089662879484160, -0.3}, { 0.588039946704745 ,  0.119201598477037,0.4 },\
	{ 0.588039946704745, -0.119201598477037, -0.4 }, { 0.581347453026387 ,  0.148442375552714 ,0.5},\
	{ 0.581347453026387, -0.148442375552714, -0.5}, { 0.573201893475364 ,  0.177312123996804 ,0.6},\
	{ 0.573201893475364, -0.177312123996804, -0.6} };		//x, y, theta
	double traj_cost[14] = { 0.6,0.600364448468052 ,0.600364448468052 ,\
		0.601458896685019 , 0.601458896685019, 0.603283799402430, 0.603283799402430,\
		0.605833261322673 , 0.605833261322673 , 0.609086090310233 , 0.609086090310233 ,\
		0.612995520383789, 0.612995520383789, 0 };		//sf
	
public:
	Astarplanner() {
		for (auto i = 0; i < 13; i++) {
			traj_lib[i] *= 0.25;
			traj_cost[i] *= 0.25;
			traj_lib[i][2] *= 4;
		}
	}
	~Astarplanner() {
		for (auto p : astar_map)
			delete p;
	}
	bool AstarSearch(const PCTrajNode& start, const PCTrajNode& goal);
	vector<PCTrajNode> getPath();
	vector<AstarNodePtr> retrievePath(AstarNodePtr current);
};

