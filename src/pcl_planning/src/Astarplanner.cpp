#include "Astarplanner.h"

bool Astarplanner::AstarSearch(const PCTrajNode& start, const PCTrajNode& goal)
{
	pcl::StopWatch time;
	
	AstarNodePtr startPtr = new AstarNode(start);
	AstarNodePtr endPtr = new AstarNode(goal);
	astar_map.push_back(startPtr);
	astar_map.push_back(endPtr);

	priority_queue<AstarNodePtr, std::vector<AstarNodePtr>, NodeComparator> empty;
	openSet.swap(empty);

	AstarNodePtr neighborPtr = NULL;
	AstarNodePtr current = NULL;

	startPtr->gScore = 0;
	startPtr->fScore = getHeu(startPtr, endPtr);
	openSet.push(startPtr);

	int iter = 0;
	while (!openSet.empty())
	{
		iter++;
		current = openSet.top();
		cout << "iteration=" << iter << endl;
		cout << "gscore=" << current->gScore << endl;
		openSet.pop();

		//pcl::StopWatch ti;
		//current->node.ter_assess();
		//cout << "time consume: " << ti.getTime() << "ms" << std::endl;
		//if (!current->node.isTraversable())
			//continue;

		if (current->fScore - 0.1*current->gScore < PCTrajNode::PR.d_nom * 3)
		{
			//cout << "try connect!" << endl;
			vector<PCTrajNode> srt = PCTrajNode::SRT_Generate(current->node, endPtr->node);
			if (!srt.empty())
			{
				for (size_t i = 1; i < srt.size() - 1; i++)
				{
					AstarNodePtr srtp = new AstarNode(srt[i]);
					srtp->cameFrom = current;
					current = srtp;
				}
				endPtr->cameFrom = current;
				cout << "get path in  iteration: " << iter << endl;
				cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
				astar_path = retrievePath(endPtr);
				return true;
			}
			//cout << "get path in  iteration: " << iter << endl;
			//cout << "time consume: " << time.getTime() / 1000 << "s" << std::endl;
			//endPtr->cameFrom = current;
			//astar_path = retrievePath(endPtr);
			//return true;
		}

		for (int i = 0; i < 13; i++)
		{
			Matrix4d T_i = current->node.Trans(traj_lib[i]);
			PCTrajNode pc_new(T_i, false);
			if (pc_new.isTraversable())
			{
				AstarNodePtr neighborPtr = new AstarNode(pc_new);
				astar_map.push_back(neighborPtr);

				neighborPtr->cameFrom = current;
				neighborPtr->gScore = current->gScore + traj_cost[i] + PCTrajNode::PR.d_nom - PCTrajNode::PR.d_nom * pc_new.get_tau();
				//pcl::StopWatch ti;
				neighborPtr->fScore = 0.1*neighborPtr->gScore + getHeu(neighborPtr, endPtr);
				//cout << "Heu time consume: " << ti.getTime() << "ms" << std::endl;
				openSet.push(neighborPtr);
			}
		}
	}

	return false;
}

vector<AstarNodePtr> Astarplanner::retrievePath(AstarNodePtr current)
{
	vector<AstarNodePtr> path;
	path.push_back(current);

	while (current->cameFrom != NULL)
	{
		current = current->cameFrom;
		path.push_back(current);
	}

	return path;
}

vector<PCTrajNode> Astarplanner::getPath()
{
	vector<PCTrajNode> path;

	for (auto ptr : astar_path)
	{
		path.push_back(ptr->node);
		cout << ptr->node.get_pos()[0] << " " << ptr->node.get_pos()[1] << " " << ptr->node.get_pos()[2] << endl;
	}
	reverse(path.begin(), path.end());
	return path;
}

double Astarplanner::getObvpHeu(AstarNodePtr node1, AstarNodePtr node2)
{
	VectorXd C = PCTrajNode::get_connect(node1->node, node2->node);
	//return 0.9*getEuclHeu(node1, node2);
	if (C[4] == 0)
	{
		//cout << "dist=" << 0.9*getEuclHeu(node1, node2) << endl;
		return 0.9*getEuclHeu(node1, node2);
	}
	else
	{
		//cout << "cc=" << getEuclHeu(node1, node2) << endl;
		//cout << "c=" << C[4] << endl;
		return 0.1*C[4] + 0.81*getEuclHeu(node1, node2);
	}
	return 0;
}