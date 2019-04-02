#ifndef ROADMAP_GLOBAL_PLANNER_DIJKSTRA_H
#define ROADMAP_GLOBAL_PLANNER_DIJKSTRA_H

#include <vector>
#include <geometry_msgs/Pose.h>
#include <map>
#include <roadmap_global_planner_msgs/AdjacencyList.h>
#include <algorithm>
#include <iterator>
namespace roadmap_global_planner {


class dijkstra
{
public:

	std::vector<int> getPlan (int start, int goal);
	bool setGraph(roadmap_global_planner_msgs::AdjacencyList graph);
	double getCost(int a, int b);
	void nodeToPose(int node, geometry_msgs::Pose& pose);
	void poseToNode(geometry_msgs::Pose pose, int& node);

private:
	roadmap_global_planner_msgs::AdjacencyList graph_;
	std::vector<std::vector<double> > matrix;
	std::vector<int> visited;
	double distance(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2);
	int findMinCostUnvisitedNode(std::vector<std::vector<double> > mat);
	bool isUnvisited(int vertex);
	std::vector<int> traceParent(int goal, int start);
	
};

}
#endif