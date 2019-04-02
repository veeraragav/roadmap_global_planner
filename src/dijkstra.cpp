#include <roadmap_global_planner/dijkstra.h>
#include <iterator>
#include <limits>
#include <ros/console.h>
namespace roadmap_global_planner {


bool dijkstra::setGraph(roadmap_global_planner_msgs::AdjacencyList graph){
graph_ = graph;
return true;
}

double dijkstra::getCost(int a, int b){
    geometry_msgs::Pose p1, p2;
    nodeToPose(a, p1);
    nodeToPose(b, p2);
   return distance(p1, p2);
}

void dijkstra::nodeToPose(int node, geometry_msgs::Pose& pose){
    pose = graph_.map[node].pose;
}

void dijkstra::poseToNode(geometry_msgs::Pose pose, int& node){

    std::vector<roadmap_global_planner_msgs::MapPair>::iterator ptr;
    ptr = graph_.map.begin();
    for(; ptr!= graph_.map.end(); ptr++){
        if(distance(pose,(ptr->pose) ) < 0.01) node = ptr->node;
    }
}

double dijkstra::distance(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2){
    return hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
}

int dijkstra::findMinCostUnvisitedNode(std::vector<std::vector<double> > mat){
    int small = 99999;
    int node = 99999;
    for(int i = 0; i<mat.size(); i++){
        if (mat[i][1] < small) {
            if(isUnvisited(mat[i][0]))
            {
            small = mat[i][1];
            node = mat[i][0];
            }
        }
    }
    return node;
}

bool dijkstra::isUnvisited(int vertex){
    if ( std::find(visited.begin(), visited.end(), vertex) != visited.end() )
   return 0;
else
  return 1;
}


std::vector<int> dijkstra::traceParent (int goal, int start){
    int parent = goal;
    std::vector<int> path;
    path.push_back(goal);
 
    int i = 0;
    while(parent != start){
       
        parent = matrix[parent][2];
        path.push_back(parent);
    }
    //path.push_back(start);
    std::reverse(path.begin(), path.end());
 
    return path;
}

std::vector<int> dijkstra::getPlan (int start, int goal){
    //populate table
    matrix.clear();
    visited.clear();

    for(int i = 0; i < graph_.map.size(); i++){
        std::vector<double> row;
        row.resize(3);
        row[0] = (double)i;
        if(i == start) row[1] = 0;
        else row[1] = std::numeric_limits<double>::max();
        row[2] = std::numeric_limits<double>::max();
        matrix.push_back(row);
    }
  
    // Dijkstra's Algo

    while(visited.size() < graph_.adj_list.size()){
        int current_vertex = findMinCostUnvisitedNode(matrix);
        visited.push_back(current_vertex);
        for(unsigned int i =0; i<graph_.adj_list[current_vertex].list.size(); i++){
            if(isUnvisited(graph_.adj_list[current_vertex].list[i])){
                // do cost
              double newCost = getCost(current_vertex, graph_.adj_list[current_vertex].list[i]) + matrix[current_vertex][1];
              if(matrix[graph_.adj_list[current_vertex].list[i]][1] > newCost){
                  matrix[graph_.adj_list[current_vertex].list[i]][1] = newCost;
                  matrix[graph_.adj_list[current_vertex].list[i]][2] = current_vertex;
              }
            }
        }
        
    }

    //traceback


   return traceParent(goal, start);

}



}