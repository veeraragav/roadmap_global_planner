#include "ros/ros.h"
#include "roadmap_global_planner/GraphRoadmap.h"
#include <cstdlib>
#include <geometry_msgs/Pose.h>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_UI");


  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<roadmap_global_planner::GraphRoadmap>("/move_base/RoadmapGlobalPlanner/getRoadmap");

  roadmap_global_planner::GraphRoadmap srv;

  roadmap_global_planner_msgs::MapPair pair;
  std::vector<roadmap_global_planner_msgs::MapPair> mapPair;

  // pair.node = 0;
  // geometry_msgs::Pose pose;
  // pose.position.x = 4;
  // pose.position.y = -2;
  // pose.position.z = 0;  
  // pose.orientation.x = 0;
  // pose.orientation.y = 0;
  // pose.orientation.z = 0;
  // pose.orientation.w = 1;
  // pair.pose = pose;
  // mapPair.push_back(pair);

  // pair.node = 1;
  // pose.position.x = 4;
  // pose.position.y = 2;
  // pose.position.z = 0;  
  // pose.orientation.x = 0;
  // pose.orientation.y = 0;
  // pose.orientation.z = 0;
  // pose.orientation.w = 1;
  // pair.pose = pose;
  // mapPair.push_back(pair);

  // pair.node = 2;
  // pose.position.x = -4;
  // pose.position.y = 2;
  // pose.position.z = 0;  
  // pose.orientation.x = 0;
  // pose.orientation.y = 0;
  // pose.orientation.z = 0;
  // pose.orientation.w = 1;
  // pair.pose = pose;
  // mapPair.push_back(pair);

  // pair.node = 3;
  // pose.position.x = -4;
  // pose.position.y = -2;
  // pose.position.z = 0;  
  // pose.orientation.x = 0;
  // pose.orientation.y = 0;
  // pose.orientation.z = 0;
  // pose.orientation.w = 1;
  // pair.pose = pose;
  // mapPair.push_back(pair);

  //   pair.node = 4;
  // pose.position.x = 0;
  // pose.position.y = 0;
  // pose.position.z = 0;  
  // pose.orientation.x = 0;
  // pose.orientation.y = 0;
  // pose.orientation.z = 0;
  // pose.orientation.w = 1;
  // pair.pose = pose;
  // mapPair.push_back(pair);


pair.node = 0;
  geometry_msgs::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;  
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  pair.pose = pose;
  mapPair.push_back(pair);

  pair.node = 1;
  pose.position.x = 10;
  pose.position.y = 0;
  pose.position.z = 0;  
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  pair.pose = pose;
  mapPair.push_back(pair);

  pair.node = 2;
  pose.position.x = 10;
  pose.position.y = 13.3;
  pose.position.z = 0;  
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  pair.pose = pose;
  mapPair.push_back(pair);

  pair.node = 3;
  pose.position.x = -9;
  pose.position.y = 13.3;
  pose.position.z = 0;  
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  pair.pose = pose;
  mapPair.push_back(pair);

    pair.node = 4;
  pose.position.x = -9;
  pose.position.y = 0;
  pose.position.z = 0;  
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  pair.pose = pose;
  mapPair.push_back(pair);


  srv.request.adjacency_list.map = mapPair;

  roadmap_global_planner_msgs::PointList pList;

  // pList.list.push_back(0);
  // pList.list.push_back(1);
  // pList.list.push_back(3);
  // srv.request.adjacency_list.adj_list.push_back(pList);
  // pList.list.clear();

  // pList.list.push_back(1);
  // pList.list.push_back(2);
  // pList.list.push_back(0);
  // srv.request.adjacency_list.adj_list.push_back(pList);
  // pList.list.clear();

  // pList.list.push_back(2);
  // pList.list.push_back(3);
  // pList.list.push_back(1);
  // srv.request.adjacency_list.adj_list.push_back(pList);
  // pList.list.clear();

  // pList.list.push_back(3);
  // pList.list.push_back(4);
  // pList.list.push_back(2);
  // srv.request.adjacency_list.adj_list.push_back(pList);
  // pList.list.clear();

  //   pList.list.push_back(4);
  // pList.list.push_back(3);
  // pList.list.push_back(0);
  // srv.request.adjacency_list.adj_list.push_back(pList);
  // pList.list.clear();

  pList.list.push_back(0);
  pList.list.push_back(1);
  pList.list.push_back(4);
  srv.request.adjacency_list.adj_list.push_back(pList);
  pList.list.clear();

  pList.list.push_back(1);
  pList.list.push_back(2);
  pList.list.push_back(0);
  srv.request.adjacency_list.adj_list.push_back(pList);
  pList.list.clear();

  pList.list.push_back(2);
  pList.list.push_back(3);
  pList.list.push_back(1);
  srv.request.adjacency_list.adj_list.push_back(pList);
  pList.list.clear();

  pList.list.push_back(3);
  pList.list.push_back(4);
  pList.list.push_back(2);
  srv.request.adjacency_list.adj_list.push_back(pList);
  pList.list.clear();

    pList.list.push_back(4);
  pList.list.push_back(3);
  pList.list.push_back(0);
  srv.request.adjacency_list.adj_list.push_back(pList);
  pList.list.clear();
  


 
  if (client.call(srv))
  {
    ROS_INFO("Called Service");
  }
  else
  {
    ROS_ERROR("Failed to call service ");
    return 1;
  }

  return 0;
}