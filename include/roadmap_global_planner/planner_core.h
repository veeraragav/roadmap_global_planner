#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <std_msgs/String.h>
#include <roadmap_global_planner/GraphRoadmap.h>
#include <geometry_msgs/Pose.h>
#include <roadmap_global_planner/dijkstra.h>
#include <roadmap_global_planner_msgs/AdjacencyList.h>
#include <map>
#include <roadmap_global_planner/orientation_filler.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <sstream>
namespace roadmap_global_planner {

class RoadmapGlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
       
        RoadmapGlobalPlanner();

        RoadmapGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        ~RoadmapGlobalPlanner();


        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        bool updateGraphService(roadmap_global_planner::GraphRoadmap::Request& req, roadmap_global_planner::GraphRoadmap::Response& resp);

        double distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);

        int findNearestNode(geometry_msgs::Pose p);
        ros::Publisher plan_pub_;
        ros::Publisher marker_pub;

    private:
        ros::ServiceServer graph_srv;
        roadmap_global_planner_msgs::AdjacencyList roadMap_;
        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        std::string tf_prefix_;
        bool initialized_;
        dijkstra planner_;
        OrientationFiller orientation_filler_;
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        void publishWP(const std::vector<roadmap_global_planner_msgs::MapPair>& WP);
        

};

} //end namespace roadmap_global_planner

#endif
