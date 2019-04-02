#include <roadmap_global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(roadmap_global_planner::RoadmapGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace roadmap_global_planner {


RoadmapGlobalPlanner::RoadmapGlobalPlanner() :
        costmap_(NULL), initialized_(false) {
   
}

RoadmapGlobalPlanner::RoadmapGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false) {

    //initialize the planner
    initialize(name, costmap, frame_id);
}

RoadmapGlobalPlanner::~RoadmapGlobalPlanner() {

}

void RoadmapGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {

    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void RoadmapGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {

    if ( !initialized_ ) {
    
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        //get the tf prefix
         ros::NodeHandle prefix_nh;
         tf_prefix_ = tf::getPrefixParam(prefix_nh);
          plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
         marker_pub = private_nh.advertise<visualization_msgs::Marker>("waypoint_marker", 1);
         graph_srv = private_nh.advertiseService("getRoadmap", &RoadmapGlobalPlanner::updateGraphService, this);
        orientation_filler_.setMode(1); //forward orientation
        initialized_ = true;
   
        
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

  double RoadmapGlobalPlanner::distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
  {
    return hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
  }


bool RoadmapGlobalPlanner::updateGraphService(roadmap_global_planner::GraphRoadmap::Request& req, roadmap_global_planner::GraphRoadmap::Response& resp){

    roadMap_ = req.adjacency_list;  
    planner_.setGraph(roadMap_);
    publishWP(roadMap_.map);
    //std::cout<<req;
    return 1;
}

void RoadmapGlobalPlanner::publishWP(const std::vector<roadmap_global_planner_msgs::MapPair>& WP){
    for(int i = 0; i < WP.size(); i++){
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
    
        marker.id = i;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::Marker::SPHERE;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = WP[i].pose.position.x;
        marker.pose.position.y = WP[i].pose.position.y;
        marker.pose.position.z = WP[i].pose.position.z;
        marker.pose.orientation.x = WP[i].pose.orientation.x;
        marker.pose.orientation.y = WP[i].pose.orientation.y;
        marker.pose.orientation.z = WP[i].pose.orientation.z;
        marker.pose.orientation.w = WP[i].pose.orientation.w;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        std::stringstream ss;
        ss<<"Waypoint_"<<i;
        std::string str = ss.str();
        marker.text = str;
        ros::Duration seconds(600.0);
        marker.lifetime = seconds;
        marker_pub.publish(marker);
    }
}

int RoadmapGlobalPlanner::findNearestNode(geometry_msgs::Pose p){
    int dist = 999999;
    int node = 999999;
    for (int i = 0; i < roadMap_.map.size(); i++ ){
        if(distance(p, roadMap_.map[i].pose) < dist){
            dist = distance(p, roadMap_.map[i].pose);
            node = roadMap_.map[i].node;
        }
    }
    return node;
}

bool RoadmapGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    unsigned int start_node = findNearestNode(start.pose);
    unsigned int goal_node = findNearestNode(goal.pose);

    std::vector<int> nodePath = planner_.getPlan (start_node, goal_node);


    for(int i = 0; i< nodePath.size(); i++)
    std::cout<<nodePath[i]<<std::endl;

    std::vector<geometry_msgs::PoseStamped> path;

    ////////////////////////////////////////////////
    // geometry_msgs::Pose  q;
    // planner_.nodeToPose(nodePath[0], q);
    // double magn, vector[2];
    // magn = distance(q, start.pose);
    // vector[0] = (q.position.x - start.pose.position.x) / magn ;
    // vector[1] = (q.position.y - start.pose.position.y) / magn ;
    // geometry_msgs::PoseStamped point;
    // point.pose = start.pose;
    // point.header.frame_id = frame_id_;
    // point.header.stamp = ros::Time::now();
    // while(distance(point.pose, q) > 0.01){
    //         path.push_back(point);
    //         point.pose.position.x += 0.01 * vector[0];
    //         point.pose.position.y += 0.01 * vector[1];
    //         point.header.stamp = ros::Time::now();
    //         point.pose.orientation.x = 0.0;
    //         point.pose.orientation.y = 0.0;
    //         point.pose.orientation.z = 0.0;
    //         point.pose.orientation.w = 1.0;
    //    }
    








    /////////////////////////////////////////////

    for(int i = 0; i < nodePath.size() - 1; i++){
        int a = nodePath[i];
        int b = nodePath[i+1];
        geometry_msgs::Pose start, stop;
        planner_.nodeToPose(a, start);
        planner_.nodeToPose(b, stop); 
        double vector[2], mag;
        mag = distance(start, stop);
        vector[0] = (stop.position.x - start.position.x) / mag ;
        vector[1] = (stop.position.y - start.position.y) / mag ;
        geometry_msgs::PoseStamped point;
        point.pose = start;
        point.header.frame_id = frame_id_;
        point.header.stamp = ros::Time::now();

        while(distance(point.pose, stop) > 0.01){
            path.push_back(point);
            point.pose.position.x += 0.01 * vector[0];
            point.pose.position.y += 0.01 * vector[1];
            point.header.stamp = ros::Time::now();
            point.pose.orientation.x = 0.0;
            point.pose.orientation.y = 0.0;
            point.pose.orientation.z = 0.0;
            point.pose.orientation.w = 1.0;
        }

    }
    geometry_msgs::PoseStamped Goal;
    Goal = goal;
    Goal.header.stamp = ros::Time::now();
    path.push_back(Goal);
    plan = path;

//fill orientation
    orientation_filler_.processPath(start,  plan);
    
    
    //publish the plan for visualization purposes
    publishPlan(path);


    return 1;
}


void RoadmapGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

} //end namespace roadmap_global_planner


