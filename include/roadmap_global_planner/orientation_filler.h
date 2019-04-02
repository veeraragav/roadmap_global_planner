
#include <nav_msgs/Path.h>
#include <iostream>
#include <ros/console.h>
namespace roadmap_global_planner {

enum OrientationMode { NONE, FORWARD, INTERPOLATE, FORWARDTHENINTERPOLATE, BACKWARD, LEFTWARD, RIGHTWARD };

class OrientationFiller {
    public:
        OrientationFiller() : omode_(FORWARD) {
            ROS_INFO("Or Cons");
        }
    
    
        virtual void processPath(const geometry_msgs::PoseStamped& start,
                                 std::vector<geometry_msgs::PoseStamped>& path);
                                 
        void setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path, int index);
        void interpolate(std::vector<geometry_msgs::PoseStamped>& path, 
                         int start_index, int end_index);
                         
        void setMode(OrientationMode new_mode){ omode_ = new_mode; }
        void setMode(int new_mode){ setMode((OrientationMode) new_mode); }

        void setWindowSize(size_t window_size){ window_size_ = window_size; }
    protected:
        OrientationMode omode_;
        int window_size_;
};

} //end namespace global_planner

