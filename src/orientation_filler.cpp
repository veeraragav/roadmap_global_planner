
#include <roadmap_global_planner/orientation_filler.h>
#include <tf/tf.h>
#include <angles/angles.h>

namespace roadmap_global_planner {

void set_angle(geometry_msgs::PoseStamped* pose, double angle)
{
  
    pose->pose.orientation = tf::createQuaternionMsgFromYaw(angle); 
}

double getYaw(geometry_msgs::PoseStamped pose)
{
    return tf::getYaw(pose.pose.orientation);
}

void OrientationFiller::processPath(const geometry_msgs::PoseStamped& start, 
                                    std::vector<geometry_msgs::PoseStamped>& path)
{
   
    int n = path.size();
    switch(omode_) {
        case FORWARD:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
            
            }
            break;
        case BACKWARD:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angles::normalize_angle(getYaw(path[i]) + M_PI));
            }
            break;
        case LEFTWARD:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angles::normalize_angle(getYaw(path[i]) - M_PI_2));
            }
            break;
        case RIGHTWARD:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angles::normalize_angle(getYaw(path[i]) + M_PI_2));
            }
            break;
        case INTERPOLATE:
            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, 0, n-1);
            break;
        case FORWARDTHENINTERPOLATE:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
            }
            
            int i=n-3;
            double last = getYaw(path[i]);
            while( i>0 ){
                double new_angle = getYaw(path[i-1]);
                double diff = fabs(angles::shortest_angular_distance(new_angle, last));
                if( diff>0.35)
                    break;
                else
                    i--;
            }
            
            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, i, n-1);
            break;           
    }
}
    
void OrientationFiller::setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path, int index)
{
  int index0 = std::max(0, index - window_size_);
  int index1 = std::min((int)path.size() - 1, index + window_size_);

  double x0 = path[index0].pose.position.x,
         y0 = path[index0].pose.position.y,
         x1 = path[index1].pose.position.x,
         y1 = path[index1].pose.position.y;

  double angle = atan2(y1-y0,x1-x0);
  set_angle(&path[index], angle);
}

void OrientationFiller::interpolate(std::vector<geometry_msgs::PoseStamped>& path, 
                                    int start_index, int end_index)
{
    double start_yaw = getYaw(path[start_index]),
           end_yaw   = getYaw(path[end_index  ]);
    double diff = angles::shortest_angular_distance(start_yaw, end_yaw);
    double increment = diff/(end_index-start_index);
    for(int i=start_index; i<=end_index; i++){
        double angle = start_yaw + increment * i;
        set_angle(&path[i], angle);
    }
}
                                   

};
