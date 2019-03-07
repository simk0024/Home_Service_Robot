#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class AddMarker{
  private: 
  	ros::NodeHandle n;
  	ros::Publisher marker_pub;
  	ros::Subscriber odom_sub;
  	visualization_msgs::Marker marker;
  	double pickupPosition[2] = {0.0, 8.0};
  	double dropoffPosition[2] = {-8.0, 0.0};
  	double tolerance = 0.25;
  	uint32_t shape = visualization_msgs::Marker::CUBE;
  
  public:
  	AddMarker(){
      	//Initialize marker publisher and odom subscriber
    	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
      	odom_sub = n.subscribe("/odom", 10, &AddMarker::odom_callback, this);
      	
      	//Initial marker
      	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
    	marker.header.frame_id = "/map";
    	marker.header.stamp = ros::Time::now();
      	// Set the namespace and id for this marker.  This serves to create a unique ID
    	// Any marker sent with the same namespace and id will overwrite the old one
    	marker.ns = "basic_shapes";
    	marker.id = 0;
      	//Set marker type
      	marker.type = shape;
      	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    	marker.action = visualization_msgs::Marker::ADD;
      	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    	setMarkerPosition(pickupPosition);
      	marker.pose.position.z = 0.0;
      	marker.pose.orientation.x = 0.0;
    	marker.pose.orientation.y = 0.0;
    	marker.pose.orientation.z = 0.0;
    	marker.pose.orientation.w = 1.0;
      	// Set the scale of the marker -- 1x1x1 here means 1m on a side
    	marker.scale.x = 0.2;
    	marker.scale.y = 0.2;
    	marker.scale.z = 0.2;
		// Set the color -- be sure to set alpha to something non-zero!
    	marker.color.r = 1.0f;
    	marker.color.g = 0.0f;
    	marker.color.b = 0.0f;
    	marker.color.a = 1.0;
    	marker.lifetime = ros::Duration();
      	publishMarker();
    }
  
  	void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
  		float pos_x = odom_msg->pose.pose.position.x;
  		float pos_y = odom_msg->pose.pose.position.y;
      
      	bool reachPickup = (getDistance(pos_x, pos_y, pickupPosition[0], pickupPosition[1]) <= tolerance);
        bool reachDropoff = (getDistance(pos_x, pos_y, dropoffPosition[0], dropoffPosition[1]) <= tolerance);
      
      	if (reachPickup){
          ROS_INFO("Robot reached Pick-up zone....");
          setMarkerPosition(dropoffPosition);
          ros::Duration(5).sleep();
          publishMarker();
        }
      	else if (reachDropoff){
          ROS_INFO("Robot reached Drop-off zone....");
          ros::Duration(5).sleep();
        }
      
	}
  
  	void publishMarker(){
    	while (marker_pub.getNumSubscribers() < 1)
    	{
      		if (!ros::ok())
      		{
	        	return;
	      	}
	      	ROS_WARN_ONCE("Please create a subscriber to the marker");
	      	sleep(1);
	    }
	    marker_pub.publish(marker);
    }
      
  	void setMarkerPosition(double position[])
    {
      	marker.pose.position.x = position[0];
     	marker.pose.position.y = position[1];
    }
  
  	double getDistance(double x1, double y1, double x2, double y2)
    {
      double dx = x1 - x2;
      double dy = y1 - y2;
      return sqrt(dx * dx + dy * dy);
    }
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_node");
  AddMarker AddMarker;
  ros::Rate r(1);
  ros::spin();
  return 0;
}