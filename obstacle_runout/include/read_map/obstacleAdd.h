#ifndef _OBSTACLEADD_H_
#define _OBSTACLEADD_H_

#include "read_map/createGrid.h"
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pthread.h>
using namespace std;

typedef pcl::PointXYZI VPoint;

extern Point real_time_location;
extern int replan_received;

class obstacleAdd
{
	public:
		ros::Subscriber subfinal;
		ros::Subscriber obstacle_point_sub;
		ros::Subscriber dstar_replan_sub;
		ros::Subscriber current_heading_sub;
		ros::Subscriber map_id_sub;
		ros::Subscriber map_change_sub;
		ros::Publisher dynamic_map;
		ros::Publisher obstacle_point_pub;

		void topic_exchange(ros::NodeHandle n);
		void GetObstacles(const std_msgs::String::ConstPtr & msg);
		void FinalDataHandler(const std_msgs::String::ConstPtr &msg);
		void HeadingHandler(const std_msgs::String::ConstPtr &msg);
		void ReplanHandler(const std_msgs::Int8::ConstPtr &msg);
		void MapIdHandler(const std_msgs::Int8::ConstPtr &msg);
		void MapChangeHandler(const std_msgs::Int8::ConstPtr &msg);

		vector<Point> LocalToGlobalObsPoint(vector<Point> obstacle_point,int pointCount);
		int PointInGrid(double x,double y);
		void AddUnreachableGridAfterRobot();
		void dynamicObstacleAdd(vector<Point> global_obspoint,int cloudSize);
};

#endif




