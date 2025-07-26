/*
Purpose: RVIZ
Last updated: 
Last updated on: 
Author: Ratijit Mitra
*/


#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define X_OFFSET 0.5
#define Y_OFFSET X_OFFSET
#define Z_OFFSET X_OFFSET
#define TRANSPARENCY 0.1

using namespace std;

typedef vector<vector<float> > float_mat;
typedef vector<float_mat> double_3d_vec;

class cRVIZ
{
	public:
		uint ws_x, ws_y, ws_z;
		ros::Publisher rviz_pub;
		visualization_msgs::MarkerArray rviz_marker_array;

		void init_marker(visualization_msgs::Marker& marker, int id, float x, float y, float z, float r, float g, float b);
		void init_marker3d(visualization_msgs::Marker& marker, int id, float x, float y, float z, float r, float g, float b, float a);
		void init_rviz(int ws_size_x, int ws_size_y, float_mat ws, ros::NodeHandle *nh);
		void init_rviz3d(uint ws_size_x, uint ws_size_y, uint ws_size_z, double_3d_vec ws, ros::NodeHandle *nh);
		void update_rviz(float_mat ws);
		void update_rviz3d(double_3d_vec ws);
};