#include "con_plan_exec_pkg/rviz.h"


void cRVIZ::init_marker(visualization_msgs::Marker& marker, int id, float x, float y, float z, float r, float g, float b)
{
    marker.header.frame_id = "rviz_frame";
    marker.header.stamp = ros::Time::now();

    marker.ns = "rviz_ns";
    marker.id = id;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x + X_OFFSET;
    marker.pose.position.y = y + Y_OFFSET;
    marker.pose.position.z = z + Z_OFFSET;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
}


void cRVIZ::init_marker3d(visualization_msgs::Marker& marker, int id, float x, float y, float z, float r, float g, float b, float a)
{
    marker.header.frame_id = "rviz_frame";
    marker.header.stamp = ros::Time::now();

    marker.ns = "rviz_ns";
    marker.id = id;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x + X_OFFSET;
    marker.pose.position.y = y + Y_OFFSET;
    marker.pose.position.z = z + Z_OFFSET;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;     // 0: Transparent, 1: Opaque

    marker.lifetime = ros::Duration();
}


void cRVIZ::init_rviz(int ws_size_x, int ws_size_y, float_mat ws, ros::NodeHandle *nh)
{
    // std::cout << "RViz init...\n";
    ws_x = ws_size_x;
    ws_y = ws_size_y;
    rviz_pub = nh->advertise<visualization_msgs::MarkerArray>("rviz_topic", 10);

    for(uint row_id = 0; row_id < ws_x; row_id++)
        for(uint col_id = 0; col_id < ws_y; col_id++)
        {
            visualization_msgs::Marker marker;
            uint marker_id = row_id * ws_y + col_id;

            if(ws[row_id][col_id] == -1.0)
                init_marker(marker, marker_id, row_id, col_id, 0, 0, 0, 0);
            else if(ws[row_id][col_id] == 0.0)
                init_marker(marker, marker_id, row_id, col_id, 0, 1, 0, 0);
            else if(ws[row_id][col_id] == 0.5)
                init_marker(marker, marker_id, row_id, col_id, 0, 1, 1, 0);
            else
                init_marker(marker, marker_id, row_id, col_id, 0, 0, 1, 0);

            rviz_marker_array.markers.push_back(marker);
        }

    rviz_pub.publish(rviz_marker_array);
}


void cRVIZ::init_rviz3d(uint ws_size_x, uint ws_size_y, uint ws_size_z, double_3d_vec ws, ros::NodeHandle *nh)
{
    // std::cout << "RViz init...\n";
    ws_x = ws_size_x;
    ws_y = ws_size_y;
    ws_z = ws_size_z;
    rviz_pub = nh->advertise<visualization_msgs::MarkerArray>("rviz_topic", 10);

    for(uint k = 0; k < ws_z; k++)
        for(uint i = 0; i < ws_x; i++)
            for(uint j = 0; j < ws_y; j++)
            {
                visualization_msgs::Marker marker;
                uint marker_id = k * ws_x * ws_y + i * ws_y + j;

                if(ws[k][i][j] == -1.0)
                    init_marker3d(marker, marker_id, i, j, k, 0, 0, 0, TRANSPARENCY);
                else if(ws[k][i][j] == 0.0)
                    init_marker3d(marker, marker_id, i, j, k, 1, 0, 0, 1);
                else if(ws[k][i][j] == 0.5)
                    init_marker3d(marker, marker_id, i, j, k, 1, 1, 0, TRANSPARENCY);
                else
                    init_marker3d(marker, marker_id, i, j, k, 0, 1, 0, TRANSPARENCY);

                rviz_marker_array.markers.push_back(marker);
            }

    rviz_pub.publish(rviz_marker_array);
}


void cRVIZ::update_rviz(float_mat ws)
{
    // std::cout << "RViz update...\n";

    for(uint row_id = 0; row_id < ws_x; row_id++)
        for(uint col_id = 0; col_id < ws_y; col_id++)
        {
            int marker_id = row_id * ws_y + col_id;
            std_msgs::ColorRGBA marker_color = rviz_marker_array.markers[marker_id].color;

            if((ws[row_id][col_id] == -1.0) && ((marker_color.r != 0) || (marker_color.b != 0) || (marker_color.b != 0)))
            {
                rviz_marker_array.markers[marker_id].color.r = 0;
                rviz_marker_array.markers[marker_id].color.g = 0;
                rviz_marker_array.markers[marker_id].color.b = 0;
            }
            else if((ws[row_id][col_id] == 0.0) && ((marker_color.r != 1) || (marker_color.b != 0) || (marker_color.b != 0)))
            {
                rviz_marker_array.markers[marker_id].color.r = 1;
                rviz_marker_array.markers[marker_id].color.g = 0;
                rviz_marker_array.markers[marker_id].color.b = 0;
            }
            else if((ws[row_id][col_id] == 0.5) && ((marker_color.r != 1) || (marker_color.b != 1) || (marker_color.b != 0)))
            {
                rviz_marker_array.markers[marker_id].color.r = 1;
                rviz_marker_array.markers[marker_id].color.g = 1;
                rviz_marker_array.markers[marker_id].color.b = 0;
            }
            else if((ws[row_id][col_id] == 1.0) && ((marker_color.r != 0) || (marker_color.b != 1) || (marker_color.b != 0)))
            {
                rviz_marker_array.markers[marker_id].color.r = 0;
                rviz_marker_array.markers[marker_id].color.g = 1;
                rviz_marker_array.markers[marker_id].color.b = 0;
            }
        }

    rviz_pub.publish(rviz_marker_array);
}


void cRVIZ::update_rviz3d(double_3d_vec ws)
{
    // std::cout << "RViz update...\n";

    for(uint k = 0; k < ws_z; k++)
        for(uint i = 0; i < ws_x; i++)
            for(uint j = 0; j < ws_y; j++)
            {
                int marker_id = k * ws_x * ws_y + i * ws_y + j;
                std_msgs::ColorRGBA marker_color = rviz_marker_array.markers[marker_id].color;

                if((ws[k][i][j] == -1.0) && ((marker_color.r != 0) || (marker_color.b != 0) || (marker_color.b != 0)))
                {
                    rviz_marker_array.markers[marker_id].color.r = 0;
                    rviz_marker_array.markers[marker_id].color.g = 0;
                    rviz_marker_array.markers[marker_id].color.b = 0;
                    rviz_marker_array.markers[marker_id].color.a = TRANSPARENCY;
                }
                else if((ws[k][i][j] == 0.0) && ((marker_color.r != 1) || (marker_color.b != 0) || (marker_color.b != 0)))
                {
                    rviz_marker_array.markers[marker_id].color.r = 1;
                    rviz_marker_array.markers[marker_id].color.g = 0;
                    rviz_marker_array.markers[marker_id].color.b = 0;
                    rviz_marker_array.markers[marker_id].color.a = 1;
                }
                else if((ws[k][i][j] == 0.5) && ((marker_color.r != 1) || (marker_color.b != 1) || (marker_color.b != 0)))
                {
                    rviz_marker_array.markers[marker_id].color.r = 1;
                    rviz_marker_array.markers[marker_id].color.g = 1;
                    rviz_marker_array.markers[marker_id].color.b = 0;
                    rviz_marker_array.markers[marker_id].color.a = TRANSPARENCY;
                }
                else if((ws[k][i][j] == 1.0) && ((marker_color.r != 0) || (marker_color.b != 1) || (marker_color.b != 0)))
                {
                    rviz_marker_array.markers[marker_id].color.r = 0;
                    rviz_marker_array.markers[marker_id].color.g = 1;
                    rviz_marker_array.markers[marker_id].color.b = 0;
                    rviz_marker_array.markers[marker_id].color.a = TRANSPARENCY;
                }
            }

    rviz_pub.publish(rviz_marker_array);
}