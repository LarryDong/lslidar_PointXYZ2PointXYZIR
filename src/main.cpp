
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lslidar_conversion.h"


ros::Publisher g_pub_pc;

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg_pc){
    ROS_INFO("Lidar callback. Start convertion");
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PointCloud<myPointXYZIR> pc_new;
    pcl::fromROSMsg(*msg_pc, pc);

    // convert to PointXYZIR.
    pc_new.points.reserve(pc.points.size());
    myPointXYZIR pt_new;
    for(const pcl::PointXYZ& p : pc.points){
        float angle = atan(p.z / sqrt(p.x * p.x + p.y * p.y)) * 180 / M_PI;
        int scanID = int(angle + 17);           // 1 degree decode mode.
        pt_new.x = p.x;
        pt_new.y = p.y;
        pt_new.z = p.z;
        pt_new.intensity = 0;      // Not used.
        pt_new.ring = scanID;
        pc_new.points.push_back(pt_new);
    }
    sensor_msgs::PointCloud2 msg_pc_new;
    pcl::toROSMsg(pc_new, msg_pc_new);
    msg_pc_new.header.frame_id = "laser_link";
    msg_pc_new.header.stamp = msg_pc->header.stamp;
    g_pub_pc.publish(msg_pc_new);
    ROS_INFO("Published new pointcloud.");
}

int main(int argc, char **argv){

    ros::init(argc, argv, "lslidar_conversion");
    ros::NodeHandle nh;
    ROS_WARN("--> lslidar data type conversion begin...");
    ros::Subscriber sub_pc = nh.subscribe<sensor_msgs::PointCloud2>("/pc_input", 100, lidarCallback);
    g_pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/pc_output", 100);
    
    
    ros::Rate rate(100);

    while (ros::ok()){
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}