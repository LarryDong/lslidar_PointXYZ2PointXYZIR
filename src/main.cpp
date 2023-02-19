
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lslidar_conversion.h"

#define RING_NUMBER 32
// #define PUB_EACH_RING               // publish each ring for debug.

const vector<float> g_ring_angle = {-18, -15, -12, -10, -8, -7, -6, -5,
                              -4, -3.33, -3, -2.66, -2.33, -2, -1.66, -1.33,
                              -1, -0.66, -0.33, 0, 0.33, 0.66, 1, 1.33, 
                              1.66, 2, 3, 4, 6, 8, 11, 14};             // ring angles defined by leishen
vector<float> g_angle_range;                                            // define a range between each ring angle.
ros::Publisher g_pub_pc;

#ifdef PUB_EACH_RING           
vector<ros::Publisher> gv_pub_rings;
vector<pcl::PointCloud<myPointXYZIR>> gv_pc(RING_NUMBER);
#endif


void initRingAngleRange(void){
    // calculate angle range
    assert(RING_NUMBER==g_ring_angle.size());
    g_angle_range.push_back(-100);            // assign a very large value
    for(int i=0; i<RING_NUMBER-1; ++i){
        float middle_value = (g_ring_angle[i] + g_ring_angle[i + 1]) / 2;   // calculate the average value between two ring.
        g_angle_range.push_back(middle_value);
    }
    g_angle_range.push_back(100);
}


void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg_pc){
    // ROS_INFO("Lidar callback. Start convertion");
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PointCloud<myPointXYZIR> pc_new;
    pcl::fromROSMsg(*msg_pc, pc);

    // convert to PointXYZIR.
    pc_new.points.reserve(pc.points.size());
    myPointXYZIR pt_new;

    for(const pcl::PointXYZ& p : pc.points){
        float angle = atan(p.z / sqrt(p.x * p.x + p.y * p.y)) * 180 / M_PI;
        if(std::isnan(angle))           // remove nan point.
            continue;
        // int scanID = int(angle + 17);           // 1 degree decode mode, just add 17 is okay.
        // for 0.33 degree mode.
        int scanID = -1;
        for(int i=0; i<RING_NUMBER; ++i){
            if(angle > g_angle_range[i] && angle <= g_angle_range[i+1]){
                scanID = i;
                break;
            }
        }
        pt_new.x = p.x;
        pt_new.y = p.y;
        pt_new.z = p.z;
        pt_new.intensity = 0;       // intensity is not used.
        pt_new.ring = scanID;
        pc_new.points.push_back(pt_new);
#ifdef PUB_EACH_RING
        gv_pc[scanID].push_back(pt_new);
#endif
    }
    sensor_msgs::PointCloud2 msg_pc_new;
    pcl::toROSMsg(pc_new, msg_pc_new);
    msg_pc_new.header.frame_id = "laser_link";
    msg_pc_new.header.stamp = msg_pc->header.stamp;
    g_pub_pc.publish(msg_pc_new);
    ROS_INFO("Published new pointcloud.");

#ifdef PUB_EACH_RING
    ROS_INFO("DEBUG: publish all rings for view.");
    for(int i=0; i<RING_NUMBER; ++i){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(gv_pc[i], msg);
        gv_pc[i].points.resize(0);
        msg.header.frame_id = "laser_link";
        msg.header.stamp = msg_pc->header.stamp;
        gv_pub_rings[i].publish(msg);
    }
#endif
}



int main(int argc, char **argv){
    ros::init(argc, argv, "lslidar_conversion");
    ros::NodeHandle nh;
    ROS_INFO("--> lslidar data type conversion begin...");
    initRingAngleRange();           // init g_angle.
    ros::Subscriber sub_pc = nh.subscribe<sensor_msgs::PointCloud2>("/pc_input", 100, lidarCallback);
    g_pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/pc_output", 100);
    
#ifdef PUB_EACH_RING                   // publish all rings.
    ROS_INFO("Debug: publish each rings.");
    ros::Publisher pub_ring;
    for(int i=0; i<RING_NUMBER; ++i){
        pub_ring = nh.advertise<sensor_msgs::PointCloud2>("/ring_" + to_string(i), 100);
        gv_pub_rings.push_back(pub_ring);
    }
#endif

    ros::Rate rate(100);
    while (ros::ok()){
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}