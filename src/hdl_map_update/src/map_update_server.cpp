#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "hdl_map_update/Chunk_index.h"

using PointT = pcl::PointXYZI;



//-----------------------------------------------------------------------------------
class ServeAndPublish
{

public:
  ServeAndPublish()
  {
    globalmap_pub = n.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);

    update_service = n.advertiseService("/update_map", &ServeAndPublish::updateCallback, this);    

  }


  bool updateCallback(hdl_map_update::Chunk_index::Request &req, hdl_map_update::Chunk_index::Response &res)
  {

    ROS_INFO("row:%d column:%d", req.chunk_index_1, req.chunk_index_2);
    //generate new map and publish once.
    res.result = "OK";


    int i = req.chunk_index_1;
    int j = req.chunk_index_2;

    std::string pcd_file_0_0 = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/map_1124_ascii_" + std::to_string(i) + "_" + std::to_string(j) + ".pcd";
    std::string pcd_file_1_0 = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/map_1124_ascii_" + std::to_string(i + 1) + "_" + std::to_string(j) + ".pcd";
    std::string pcd_file_n1_0 = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/map_1124_ascii_" + std::to_string(i - 1) + "_" + std::to_string(j) + ".pcd";
    std::string pcd_file_0_1 = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/map_1124_ascii_" + std::to_string(i) + "_" + std::to_string(j + 1) + ".pcd";
    std::string pcd_file_1_1 = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/map_1124_ascii_" + std::to_string(i + 1) + "_" + std::to_string(j + 1) + ".pcd";
    std::string pcd_file_n1_1 = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/map_1124_ascii_" + std::to_string(i - 1) + "_" + std::to_string(j + 1) + ".pcd";
    std::string pcd_file_0_n1 = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/map_1124_ascii_" + std::to_string(i) + "_" + std::to_string(j - 1) + ".pcd";
    std::string pcd_file_1_n1 = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/map_1124_ascii_" + std::to_string(i + 1) + "_" + std::to_string(j - 1) + ".pcd";
    std::string pcd_file_n1_n1 = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/map_1124_ascii_" + std::to_string(i - 1) + "_" + std::to_string(j - 1) + ".pcd";

    globalmap_0_0.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file_0_0, *globalmap_0_0);
    globalmap_0_0->header.frame_id = "map";

    globalmap_1_0.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file_1_0, *globalmap_1_0);
    globalmap_1_0->header.frame_id = "map";

    globalmap_n1_0.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file_n1_0, *globalmap_n1_0);
    globalmap_n1_0->header.frame_id = "map";

    globalmap_0_1.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file_0_1, *globalmap_0_1);
    globalmap_0_1->header.frame_id = "map";

    globalmap_1_1.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file_1_1, *globalmap_1_1);
    globalmap_1_1->header.frame_id = "map";

    globalmap_n1_1.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file_n1_1, *globalmap_n1_1);
    globalmap_n1_1->header.frame_id = "map";

    globalmap_0_n1.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file_0_n1, *globalmap_0_n1);
    globalmap_0_n1->header.frame_id = "map";

    globalmap_1_n1.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file_1_n1, *globalmap_1_n1);
    globalmap_1_n1->header.frame_id = "map";

    globalmap_n1_n1.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(pcd_file_n1_n1, *globalmap_n1_n1);
    globalmap_n1_n1->header.frame_id = "map";




    globalmap_output.reset(new pcl::PointCloud<PointT>());
    *globalmap_output += *globalmap_0_0;
    *globalmap_output += *globalmap_1_0;
    *globalmap_output += *globalmap_n1_0;
    *globalmap_output += *globalmap_0_1;
    *globalmap_output += *globalmap_1_1;
    *globalmap_output += *globalmap_n1_1;
    *globalmap_output += *globalmap_0_n1;
    *globalmap_output += *globalmap_1_n1;
    *globalmap_output += *globalmap_n1_n1;
    globalmap_output->header.frame_id = "map";


    globalmap_pub.publish(globalmap_output);

    return true;

  }

private:
  ros::NodeHandle n;
  ros::Publisher globalmap_pub;
  ros::ServiceServer update_service;

  pcl::PointCloud<PointT>::Ptr globalmap_output;
  pcl::PointCloud<PointT>::Ptr globalmap_0_0;
  pcl::PointCloud<PointT>::Ptr globalmap_1_0;
  pcl::PointCloud<PointT>::Ptr globalmap_n1_0;
  pcl::PointCloud<PointT>::Ptr globalmap_0_1;
  pcl::PointCloud<PointT>::Ptr globalmap_1_1;
  pcl::PointCloud<PointT>::Ptr globalmap_n1_1;
  pcl::PointCloud<PointT>::Ptr globalmap_0_n1;
  pcl::PointCloud<PointT>::Ptr globalmap_1_n1;
  pcl::PointCloud<PointT>::Ptr globalmap_n1_n1;

};

//-----------------------------------------------------------------------------------






int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_update_server");
//    ros::NodeHandle n;
//    ros::ServiceServer update_service = n.advertiseService("/update_map", updateCallback);

    ROS_INFO("Ready to update map.");

    ServeAndPublish SAPObject;
    ros::spin();

    return 0;

}
