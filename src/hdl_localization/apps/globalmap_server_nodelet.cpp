
#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>



int chunk_index_1 = 4;
int chunk_index_2 = 2; // When I turn on the robot, which chunk am I now?

std::string pcd_folder_path = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/";
std::string pcd_same_part = "map_1124_ascii_";



namespace hdl_localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet() {
  }
  virtual ~GlobalmapServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    // publish globalmap with "latched" publisher
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(1.0), &GlobalmapServerNodelet::pub_once_cb, this, true, true);
  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");  // Set param in launch file and read param. (no need here)

    globalmap_output.reset(new pcl::PointCloud<PointT>());
 
    // pcl::io::loadPCDFile(globalmap_pcd, *globalmap_output); // 



    int i = chunk_index_1;
    int j = chunk_index_2;

    std::string pcd_file_0_0 = pcd_folder_path + pcd_same_part + std::to_string(i) + "_" + std::to_string(j) + ".pcd";
    std::string pcd_file_1_0 = pcd_folder_path + pcd_same_part + std::to_string(i + 1) + "_" + std::to_string(j) + ".pcd";
    std::string pcd_file_n1_0 = pcd_folder_path + pcd_same_part + std::to_string(i - 1) + "_" + std::to_string(j) + ".pcd";
    std::string pcd_file_0_1 = pcd_folder_path + pcd_same_part + std::to_string(i) + "_" + std::to_string(j + 1) + ".pcd";
    std::string pcd_file_1_1 = pcd_folder_path + pcd_same_part + std::to_string(i + 1) + "_" + std::to_string(j + 1) + ".pcd";
    std::string pcd_file_n1_1 = pcd_folder_path + pcd_same_part + std::to_string(i - 1) + "_" + std::to_string(j + 1) + ".pcd";
    std::string pcd_file_0_n1 = pcd_folder_path + pcd_same_part + std::to_string(i) + "_" + std::to_string(j - 1) + ".pcd";
    std::string pcd_file_1_n1 = pcd_folder_path + pcd_same_part + std::to_string(i + 1) + "_" + std::to_string(j - 1) + ".pcd";
    std::string pcd_file_n1_n1 = pcd_folder_path + pcd_same_part + std::to_string(i - 1) + "_" + std::to_string(j - 1) + ".pcd";

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
  }



  void pub_once_cb(const ros::WallTimerEvent& event) {
    globalmap_pub.publish(globalmap_output);
  }




private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;

  ros::WallTimer globalmap_pub_timer;

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

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)
