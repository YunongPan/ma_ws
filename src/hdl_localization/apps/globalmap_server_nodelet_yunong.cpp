
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

namespace hdl_localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI; //////////////// modified

  int map_num = 1;

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
    globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(5.0), &GlobalmapServerNodelet::pub_once_cb, this, false, true);
  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");

    globalmap00.reset(new pcl::PointCloud<PointT>());    
    pcl::io::loadPCDFile("/home/yunong/ma_ws/src/hdl_localization/data/test_pcd_filtered_-10.pcd", *globalmap00);
    globalmap00->header.frame_id = "map";

    globalmap01.reset(new pcl::PointCloud<PointT>()); 
    pcl::io::loadPCDFile("/home/yunong/ma_ws/src/hdl_localization/data/test_pcd_filtered_-11.pcd", *globalmap01);
    globalmap01->header.frame_id = "map";

    globalmap11.reset(new pcl::PointCloud<PointT>()); 
    pcl::io::loadPCDFile("/home/yunong/ma_ws/src/hdl_localization/data/test_pcd_filtered_01.pcd", *globalmap11);
    globalmap11->header.frame_id = "map";

    globalmap10.reset(new pcl::PointCloud<PointT>()); 
    pcl::io::loadPCDFile("/home/yunong/ma_ws/src/hdl_localization/data/test_pcd_filtered_00.pcd", *globalmap10);
    globalmap10->header.frame_id = "map";

  }



//////////////////////////
//    std::ifstream utm_file(globalmap_pcd + ".utm");
//    if (utm_file.is_open() && private_nh.param<bool>("convert_utm_to_local", true)) {
//      double utm_easting;
//      double utm_northing;
//      double altitude;
//      utm_file >> utm_easting >> utm_northing >> altitude;
//      for(auto& pt : globalmap->points) {
//        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
//      }
//      ROS_INFO_STREAM("Global map offset by UTM reference coordinates (x = "
//                      << utm_easting << ", y = " << utm_northing << ") and altitude (z = " << altitude << ")");
//    }
///////////////////////////


//    // downsample globalmap
//    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
//    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
//    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
//    voxelgrid->setInputCloud(globalmap);

//    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
//    voxelgrid->filter(*filtered);
//    globalmap = filtered;



  void pub_once_cb(const ros::WallTimerEvent& event) {

    if (map_num == 1){
      globalmap_pub.publish(globalmap00);
    }
    else if(map_num == 2){
      globalmap_pub.publish(globalmap01);
    }
    else if(map_num == 3){
      globalmap_pub.publish(globalmap11);
    }
    else{
      globalmap_pub.publish(globalmap10);
    }
    map_num ++;
    if (map_num == 5){
      map_num = 1;
    }

  } 

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;

  ros::WallTimer globalmap_pub_timer;
  pcl::PointCloud<PointT>::Ptr globalmap00;
  pcl::PointCloud<PointT>::Ptr globalmap01;
  pcl::PointCloud<PointT>::Ptr globalmap11;
  pcl::PointCloud<PointT>::Ptr globalmap10;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)



