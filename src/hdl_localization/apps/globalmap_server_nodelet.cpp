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

// In order to start hdl_localization, a global map must be loaded.
// It doesn't matter what that global map looks like, since the global map will be updated when the map_update node is turned on.

//-----------------------------------------------------------------------------------
// The parameters can be set here.
// Choose a random file from folder /ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files and change this parameter to the full path to this file.
std::string random_file = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/gazebo_map_all_220514_ascii_31.38_0_7.pcd";
//-----------------------------------------------------------------------------------

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
            // read point cloud map from the random pcd file and set it as the initial globalmap.
            globalmap_output.reset(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(random_file, *globalmap_output);
            globalmap_output->header.frame_id = "map";
        }

        void pub_once_cb(const ros::WallTimerEvent& event) {
            globalmap_pub.publish(globalmap_output);
        }

        private:
        ros::NodeHandle nh;
        ros::NodeHandle mt_nh;
        ros::NodeHandle private_nh;
        ros::Publisher globalmap_pub;
        ros::WallTimer globalmap_pub_timer;
        pcl::PointCloud<PointT>::Ptr globalmap_output;
    };
}


PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)
