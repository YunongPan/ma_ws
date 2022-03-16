#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include "dirent.h"

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

int chunk00_x_min_bound;
int chunk00_y_min_bound;
int chunk_index_1_previous = -2;
int chunk_index_2_previous = -2;

std::string pcd_folder_path = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/";
std::string pcd_same_part = "map_1124_ascii_";

using PointT = pcl::PointXYZI;

//-----------------------------------------------------------------------------------
std::vector<std::string> spliter(std::string satz, const char* delim){ // split the name of the the name of the random pcd file. Ready to get the label number.
  const char* c_satz = satz.c_str();
  std::vector<std::string> s;
  char* token = strtok((char*)c_satz, delim);
  while (token != NULL)
  {
    s.push_back(std::string(token));
    token = strtok(NULL, delim);
  }
  return s;

}



//-----------------------------------------------------------------------------------
std::string GetFileNames(std::string path) { // Read the name of the random pcd file, without the ".pcd".
  DIR *dpdf;
  struct dirent *epdf;

  dpdf = opendir(path.c_str());
  if (dpdf != NULL){
    int i = 0;
    while (epdf = readdir(dpdf)){
      // printf("Filename: %s",epdf->d_name);
      std::cout << epdf->d_name << std::endl;
      i += 1;
      if (i>=1){
        break;
      }
    }
  }
  closedir(dpdf);
  return epdf->d_name;
}



//-----------------------------------------------------------------------------------
std::string readTxt(std::string file) // Read line 11 of the random pcd file, which means the first point.
{
  int i = 0;
  std::ifstream infile;
  infile.open(file.data());
  std::string line;
  if (infile.is_open()) {

    while (std::getline(infile, line) && i<12) {

      i++;
    }
  infile.close();
  }
  return line;
}



//-----------------------------------------------------------------------------------
class ListenAndPublish
{

public:
  ListenAndPublish()
  {
//    ros::service::waitForService("/update_map"); ////////////////////////
//    update_client = n.serviceClient<hdl_map_update::Chunk_index>("/update_map"); ////////////////////////



    globalmap_pub = n.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
///    location_sub = n.subscribe("/base/odom", 10, &SubscribeAndPublish::locationCallback, this);    
    
  
    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (n.ok()){ 
      tf::StampedTransform transform;
      try{
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      float robot_location_x = transform.getOrigin().x(); 
      float robot_location_y = transform.getOrigin().y(); 


      int chunk_index_2 = (robot_location_x - chunk00_x_min_bound)/30;
      int chunk_index_1 = (chunk00_y_min_bound - robot_location_y)/30 + 1; // Calculate the Label of the submap where the robot is currently located.

      if (chunk_index_1 != chunk_index_1_previous || chunk_index_2 != chunk_index_2_previous){
        chunk_index_1_previous = chunk_index_1;
        chunk_index_2_previous = chunk_index_2;


//-----------------------------------------------------------------------------------
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


        globalmap_pub.publish(globalmap_output);

        rate.sleep();

//-----------------------------------------------------------------------------------





//      update_client.call(srv);                                           // Once across the bound of the submap, send out the srv.
        ROS_INFO("Global map has been updated.");
      } // if end





//    hdl_map_update::Chunk_index srv; // build a srv (srv Name: "srv", Type: Chunk_index)
//    srv.request.chunk_index_1 = chunk_index_1; //area index
//    srv.request.chunk_index_2 = chunk_index_2; //area index


      std::cout << robot_location_x << std::endl;
      std::cout << robot_location_y << std::endl;
      std::cout << chunk_index_1 << std::endl;
      std::cout << chunk_index_2 << std::endl;




    } //while end


  } // ListenAndPublish() end




private:
  ros::NodeHandle n;
//  ros::Subscriber location_sub;
//  ros::ServiceClient update_client; //////////////////////////////////////////////7

  ros::Publisher globalmap_pub;
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


}; //Class end

//-----------------------------------------------------------------------------------

int main(int argc, char** argv)
{

  std::string first_pcd_file_name = GetFileNames(pcd_folder_path);
  std::cout << first_pcd_file_name << std::endl;

  first_pcd_file_name = first_pcd_file_name.substr(0, first_pcd_file_name.length() - 4);
  
  std::string first_pcd_file_path_pcd = pcd_folder_path + first_pcd_file_name + ".pcd";

  std::string first_pcd_file_path_txt = pcd_folder_path + first_pcd_file_name + ".txt";
  rename(first_pcd_file_path_pcd.c_str(), first_pcd_file_path_txt.c_str());


  std::string ref_point = readTxt(first_pcd_file_path_txt); // Got the first point in random pcd file. Just find a random pcd and read the first point of the pcd.
  std::cout << ref_point << std::endl;

  rename(first_pcd_file_path_txt.c_str(), first_pcd_file_path_pcd.c_str());


//--------------------------------------------------------------------------------------------  
  std::string s = ref_point;
  std::stringstream ss(s);
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string> vstrings(begin, end);

  float ref_point_x = std::stof(vstrings[0]);
  float ref_point_y = std::stof(vstrings[1]);
  std::cout << ref_point_x << " " << ref_point_y << std::endl; // Get the referenz point position x and y.

//--------------------------------------------------------------------------------------------  
  std::vector<std::string> pcd_name_spl = spliter(first_pcd_file_name, "_");
  int ref_chunk_index_1 = std::stoi(pcd_name_spl[pcd_name_spl.size() - 2]);
  int ref_chunk_index_2 = std::stoi(pcd_name_spl[pcd_name_spl.size() - 1]);


  std::cout << ref_chunk_index_1 << std::endl;
  std::cout << ref_chunk_index_2 << std::endl; // Just find a random pcd and read the first point of the pcd. Read the label of the filename. Count out the leftmost and lower bounds of the map. 


  chunk00_x_min_bound = floor((ref_point_x - 15)/30) * 30 + 15 - 30 * ref_chunk_index_2;
  chunk00_y_min_bound = floor((ref_point_y - 15)/30) * 30 + 15 + 30 * ref_chunk_index_1;

  std::cout << chunk00_x_min_bound << std::endl;
  std::cout << chunk00_y_min_bound << std::endl;










  ros::init(argc, argv, "map_update_caller");

  ListenAndPublish LAPObject; // Listener And publisher


  //std::string path = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files";
  //for (const auto & entry : std::filesystem::directory_iterator(path))
  //  std::cout << entry.path() << std::endl;
//}
    //open hdl_map_update/pcd_files/splitted_pcd_files


//    ros::spin();

    return 0;

};
