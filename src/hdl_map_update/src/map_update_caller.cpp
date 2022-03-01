#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include "hdl_map_update/Chunk_index.h"
#include <nav_msgs/Odometry.h>

#include "dirent.h"

int chunk00_x_min_bound;
int chunk00_y_min_bound;
int chunk_index_1_previous = -1;
int chunk_index_2_previous = -1;

//-----------------------------------------------------------------------------------
std::vector<std::string> spliter(std::string satz, const char* delim){
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
std::string GetFileNames(std::string path) {
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
std::string readTxt(std::string file)
{
  int i = 0;
  std::ifstream infile;
  infile.open(file.data());
  std::string line;
  if (infile.is_open()) {

    while (std::getline(infile, line) && i<12) {
      // using printf() in all tests for consistency
      // std::cout<<line<<std::endl;
      i++;
    }
  infile.close();
  }
  return line;
}



//-----------------------------------------------------------------------------------
class SubscribeAndCall
{

public:
  SubscribeAndCall()
  {
    ros::service::waitForService("/update_map");
    update_client = n.serviceClient<hdl_map_update::Chunk_index>("/update_map");

    location_sub = n.subscribe("/base/odom", 10, &SubscribeAndCall::locationCallback, this);    

  }


  void locationCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {

    float robot_location_x = msg->pose.pose.position.x - 29; ////zan shi///////////////
    float robot_location_y = msg->pose.pose.position.y + 45; ////zan shi///////////////



    int chunk_index_2 = (robot_location_x - chunk00_x_min_bound)/30;
    int chunk_index_1 = (chunk00_y_min_bound - robot_location_y)/30 + 1;

    
    hdl_map_update::Chunk_index srv;
    srv.request.chunk_index_1 = chunk_index_1; //area index
    srv.request.chunk_index_2 = chunk_index_2; //area index

    std::cout << robot_location_x << std::endl;
    std::cout << robot_location_y << std::endl;
    std::cout << chunk_index_1 << std::endl;
    std::cout << chunk_index_2 << std::endl;

    if (chunk_index_1 != chunk_index_1_previous || chunk_index_2 != chunk_index_2_previous){
      chunk_index_1_previous = chunk_index_1;
      chunk_index_2_previous = chunk_index_2;
      update_client.call(srv);
      ROS_INFO("Global map has been updated: %s", srv.response.result.c_str());
    }

  }

private:
  ros::NodeHandle n;
  ros::Subscriber location_sub;
  ros::ServiceClient update_client;


};


int main(int argc, char** argv)
{

  std::string first_pcd_file_name = GetFileNames("/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files");
  std::cout << first_pcd_file_name << std::endl;

  first_pcd_file_name = first_pcd_file_name.substr(0, first_pcd_file_name.length() - 4);
  
  std::string first_pcd_file_path_pcd = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/" + first_pcd_file_name + ".pcd";

  std::string first_pcd_file_path_txt = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/" + first_pcd_file_name + ".txt";
  rename(first_pcd_file_path_pcd.c_str(), first_pcd_file_path_txt.c_str());


  std::string ref_point = readTxt(first_pcd_file_path_txt); //Got the first point in random pcd file.
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
std::cout << ref_point_x << " " << ref_point_y << std::endl;

//--------------------------------------------------------------------------------------------  
std::vector<std::string> pcd_name_spl = spliter(first_pcd_file_name, "_");
int ref_chunk_index_1 = std::stoi(pcd_name_spl[pcd_name_spl.size() - 2]);
int ref_chunk_index_2 = std::stoi(pcd_name_spl[pcd_name_spl.size() - 1]);
//pcd_name_spl[first_pcd_spl.size() - 2];

std::cout << ref_chunk_index_1 << std::endl;
std::cout << ref_chunk_index_2 << std::endl;


chunk00_x_min_bound = floor((ref_point_x - 15)/30) * 30 + 15 - 30 * ref_chunk_index_2;
chunk00_y_min_bound = floor((ref_point_y - 15)/30) * 30 + 15 + 30 * ref_chunk_index_1;

std::cout << chunk00_x_min_bound << std::endl;
std::cout << chunk00_y_min_bound << std::endl;










  ros::init(argc, argv, "map_update_caller");

  SubscribeAndCall SACObject;

    //read the name of first file in hdl_map_update/pcd_files/splitted_pcd_files




  //std::string path = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files";
  //for (const auto & entry : std::filesystem::directory_iterator(path))
  //  std::cout << entry.path() << std::endl;
//}
    //open hdl_map_update/pcd_files/splitted_pcd_files




    ros::spin();

    return 0;

};
