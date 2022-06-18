#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "dirent.h"
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <experimental/filesystem>

int chunk00_x_min_bound;
int chunk00_y_min_bound;
int chunk_index_1_previous = -1;
int chunk_index_2_previous = -1;
std::string current_map = "-1_-1";

std::string pcd_folder_path = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/gazebo_pcd_files/";
std::string pcd_same_part = "gazebo_map_all_220514_ascii_";
std::string first_pcd_file_name = "gazebo_map_all_220514_ascii_2_4.pcd";

int delay_counter = 0;

using PointT = pcl::PointXYZI;

//-----------------------------------------------------------------------------------
std::vector<std::string> Spliter(std::string satz, const char* delim) { // split the name of the the name of the random pcd file. Ready to get the label number.
    const char* c_satz = satz.c_str();
    std::vector<std::string> s;
    char* token = strtok((char*)c_satz, delim);
    while (token != NULL) {

        s.push_back(std::string(token));
        token = strtok(NULL, delim);
    }
    return s;

}

//-----------------------------------------------------------------------------------
bool IsFileExist(const std::string& file_path) {
    std::ifstream file(file_path.c_str());
    return file.good();
}

//-----------------------------------------------------------------------------------
std::vector<std::string> AddNewGrid(std::vector<std::string> n_area_old_deleted, std::vector<std::string> n_area_new) {

    std::vector<std::string> v3_new = n_area_old_deleted;

    for (int i = 0; i < n_area_new.size(); i++) {
        for (int j = 0; j < v3_new.size(); j++) {
            if (n_area_new[i] == v3_new[j]) {
                break;
            }

            if (j == v3_new.size() - 1) {
                for (int k = 0; k < v3_new.size(); k++) {
                    if (v3_new[k] == "0") {
                        v3_new[k] = n_area_new[i];
                        break;
                    }
                }
            }
        }
    }

    return v3_new;

}

//-----------------------------------------------------------------------------------
std::vector<int> FindNoSame(std::vector<std::string> n_area_old, std::vector<std::string> n_area_new) {
    std::vector<int> no_same_index;
    for (int i = 0; i < n_area_new.size(); i++) {
        for (int j = 0; j < n_area_old.size(); j++) {
            if (n_area_new[i] == n_area_old[j]) {
                n_area_old[j] = "0";
                break;
            }
        }
    }

    for(int k=0; k < n_area_old.size(); k++){
        if(n_area_old[k] != "0") {
            no_same_index.push_back(k);
        }
    }

    return no_same_index;
}

//-----------------------------------------------------------------------------------
std::string ReadTxt(std::string file) { // Read line 11 of the random pcd file, which means the first point.
    int i = 0;
    std::ifstream infile;
    infile.open(file.data());
    std::string line;
    if (infile.is_open()) {

        while (std::getline(infile, line) && i < 12) {
            i++;
        }
        infile.close();
    }
    return line;
}

//-------------------------------------------------------------------------------------
bool DelayChecker(std::string candidate_map) {

    bool need_map_update;

    if (current_map == candidate_map) {
        delay_counter = 0;

    }
    else {
        delay_counter++;
    }

    std::cout << delay_counter << std::endl;

    if (delay_counter == 3000) {       // setable time for delay.
        need_map_update = 1;
        delay_counter = 0;
    }
    else {
        need_map_update = 0;
    }

    return need_map_update;

}

//-----------------------------------------------------------------------------------
class ListenAndPublish {

    public:
    ListenAndPublish() {

        std::vector<std::string> v3(9);
        std::string v3_init_value = "0";
        fill(v3.begin(), v3.end(), v3_init_value);

        std::vector<pcl::PointCloud<PointT>> v4(9);


        globalmap_pub = n.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
        ///    location_sub = n.subscribe("/base/odom", 10, &SubscribeAndPublish::locationCallback, this);


        tf::TransformListener listener;

        ros::Rate rate(10.0);
        while (n.ok()) {
            tf::StampedTransform transform;
            try {
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

            std::string candidate_map = std::to_string(chunk_index_1) + "_" + std::to_string(chunk_index_2);


            bool need_map_update = DelayChecker(candidate_map);


            if (need_map_update == 1) {

//                // check which map need to be load according to candidate_map / chunk_index_1 & chunk_index_2,
//                // check if there have same map.
                int i = chunk_index_1;
                int j = chunk_index_2;

                std::cout << i << std::endl;
                std::cout << j << std::endl;

                std::string checked_pcd_file_r;
                std::string checked_pcd_file_n;
                std::vector<std::string> non_road_list_new;
//
                for(i = chunk_index_1 - 1; i < chunk_index_1 + 2; i++) {
                    for(j = chunk_index_2 - 1; j < chunk_index_2 + 2; j++) {

                        checked_pcd_file_n = pcd_folder_path + pcd_same_part + std::to_string(i) + "_" + std::to_string(j) + ".pcd"; ///////////////////////+++++///////////////////////////////////

                        non_road_list_new.push_back(checked_pcd_file_n);


                    }
                }


                std::vector<int> to_delete_index = FindNoSame(v3, non_road_list_new);


                for (int i = 0; i < to_delete_index.size(); i++) { // Set no more needed grid to 0 in v3, delete its pointcloud in v4.
                    v3[to_delete_index[i]] = "0";
                    temp_ptr.reset(new pcl::PointCloud<PointT>());
                    v4[to_delete_index[i]] = *temp_ptr;
                }


                std::vector<std::string> v3_new = AddNewGrid(v3, non_road_list_new); // Add new grid into v3, load its .pcd to v4


                v3 = v3_new;
                for (int i = 0; i < v3.size(); i++) {
                    if (v3[i] != "0") {

                        if (v4[i].width == 0) {

                            std::string pcd_file = v3[i];
                            temp_ptr.reset(new pcl::PointCloud<PointT>());
                            pcl::io::loadPCDFile(pcd_file, *temp_ptr);

                            v4[i] = *temp_ptr;
                            v4[i].header.frame_id = "map";

                            std::cout << v4[i].width << std::endl;  // print out how many points in loaded non road grids
                        }
                    }
                }


                globalmap_output.reset(new pcl::PointCloud<PointT>());
                for (int i = 0; i < v4.size(); i++) {
                    *globalmap_output += v4[i];
                }
                globalmap_output->header.frame_id = "map";


                chunk_index_1_previous = chunk_index_1;
                chunk_index_2_previous = chunk_index_2;


                globalmap_pub.publish(globalmap_output);
                rate.sleep();


                current_map = candidate_map;
            }

            usleep(1000);   // tf listener time interval: 1 ms.
        } //while end

    } // ListenAndPublish() end


    private:
    ros::NodeHandle n;

    ros::Publisher globalmap_pub;
    pcl::PointCloud<PointT>::Ptr globalmap_output;

    pcl::PointCloud<PointT>::Ptr temp_ptr;

}; //Class end

//-----------------------------------------------------------------------------------

int main(int argc, char** argv) {

//    std::string first_pcd_file_name = GetFileNames(pcd_folder_path);
    std::cout << first_pcd_file_name << std::endl;

    first_pcd_file_name = first_pcd_file_name.substr(0, first_pcd_file_name.length() - 4);

    std::string first_pcd_file_path_pcd = pcd_folder_path + first_pcd_file_name + ".pcd";

    std::string first_pcd_file_path_txt = pcd_folder_path + first_pcd_file_name + ".txt";
    rename(first_pcd_file_path_pcd.c_str(), first_pcd_file_path_txt.c_str());


    std::string ref_point = ReadTxt(first_pcd_file_path_txt); // Got the first point in random pcd file. Just find a random pcd and read the first point of the pcd.


    rename(first_pcd_file_path_txt.c_str(), first_pcd_file_path_pcd.c_str());


    std::string s = ref_point;
    std::stringstream ss(s);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> vstrings(begin, end);

    float ref_point_x = std::stof(vstrings[0]);
    float ref_point_y = std::stof(vstrings[1]);  // Get the referenz point position x and y.

    std::vector<std::string> pcd_name_spl = Spliter(first_pcd_file_name, "_");
    int ref_chunk_index_1 = std::stoi(pcd_name_spl[pcd_name_spl.size() - 2]);
    int ref_chunk_index_2 = std::stoi(pcd_name_spl[pcd_name_spl.size() - 1]);
    // Just find a random pcd and read the first point of the pcd. Read the label of the filename. Count out the leftmost and lower bounds of the map.

    chunk00_x_min_bound = floor((ref_point_x - 15)/30) * 30 + 15 - 30 * ref_chunk_index_2;
    chunk00_y_min_bound = floor((ref_point_y - 15)/30) * 30 + 15 + 30 * ref_chunk_index_1;




    ros::init(argc, argv, "map_update_caller");

    ListenAndPublish LAPObject; // Listener And publisher


    return 0;

};
