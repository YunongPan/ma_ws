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

//-----------------------------------------------------------------------------------
// The parameters can be set here.
// Choose a random file from folder /ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files and change this parameter to the full path to this file.
std::string random_file = "/home/yunong/ma_ws/src/hdl_map_update/pcd_files/splitted_pcd_files/gazebo_map_all_220514_ascii_31.38_0_7.pcd";
// Hysteresis time.
float hysteresis_time = 0; // [s]
//-----------------------------------------------------------------------------------

float chunk00_x_min_bound;
float chunk00_y_min_bound;
int chunk_index_1_previous = -1;
int chunk_index_2_previous = -1;
std::string current_map = "-1_-1";
int delay_counter = -1;
using PointT = pcl::PointXYZI;

//-----------------------------------------------------------------------------------
// split the name of the random pcd file to get path, chunk size and chunk index.
std::vector<std::string> Spliter(std::string satz, const char* delim) {
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
// Reorganize split file paths
std::string CombinString(std::string start, int end, std::string fill, std::vector<std::string> s) {
    std::string combined_string = start;
    for (int i = 0; i < s.size() - end; i++) {
        combined_string = combined_string + s[i] + fill;
    }
    return combined_string;
}

//-----------------------------------------------------------------------------------
// split the name of the random pcd file to get path, chunk size and chunk index
std::vector<std::string> random_file_spl = Spliter(random_file, "/");
std::string random_pcd_file_name = random_file_spl[random_file_spl.size() - 1];

std::string pcd_folder_path = CombinString("/", 1, "/", random_file_spl);

std::vector<std::string> pcd_name_spl = Spliter(random_pcd_file_name, "_");
std::string pcd_same_part = CombinString("", 2, "_", pcd_name_spl);

float chunk_size = std::stoi(pcd_name_spl[pcd_name_spl.size() - 3]);
float half_chunk_size = chunk_size/2.0;

//-----------------------------------------------------------------------------------
bool IsFileExist(const std::string& file_path) {
    std::ifstream file(file_path.c_str());
    return file.good();
}

//-----------------------------------------------------------------------------------
// Add new map chunks after deleting the old ones that are not needed
std::vector<std::string> AddNewGrid(std::vector<std::string> n_area_old_deleted, std::vector<std::string> n_area_new) {
    std::vector<std::string> v1_new = n_area_old_deleted;
    for (int i = 0; i < n_area_new.size(); i++) {
        for (int j = 0; j < v1_new.size(); j++) {
            if (n_area_new[i] == v1_new[j]) {
                break;
            }

            if (j == v1_new.size() - 1) {
                for (int k = 0; k < v1_new.size(); k++) {
                    if (v1_new[k] == "0") {
                        v1_new[k] = n_area_new[i];
                        break;
                    }
                }
            }
        }
    }
    return v1_new;
}

//-----------------------------------------------------------------------------------
// Find out which old map chunks need to be deleted when the map is updating.
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
// Read line 11 of the random pcd file, which means the first point.
std::string ReadTxt(std::string file) {
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
// Hysteresis Method. Counter increases by 1 per second until it reaches the hysteresis time. Then the map update is triggered.
bool DelayChecker(std::string candidate_map) {
    bool need_map_update;
    int count_down;

    if (current_map == candidate_map) {
        delay_counter = -1;
    }
    else {
        delay_counter++;
    }

    int hy_time = static_cast<int>(hysteresis_time);
    if (delay_counter == -1) {
        std::cout << "No map update required......" << std::endl;
    }
    else {
        count_down = hy_time - delay_counter;
        std::cout << "Map update will be triggered after " + std::to_string(count_down) + " seconds." << std::endl;
    }

    if (delay_counter == hy_time) {
        need_map_update = 1;
        std::cout << "Updating map." << std::endl;
        delay_counter = -1;
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
        // Create a vector to record the paths of all map chunks.
        std::vector<std::string> v1(9);
        std::string v1_init_value = "0";
        fill(v1.begin(), v1.end(), v1_init_value);

        // Create a vector to store the point cloud of all map chunks. Corresponding to V1.
        std::vector<pcl::PointCloud<PointT>> v2(9);

        globalmap_pub = n.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);

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

            // Calculate the Label of the chunk where the robot is currently located.
            int chunk_index_2 = (robot_location_x - chunk00_x_min_bound)/chunk_size;
            int chunk_index_1 = (chunk00_y_min_bound - robot_location_y)/chunk_size + 1;

            // The Label of the chunk where the robot is currently located.
            std::string candidate_map = std::to_string(chunk_index_1) + "_" + std::to_string(chunk_index_2);

            // Check if the map update is triggered. Considering the hysteresis.
            bool need_map_update = DelayChecker(candidate_map);

            // Start map update
            if (need_map_update == 1) {
                // check which map need to be load according to candidate_map (chunk_index_1 & chunk_index_2).
                int i = chunk_index_1;
                int j = chunk_index_2;

                // The names of the nine map chunks around the current location.
                std::string single_chunk_file;
                std::vector<std::string> chunk_list_new;

                for(i = chunk_index_1 - 1; i < chunk_index_1 + 2; i++) {
                    for(j = chunk_index_2 - 1; j < chunk_index_2 + 2; j++) {
                        single_chunk_file = pcd_folder_path + pcd_same_part + std::to_string(i) + "_" + std::to_string(j) + ".pcd";
                        chunk_list_new.push_back(single_chunk_file);
                    }
                }

                // Find out which old map chunks are no longer needed. They need to be deleted.
                std::vector<int> to_delete_index = FindNoSame(v1, chunk_list_new);

                // Set no more needed grid to "0" in v1, delete its pointcloud in v2.
                for (int i = 0; i < to_delete_index.size(); i++) {
                    v1[to_delete_index[i]] = "0";
                    temp_ptr.reset(new pcl::PointCloud<PointT>());
                    v2[to_delete_index[i]] = *temp_ptr;
                }

                // Add new chunks into v1, load corresponding new .pcd to v2.
                std::vector<std::string> v1_new = AddNewGrid(v1, chunk_list_new);
                v1 = v1_new;
                for (int i = 0; i < v1.size(); i++) {
                    if (v1[i] != "0") {
                        if (v2[i].width == 0) {
                            std::string pcd_file = v1[i];
                            temp_ptr.reset(new pcl::PointCloud<PointT>());
                            pcl::io::loadPCDFile(pcd_file, *temp_ptr);
                            v2[i] = *temp_ptr;
                            v2[i].header.frame_id = "map";
                            std::cout << "New chunk with " + std::to_string(v2[i].width) + " points." << std::endl;// print out how many points in loaded non road grids
                        }
                    }
                }

                // Stitch all chunks in v2 and publish
                globalmap_output.reset(new pcl::PointCloud<PointT>());
                for (int i = 0; i < v2.size(); i++) {
                    *globalmap_output += v2[i];
                }
                globalmap_output->header.frame_id = "map";

                chunk_index_1_previous = chunk_index_1;
                chunk_index_2_previous = chunk_index_2;

                globalmap_pub.publish(globalmap_output);
                rate.sleep();

                current_map = candidate_map;
            }

            // tf listener time interval: 1 s.
            usleep(1000000);
        } //while end
    } // ListenAndPublish() end

    private:
    ros::NodeHandle n;

    ros::Publisher globalmap_pub;
    pcl::PointCloud<PointT>::Ptr globalmap_output;

    pcl::PointCloud<PointT>::Ptr temp_ptr;

}; //Class end


int main(int argc, char** argv) {
    // Open the random pcd and read the first point of it as reference point.
    // Then read the labels from the filename. Based on the reference point and labels calculate out the leftmost and lower bounds of the map.

    // Get the first point in the random pcd file.
    random_pcd_file_name = random_pcd_file_name.substr(0, random_pcd_file_name.length() - 4);
    std::string random_pcd_file_path_pcd = pcd_folder_path + random_pcd_file_name + ".pcd";
    std::string random_pcd_file_path_txt = pcd_folder_path + random_pcd_file_name + ".txt";
    rename(random_pcd_file_path_pcd.c_str(), random_pcd_file_path_txt.c_str());
    std::string ref_point = ReadTxt(random_pcd_file_path_txt);
    rename(random_pcd_file_path_txt.c_str(), random_pcd_file_path_pcd.c_str());

    // Get the referenz point position x and y.
    std::string s = ref_point;
    std::stringstream ss(s);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> vstrings(begin, end);
    float ref_point_x = std::stof(vstrings[0]);
    float ref_point_y = std::stof(vstrings[1]);

    // Read the labels from the filename.
    int ref_chunk_index_1 = std::stoi(pcd_name_spl[pcd_name_spl.size() - 2]);
    int ref_chunk_index_2 = std::stoi(pcd_name_spl[pcd_name_spl.size() - 1]);

    // Based on the reference point and labels calculate out the leftmost and lower bounds of the map.
    // They will be used in class ListenAndPublish.
    chunk00_x_min_bound = floor((ref_point_x - half_chunk_size)/chunk_size) * chunk_size + half_chunk_size - chunk_size * ref_chunk_index_2;
    chunk00_y_min_bound = floor((ref_point_y - half_chunk_size)/chunk_size) * chunk_size + half_chunk_size + chunk_size * ref_chunk_index_1;

    ros::init(argc, argv, "map_update_caller");

    ListenAndPublish LAPObject; // Listener And publisher

    return 0;

};
