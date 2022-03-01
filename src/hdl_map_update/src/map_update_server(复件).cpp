#include <ros/ros.h>
#include "hdl_map_update/Chunk_index.h"


bool updateCallback(hdl_map_update::Chunk_index::Request &req, hdl_map_update::Chunk_index::Response &res)
{
    ROS_INFO("row:%d column:%d", req.chunk_index_1, req.chunk_index_2);

    //generate new map and publish once.

    res.result = "OK";

    return true;
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_update_server");
    ros::NodeHandle n;
    ros::ServiceServer update_service = n.advertiseService("/update_map", updateCallback);

    ROS_INFO("Ready to update map.");
    ros::spin();

    return 0;

}
