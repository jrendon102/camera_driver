/**
 * @file view_video_feeds.cpp
 * @author Julian Rendon (jarendon10@gmail.com)
 * @brief ROS Subscriber node to view video feeds.
 *
 * This subscriber node uses the Camera class to convert ROS messages to video feeds. The video
 * feeds are displayed in a new window.
 *
 * @version 1.0.0
 * @date 2022-12-01
 * @copyright Copyright (c) 2022
 */
#include <camera_usb_driver/camera.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "view_video_feeds_node");
    Camera video = Camera(true);   // View video feeds.
    ros::spin();
    return 0;
}