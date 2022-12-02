/**
 * @file send_video_feeds.cpp
 * @author Julian Rendon (jarendon10@gmail.com)
 * @brief ROS Publisher node used to send video feeds.
 *
 * This publisher node using the class Camera to capture frames from a usb connected camera and
 * converts the CvImage to a ROS message.
 *
 * @version 1.0.0
 * @date 2022-12-01
 * @copyright Copyright (c) 2022
 */
#include <camera_usb_driver/camera.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_video_feeds_node");
    Camera video = Camera(false);   // Publish video feeds.
    return 0;
}