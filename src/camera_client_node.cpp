/**
 * @file camera_client_node.cpp
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
#include <camera_driver/camera.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_client_node_node", ros::init_options::AnonymousName);
    Camera video = Camera(true);   // View video feeds.
    while (ros::ok())
    {
        // ASCII code for esc key is 27.
        // If key is pressed close window and terminate node.
        char key = (char) cv::waitKey(27);
        if (key == 27)
        {
            ROS_INFO("Terminating video feeds.");
            cv::destroyAllWindows;   // Close out all GUI windows.
            break;
        }
        ros::spinOnce();
    }
    return 0;
}
