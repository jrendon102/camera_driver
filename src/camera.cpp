/**
 * @file camera.cpp
 * @author Julian Rendon (jarendon10@gmail.com)
 * @version 1.0.0
 * @date 2022-11-30
 * @copyright Copyright (c) 2022
 */
#include <camera_usb_driver/camera.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

// Constructor.
Camera::Camera(bool view_feeds)
{
    get_parameters();   // Get the parameters
    image_transport::ImageTransport img_tr(_nh);

    // If feeds is set to true, then we want to view the video feeds.
    if (view_feeds)
    {
        ROS_INFO("Receiving video feeds for [%s].", camera_name.c_str());
        // Subscriber. Used to view video feeds.
        image_sub = img_tr.subscribe("camera/feeds", 1, &Camera::camera_feeds_callback, this);
    }
    else
    {
        ROS_INFO("Setting up video feeds for [%s].", camera_name.c_str());
        // Publisher. Used to send video feeds.
        image_pub = img_tr.advertise("camera/feeds", 1);
        send_camera_feeds();
    }
};

// Get ROS parameters set by camera config file.
void Camera::get_parameters()
{
    ROS_INFO("Setting up camera parameters.");
    _nh.getParam("camera/fps", camera_fps);
    _nh.getParam("camera/index", camera_index);
    _nh.getParam("camera/name", camera_name);
};

// Callback function to view video feeds.
void Camera::camera_feeds_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat image_frame =
            cv_bridge::toCvShare(msg, "bgr8")->image;   // Convert ROS msg to BGR image.
        cv::imshow(camera_name, image_frame);           // Display video feeds.

        // If esc key is pressed or frame is empty, close window and shutdown node.
        char key = (char) cv::waitKey(25);
        if (key == 27 || image_frame.empty())
        {
            ROS_WARN("Exiting video feeds.");
            ros::shutdown();
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from [%s] to bgr8.", msg->encoding.c_str());
    }
};

// Convert video feeds into ROS messages.
void Camera::send_camera_feeds()
{
    ros::Rate loop(camera_fps);   // Camera frames per second.
    cv::namedWindow(camera_name);
    cv::VideoCapture video_capture(camera_index, cv::CAP_V4L2);

    if (!video_capture.isOpened())
    {
        ROS_ERROR("Could not open [%s].", camera_name.c_str());
        ros::shutdown;
    }

    // Send camera feeds to create a video.
    while (ros::ok())
    {
        cv::Mat image_frame;
        video_capture >> image_frame;

        // Verify that the incoming frame is not empty.
        if (image_frame.empty())
        {
            ROS_ERROR("Frame is empty! Releasing [%s].", camera_name.c_str());
            break;
        }

        // Convert CvImage to ROS message and publish.
        sensor_msgs::ImagePtr image_msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_frame).toImageMsg();
        image_pub.publish(image_msg);

        loop.sleep();
    }

    video_capture.release();   // Release video capture.
    cv::destroyAllWindows();   // Close frame.
    ROS_INFO("Terminated video feeds.");
};