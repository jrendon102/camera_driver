/**
 * @file camera.cpp
 * @author Julian Rendon (jarendon10@gmail.com)
 * @version 1.0.0
 * @date 2022-11-30
 * @copyright Copyright (c) 2022
 */
#include <camera_usb_driver/camera.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>

// Constructor.
Camera::Camera(bool view_feeds)
{
    get_parameters();   // Get the parameters
    image_transport::ImageTransport img_tr(_nh);

    // If feeds is set to true, then we want to view the video feeds.
    if (view_feeds)
    {
        ROS_INFO("Waiting on incoming video feeds from [%s].", camera_name.c_str());
        // Subscriber
        image_sub = img_tr.subscribe("camera/feeds", 1, &Camera::camera_feeds_callback, this);
    }
    else
    {
        ROS_INFO("Setting up video feeds for [%s].", camera_name.c_str());
        // Publishers
        image_pub = img_tr.advertise("camera/feeds", 1);
        luminosity_pub = _nh.advertise<std_msgs::Float32>("/luminosity", 1);
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

// Get luminosity value of image.
float Camera::get_luminosity_value(cv::Mat image)
{
    // Convert image_frame from BGR to Grayscale.
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    // Get the row, col and total number of pixels in the image.
    float image_rows = gray_image.rows;
    float image_cols = gray_image.cols;
    float image_size = image_rows * image.cols;

    // Initialize sum of pixel intensities of image to 0.
    float sum_pixel_values = 0.0;

    // Iterate over all pixel values to get the average pixel intensity.
    // NOTE: Pixel intensities are values between 0 and 255.
    for (int row = 0; row < image_rows; row++)
    {
        for (int col = 0; col < image_cols; col++)
        {
            // NOTE: Use unsigned int 8 because an unit8_t value ranges between 0 and 255.
            sum_pixel_values += (int) gray_image.at<uint8_t>(row, col);
        }
    }

    // Get average pixel intensity.
    float avg_pixel_intensity = sum_pixel_values / image_size;

    // Normalize average pixel to give value between 0 and 1.
    float luminosity_value = (avg_pixel_intensity) / 255;
    return luminosity_value;
};

// Callback function to view video feeds.
void Camera::camera_feeds_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat image_frame =
            cv_bridge::toCvShare(msg, "bgr8")->image;   // Convert ROS msg to BGR image.
        cv::imshow(camera_name, image_frame);           // Display video feeds.

        // If frame is empty, close window and shutdown node.
        if (image_frame.empty())
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
    cv::namedWindow(camera_name);
    cv::VideoCapture video_capture(camera_index, cv::CAP_V4L2);
    ros::Rate loop(camera_fps);   // Camera frames per second.
    sensor_msgs::ImagePtr image_msg;
    std_msgs::Float32 luminosity_value;

    if (!video_capture.isOpened())
    {
        ROS_ERROR("Could not open [%s].", camera_name.c_str());
    }

    else
    {
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

            // Convert CvImage to ROS message and publish message.
            image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_frame).toImageMsg();
            image_pub.publish(image_msg);

            // Get luminosity value and publish message.
            luminosity_value.data = get_luminosity_value(image_frame);
            luminosity_pub.publish(luminosity_value);

            loop.sleep();
        }
    }
    // Release video capture.
    video_capture.release();
    ROS_INFO("Terminated video feeds.");
};
