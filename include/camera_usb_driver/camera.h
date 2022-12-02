/**
 * @file camera.h
 * @author Julian Rendon (jarendon10@gmail.com)
 * @brief Class that provides communication and basic functionalities of a standard usb connected
 * camera.
 *
 * The class is used primarily to stream and view video feeds from a usb connected camera source.
 * ROS is used to provide a simple way to transfer video feeds and other important data provided by
 * the usb camera.
 *
 * @version 1.0.0
 * @date 2022-11-30
 * @copyright Copyright (c) 2022
 */
#ifndef CAMERA_H
#define CAMERA_H

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

/**
 * @brief Camera class to communicate with usb connected camera.
 *
 * This class allows communication cameras connected via usb. The functionalities include
 * streaming and viewing video feeds using ROS.
 */
class Camera
{
  private:
    image_transport::Publisher image_pub;
    image_transport::Subscriber image_sub;
    ros::NodeHandle _nh;
    ros::Publisher luminosity_pub;
    int camera_fps, camera_index;
    std::string camera_name;

  public:
    /**
     * Construct a new Camera object. This either creates a ROS publisher(view_feeds=False) or ROS
     * subscriber(view_feeds=True).
     *
     * @param view_feeds Determines whether to view the video feeds.
     */
    Camera(bool view_feeds);

    /**
     * Get ROS parameters set by to camera configuration file. This includes index, name and
     * type of camera.
     *
     */
    void get_parameters();

    /**
     * Calculates the luminosity value of the given image frame. It converts the iamge frame to gray
     * scale the normalizes the avg pixel intensity across all pixel values. Returns luminosity
     * value. 0 = Black image (Totally Dark), 1 = White image(Totally bright)
     *
     * @param frame Image received from camera.
     * @return float
     */
    float get_luminosity_value(cv::Mat image);

    /**
     * Callback function for subscriber. Incoming ROS message is converted to BGR image to display
     * video feeds.
     *
     * @param msg Incoming ROS message.
     */
    void camera_feeds_callback(const sensor_msgs::ImageConstPtr &msg);

    /**
     * Converts the image frames from the camera into ROS message. Frames are published so that
     * videos feeds can be viewed either locally or remotely.
     *
     */
    void send_camera_feeds();
};

#endif