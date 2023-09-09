/**
 * @file camera_client_node.cpp
 * @author Julian Rendon (jarendon10@gmail.com)
 * @brief ROS Subscriber node to view video feeds.
 *
 * This subscriber node uses the Camera class to convert ROS messages to video feeds. The video
 * feeds are displayed in a new window.
 *
 * @version 1.0.0
 * @date 2023-09-06
 * @copyright Copyright (c) 2023
 */
#include <camera_driver/camera.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

#define DEFAULT "Camera"

class CameraClientNode
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber compressed_img_sub;
    sensor_msgs::ImagePtr decompressed_img;

    std::string camera_name;
    std::unique_ptr<cv::Mat> frame;

  public:
    CameraClientNode() : frame(nullptr)
    {
        if (nh.getParam("hardware/camera/name", camera_name))
        {
            ROS_INFO("Successfully acquired camera parameter.");
        }
        else
        {
            camera_name = DEFAULT;
            ROS_WARN("Could not get camera name setting it to %s", camera_name.c_str());
        }
        compressed_img_sub =
            nh.subscribe("/camera/raw_image", 10, &CameraClientNode::compressed_image_cb, this);
    };

    ~CameraClientNode()
    {
        if (frame)
        {
            cv::destroyAllWindows();
        }
    };

    void compressed_image_cb(const sensor_msgs::CompressedImageConstPtr &img_msg)
    {

        try
        {
            if (!frame)
            {
                frame = std::make_unique<cv::Mat>();
            }
            decompressed_img = cv_bridge::toCvCopy(img_msg, "bgr8")->toImageMsg();
            *frame = cv_bridge::toCvShare(decompressed_img, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Unable to perform the conversion to BGR8 from the provided image "
                      "format: [%s].",
                      decompressed_img->encoding.c_str());
        }
    };

    void loop()
    {
        ros::Rate rate(60);
        while (ros::ok())
        {
            if (frame)
            {
                Camera::display_frame(camera_name, *frame);
            }
            ros::spinOnce();
            rate.sleep();
        }
    };
};

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "camera_client_node");
        CameraClientNode camera;
        camera.loop();
    }
    catch (ros::Exception &e)
    {
        ROS_ERROR("Error ocurred: %s", e.what());
    }
    return 0;
}
