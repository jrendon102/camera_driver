/**
 * @file camera_client_node.cpp
 * @author Julian Rendon (jarendon10@gmail.com)
 * @brief ROS Subscriber node to view video feeds.
 *
 * This subscriber node uses the Camera class to convert ROS messages to video feeds. The video
 * feeds are displayed in a new window.
 *
 * @version 1.1.0
 * @date 2023-09-11
 * @copyright Copyright (c) 2023
 */
#include <camera_driver/camera.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

#define PARAM CameraUtils::cameraParams["NAME"]

class CameraClientNode
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber compImgSub;
    sensor_msgs::ImagePtr compMsg;
    std::unique_ptr<cv::Mat> frame;
    std::string cameraName;

  public:
    CameraClientNode() : frame(nullptr)
    {
        if (!nh.getParam(PARAM, cameraName))
        {
            ROS_ERROR("%s::Failed to retrieve parameter:[%s]", __func__, PARAM.c_str());
            throw std::invalid_argument("Failed to retrieve parameter: [" + PARAM + "]");
        }
        compImgSub = nh.subscribe("/camera/raw_image/compressed", 10,
                                  &CameraClientNode::CompressedImgCb, this);

        DisplayVideo();
    };

    ~CameraClientNode()
    {
        if (frame)
        {
            cv::destroyAllWindows();
        }
    };

    void CompressedImgCb(const sensor_msgs::CompressedImageConstPtr &img_msg)
    {

        try
        {
            if (!frame)
            {
                frame = std::make_unique<cv::Mat>();
            }
            compMsg = cv_bridge::toCvCopy(img_msg, "bgr8")->toImageMsg();
            *frame = cv_bridge::toCvShare(compMsg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("%s::Unable to perform conversion format:[%s]", __func__,
                      compMsg->encoding.c_str());
            throw std::runtime_error("Unable to perform the conversion from the provided image");
        }
    };

    void DisplayVideo()
    {
        ros::Rate rate(60);
        try
        {
            while (ros::ok())
            {
                if (frame)
                {
                    Camera::DisplayFrame(cameraName, *frame);
                }
                ros::spinOnce();
                rate.sleep();
            }
        }
        catch (std::exception &e)
        {
            throw;   // rethrow exception
        }
        cv::destroyAllWindows();
    };
};

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "camera_client_node");
        CameraClientNode camera;
    }
    catch (ros::Exception &e)
    {
        ROS_ERROR("Exception caught: %s.", e.what());
    }
    return 0;
}
