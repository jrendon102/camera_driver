/**
 * @file camera_server_node.cpp
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @brief ROS Publisher node used to send video feeds.
 *
 * This publisher node using the class 'Camera' to capture frames from a camera and
 * converts the CvImage to a ROS message which is then published.
 *
 * @version 1.1.0
 * @date 2023-09-06
 * @copyright Copyright (c) 2023
 */
#include <camera_driver/camera.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <vector>
class CameraServerNode
{
  private:
    ros::NodeHandle nh;
    ros::Publisher compressed_img_pub;
    std::unique_ptr<Camera> camera;
    std::string camera_name;
    CameraUtils::CameraType camera_type;
    int camera_index;
    int camera_fps;

  public:
    CameraServerNode() : camera(nullptr)
    {
        ROS_DEBUG("Initializing camera setup.");

        if (get_params())
        {
            ROS_DEBUG("Successfully acquired all parameters");
        }
        else
        {
            ROS_ERROR("Failed to get all parameters.");
        }

        camera = std::make_unique<Camera>(camera_name, camera_type, camera_index, camera_fps);
        compressed_img_pub = nh.advertise<sensor_msgs::CompressedImage>("camera/raw_image", 10);
        send_video_stream();
    };

    void send_video_stream()
    {
        ros::Rate rate(camera_fps);
        while (ros::ok())
        {
            std::unique_ptr<cv::Mat> frame = camera->capture_frame(cv::CAP_V4L2);

            if (!frame)
            {
                ROS_ERROR("Failed to capture camera frame");
                break;
            }
            sensor_msgs::CompressedImagePtr camera_msg =
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", *frame)
                    .toCompressedImageMsg(cv_bridge::JPG);
            compressed_img_pub.publish(camera_msg);
            rate.sleep();
        }
    };

    bool get_params()
    {
        bool result;
        ROS_DEBUG("Fetching camera parameters...");
        std::vector<std::string> param_names{
            "hardware/camera/name",
            "hardware/camera/index",
            "hardware/camera/type",
            "hardware/camera/fps",
        };

        std::string camera_type;
        result = nh.getParam(param_names[0], camera_name);
        result = nh.getParam(param_names[1], camera_index);
        result = nh.getParam(param_names[2], camera_type);
        result = nh.getParam(param_names[3], camera_fps);

        // Suppress compiler warning about unused result of the function call.
        (void) CameraUtils::convertStringToCameraType(camera_type, this->camera_type);

        return result;
    };
};

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "camera_server_node");
        CameraServerNode camera;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("An exception occurred during node initialization: %s", e.what());
        return 1;
    }
    return 0;
}