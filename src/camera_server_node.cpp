/**
 * @file camera_server_node.cpp
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @brief ROS Publisher node used to send video feeds.
 *
 * This publisher node using the class 'Camera' to capture frames from a camera and
 * converts the CvImage to a ROS message which is then published.
 *
 * @version 1.1.0
 * @date 2023-09-09
 * @copyright Copyright (c) 2023
 */
#include <camera_driver/camera.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#define encoding "bgr8"
#define API      cv::VideoCaptureAPIs::CAP_V4L2

class CameraServerNode : protected Camera
{
  private:
    ros::NodeHandle nh;
    CameraUtils::CameraInfo camera;
    image_transport::ImageTransport imgTr;
    image_transport::Publisher imgPub;
    sensor_msgs::ImagePtr imgMsg;
    cv::VideoCapture videoCap;
    cv::Mat frame;

  public:
    // Constructor.
    CameraServerNode(std::string name, std::string type, int index, int fps)
        : Camera(name, type, index, fps), camera(GetCameraSpecs()), imgTr(nh), videoCap(index, API)
    {
        ROS_INFO("%s::Initializing camera:[%s].", __func__, camera.name.c_str());
        imgPub = imgTr.advertise("/camera/raw_image", 10);
        SendCameraFeeds();
    };

    void SendCameraFeeds()
    {
        ROS_INFO("%s::Video streaming start.", __func__);
        cv::namedWindow(camera.name);
        while (ros::ok())
        {
            try
            {
                frame = CaptureFrame(videoCap, camera.index);
            }
            catch (std::exception &e)
            {
                ROS_ERROR("%s::Exception caught:%s", __func__, e.what());
                throw;   // rethrow exception
            }

            // Convert cv::Mat to ROS message
            imgMsg = cv_bridge::CvImage(std_msgs::Header(), encoding, frame).toImageMsg();
            imgPub.publish(imgMsg);
            ros::Rate(camera.fps).sleep();
        }
        ROS_INFO("Terminated video feeds.");
    }
};

template <typename T>
T GetParam(const std::string &paramName)
{
    ROS_DEBUG("%s::Fetching camera parameter:[%s]", __func__, paramName.c_str());
    T value;
    if (!ros::param::get(paramName, value))
    {
        ROS_ERROR("%s::Failed to retrieve parameter:[%s]", __func__, paramName.c_str());
        throw std::invalid_argument("Failed to retrieve parameter: [" + paramName + "]");
    }
    ROS_DEBUG("%s::Success.", __func__);
    return value;
}

void ConfigureLogLevel(int argc, char **argv)
{
    // Set the default log level to INFO
    ros::console::Level logLevel = ros::console::levels::Info;

    // Check if there are command-line arguments
    if (argc > 1)
    {
        std::string logLevelArg = argv[1];
        if (logLevelArg == "--debug")
        {
            logLevel = ros::console::levels::Debug;
        }
    }

    // Set the log level for ROS messages
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, logLevel);
}

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "camera_server_node");

        // Configure log level based on command-line arguments
        ConfigureLogLevel(argc, argv);

        CameraServerNode camera(GetParam<std::string>(CameraUtils::cameraParams["NAME"]),
                                GetParam<std::string>(CameraUtils::cameraParams["TYPE"]),
                                GetParam<int>(CameraUtils::cameraParams["INDEX"]),
                                GetParam<int>(CameraUtils::cameraParams["FPS"]));
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Exception caught: %s.", e.what());
    }
    return 0;
}