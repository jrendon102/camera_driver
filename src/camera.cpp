/**
 * @file camera.cpp
 * @author Julian Rendon (jarendon10@gmail.com)
 * @brief A class that facilitates communication and basic operations for a standard camera.
 *
 * This class is designed to streamline the process of capturing and displaying video feeds
 * from a camera source. It includes functionalities such as capturing frames, releasing
 * the camera, and displaying the video.
 *
 * @version 1.1.0
 * @date 2023-09-09
 * @copyright Copyright (c) 2023
 */
#include <camera_driver/camera.h>

namespace CameraUtils
{

void convertStringToCameraType(const std::string &cameraTypeStr, CameraType &cameraType)
{
    if (cameraTypeMap.find(cameraTypeStr) != cameraTypeMap.end())
    {
        cameraType = cameraTypeMap[cameraTypeStr];
    }
    else
    {
        throw std::invalid_argument("Invalid camera type: " + cameraTypeStr);
    }
};

}   // namespace CameraUtils

// Default Constructor
Camera::Camera() {}

// Parameterized Constructor
Camera::Camera(const std::string &camera_name, CameraUtils::CameraType camera_type,
               int camera_index, int frame_rate, cv::VideoCaptureAPIs preferred_api)
    : video_cap(std::make_unique<cv::VideoCapture>(camera_index, preferred_api))
{
    this->camera_name = camera_name;
    this->camera_type = camera_type;
    this->camera_index = camera_index;
    this->camera_frame_rate = frame_rate;
}

// Capture a frame from the camera
std::unique_ptr<cv::Mat> Camera::capture_frame()
{
    if (!video_cap->isOpened())
    {
        return nullptr;
    }

    std::unique_ptr<cv::Mat> frame = std::make_unique<cv::Mat>();
    *video_cap >> *frame;
    if (frame->empty())
    {
        return nullptr;
    }
    return frame;
}

// Display the video feed from the camera
void Camera::display_video(const std::string &camera_name, cv::Mat &frame)
{
    cv::imshow(camera_name, frame);
    cv::waitKey(1);
}

// Release the camera resources
void Camera::release_camera()
{
    video_cap->release();
    cv::destroyAllWindows();
}

// Get camera specs
CameraUtils::CameraInfo Camera::get_camera_specs()
{
    CameraUtils::CameraInfo camera_info;
    camera_info.name = camera_name;
    camera_info.index = camera_index;
    camera_info.type = camera_type;
    camera_info.fps = camera_frame_rate;

    return camera_info;
}