/**
 * @file camera.cpp
 * @brief A class that facilitates communication and basic operations for a standard camera.
 *
 * This class is designed to streamline the process of capturing and displaying video feeds
 * from a camera source. It includes functionalities such as capturing frames, releasing
 * the camera, and displaying the video.
 *
 * @date 2023-09-11
 * @version 1.1.0
 * @author Julian Rendon (jarendon10@gmail.com)
 * @copyright Copyright (c) 2023
 */
#include <camera_driver/camera.h>

// Default Constructor
Camera::Camera() {}

// Parameterized Constructor
Camera::Camera(std::string name, std::string type, int index, int fps)
    : cameraName(name), cameraType(type), cameraIndex(index), cameraFps(fps)
{
}

// Capture a frame from the camera
// C-02:V4L2 not working to capture frame
// Cannot pass videoCap obj by reference, not sure why
cv::Mat Camera::CaptureFrame(cv::VideoCapture videoCap, int &index)
{
    if (!videoCap.isOpened())
    {
        videoCap.release();
        throw std::runtime_error("Could not open camera with index: " + std::to_string(index));
    }
    cv::Mat frame;
    videoCap >> frame;
    if (frame.empty())
    {
        videoCap.release();
        throw std::runtime_error("Frame is empty.");
    }
    return frame;
}

// Display the captured frame
void Camera::DisplayFrame(const std::string &cameraName, cv::Mat &frame, int duration)
{
    cv::imshow(cameraName, frame);
    cv::waitKey(duration);
}

// Get camera specs
CameraInfo Camera::GetCameraSpecs()
{
    CameraInfo cameraInfo(cameraName, cameraType, cameraIndex, cameraFps);
    return cameraInfo;
}

void Camera::PrintCamInfo(const CameraUtils::CameraInfo &camera)
{
    printf("%s::%s", __func__, CameraUtils::DumpCamInfo(camera).c_str());
}