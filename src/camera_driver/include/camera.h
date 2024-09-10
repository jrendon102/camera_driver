/**
 * @file camera.h
 * @brief A class that facilitates communication and basic operations for a standard camera.
 *
 * This class is designed to streamline the process of capturing and displaying video feeds
 * from a camera source. It includes functionalities such as capturing frames, releasing
 * the camera, and displaying the video.
 *
 * @date 2023-09-11
 * @version 1.1.0
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @note This code is under copyright (c) 2023.
 */
#ifndef CAMERA_H
#define CAMERA_H

#include "camera_utils.h"
#include <opencv2/opencv.hpp>

using namespace CameraUtils;

class Camera
{
  private:
    std::string cameraName; /**< Name of the camera */
    std::string cameraType; /**< Type of camera */
    int cameraIndex;        /**< Index of the camera device */
    int cameraFps;          /**< Frame rate for video capture */

  public:
    /**
     * @brief Default constructor for the Camera class.
     */
    Camera();

    /**
     * @brief Constructor for the Camera class.
     *
     * @param cameraName    Name of the camera.
     * @param cameraType    Type of the camera (e.g., USB, IP).
     * @param cameraIndex   Index of the camera device.
     * @param fps           Frame per second for video capture.

     */
    Camera(std::string cameraName, std::string cameraType, int cameraIndex, int fps);

    /**
     * @brief Captures a frame from the camera.
     *
     * @param videoCap The video capture object to use for capturing the frame.
     * @param index The camera index
     * @return The captured frame as a cv::Mat
     * @throws std::runtime_error if the camera cannot be opened or if the captured frame is empty.
     */
    static cv::Mat CaptureFrame(cv::VideoCapture videoCap, int &index);

    /**
     * @brief Displays a video frame.
     *
     * @param cameraName The name of the camera (optional).
     * @param frame The frame to display as a cv::Mat.
     * @param duration The duration (in milliseconds) to wait before the next frame is displayed
     *                (default is 1 ms).
     */
    static void DisplayFrame(const std::string &cameraName, cv::Mat &frame, int duration = 1);

    /**
     * @brief Get the camera specs.
     *
     * @return camera specs as CameraInfo struct.
     */
    CameraInfo GetCameraSpecs();

    /**
     * @brief Prints out camera info.
     *
     */
    void PrintCamInfo(const CameraInfo &camera);
};
#endif