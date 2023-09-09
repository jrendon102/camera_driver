/**
 * @file camera.h
 * @brief A class that facilitates communication and basic operations for a standard camera.
 *
 * This class is designed to streamline the process of capturing and displaying video feeds
 * from a camera source. It includes functionalities such as capturing frames, releasing
 * the camera, and displaying the video.
 *
 * @date 2023-09-09
 * @version 1.1.0
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @note This code is under copyright (c) 2023.
 */
#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

namespace CameraUtils
{
/**
 * @brief Enum to specify different types of cameras.
 *
 */
enum class CameraType
{
    USB,      /**< Standard USB camera */
    RPI_USB,  /**< USB camera compatible with Raspberry Pi */
    RPI_FLEX, /**< Flexible Raspberry Pi camera */
    THERMAL,  /**< Thermal camera */
    DEPTH     /**< Depth-sensing camera */
};

/**
 * @brief Map from string to CameraType enum.
 *
 */
std::map<std::string, CameraType> cameraTypeMap{
    {"USB", CameraUtils::CameraType::USB},           /**< USB camera */
    {"RPI_USB", CameraUtils::CameraType::RPI_USB},   /**< Raspberry Pi-compatible USB camera */
    {"RPI_FLEX", CameraUtils::CameraType::RPI_FLEX}, /**< Raspberry Pi-compatible flexible camera */
    {"THERMAL", CameraUtils::CameraType::THERMAL},   /**< Thermal camera */
    {"DEPTH", CameraUtils::CameraType::DEPTH}        /**< Depth-sensing camera */
};

/**
 * @brief Struct to hold camera info.
 *
 */
struct CameraInfo
{
    CameraType type;  /**< Type of the camera */
    std::string name; /**< Name of the camera */
    int index;        /**< Index of the camera device */
    int fps;          /**< Frames per second (FPS) */

    // Default Constructor
    CameraInfo() {}

    // Parameterized Constructor
    CameraInfo(std::string name, int index, CameraType type, int fps)
        : name(name), index(index), type(type), fps(fps)
    {
    }
};

/**
 * @brief Converts a string representation of a camera type to a CameraType enum value.
 *
 * @param cameraTypeStr The string representation of the camera type.
 * @param cameraType A reference to a CameraType variable where the result will be stored
 *                  if the conversion is successful.
 * @throws std::invalid_argument if the input string doesn't match any known camera type.
 */
void convertStringToCameraType(const std::string &cameraTypeStr, CameraType &cameraType);

}   // namespace CameraUtils

class Camera
{
  private:
    CameraUtils::CameraType camera_type;         /**< Type of camera */
    std::string camera_name;                     /**< Name of the camera */
    int camera_index;                            /**< Index of the camera device */
    int camera_frame_rate;                       /**< Frame rate for video capture */
    std::unique_ptr<cv::VideoCapture> video_cap; /**< Video capture object */

  public:
    /**
     * @brief Default constructor for the Camera class.
     */
    Camera();

    /**
     * @brief Constructor for the Camera class.
     *
     * @param camera_name The name of the camera.
     * @param camera_type The type of the camera (e.g., USB, IP).
     * @param camera_index The index of the camera device.
     * @param frame_rate The frame rate for video capture.
     * @param preferred_api The preferred video capture API (default is
     * cv::VideoCaptureAPIs::CAP_ANY).
     */
    Camera(const std::string &camera_name, CameraUtils::CameraType camera_type, int camera_index,
           int frame_rate, cv::VideoCaptureAPIs preferred_api = cv::VideoCaptureAPIs::CAP_ANY);

    /**
     * @brief Destroy the Camera object
     *
     */
    ~Camera();

    /**
     * @brief Captures a frame from the camera.
     *
     * @param preferred_api The API type for video capture (default is cv::CAP_ANY).
     * @return A unique pointer to the captured frame as a cv::Mat, or nullptr if capture fails.
     */
    std::unique_ptr<cv::Mat> capture_frame();

    /**
     * @brief Releases the camera.
     */
    void release_camera();

    /**
     * @brief Displays a video frame.
     *
     * @param camera_name The name of the camera (optional).
     * @param frame The frame to display as a cv::Mat.
     * @param duration The duration (in milliseconds) to wait before the next frame is displayed
     * (default is 1 ms).
     */
    static void display_frame(const std::string &camera_name, cv::Mat &frame, int duration = 1);

    /**
     * @brief Get the camera specs.
     *
     * @return camera specs as CameraInfo struct.
     */
    CameraUtils::CameraInfo get_camera_specs();
};
#endif