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

#include <opencv2/opencv.hpp>

namespace CameraUtils
{

/**
 * @brief A map that defines parameter names for camera configuration.
 *
 */
std::map<std::string, std::string> cameraParams = {{"NAME", "hardware/camera/name"},
                                                   {"TYPE", "hardware/camera/type"},
                                                   {"INDEX", "hardware/camera/index"},
                                                   {"FPS", "hardware/camera/fps"}};

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
    std::string name; /**< Name of the camera */
    std::string type; /**< Type of the camera */
    int index;        /**< Index of the camera device */
    int fps;          /**< Frames per second (FPS) */

    // Default Constructor
    CameraInfo() {}

    // Parameterized Constructor
    CameraInfo(std::string name, std::string type, int index, int fps)
        : name(name), type(type), index(index), fps(fps)
    {
    }
};

/**
 * @brief Displays camera information, including its name, type, index, and FPS.
 *
 * @param camera The CameraInfo structure containing camera information.
 */
std::string DumpCamInfo(const CameraInfo &camera)
{
    std::string infoString;
    infoString += "DUMPING CAMERA INFO\n";
    infoString += "  Name: " + camera.name + "\n";
    infoString += "  Type: " + camera.type + "\n";
    infoString += "  Index: " + std::to_string(camera.index) + "\n";
    infoString += "  FPS: " + std::to_string(camera.fps);
    return infoString;
}

/**
 * @brief Converts a string representation of a camera type to a CameraType enum value.
 *
 * @param cameraTypeStr The string representation of the camera type.
 * @param cameraType A reference to a CameraType variable where the result will be stored
 *                  if the conversion is successful.
 * @throws std::invalid_argument if the input string doesn't match any known camera type.
 */
void StrToCameraType(const std::string &cameraTypeStr, CameraType &cameraType)
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
    static cv::Mat CaptureFrame(cv::VideoCapture &videoCap, int &index);

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
    CameraUtils::CameraInfo GetCameraSpecs();

    /**
     * @brief Prints out camera info.
     *
     */
    void PrintCamInfo(const CameraUtils::CameraInfo &camera);
};
#endif