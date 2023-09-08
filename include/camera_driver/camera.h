/**
 * @file camera.h
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @brief A class that facilitates communication and basic operations for a standard camera.
 *
 * This class is designed to streamline the process of capturing and displaying video feeds
 * from a camera source. It includes functionalities such as capturing frames, releasing
 * the camera, and displaying the video.
 *
 * @version 1.1.0
 * @date 2022-11-30
 * @copyright Copyright (c) 2023
 */
#include <opencv2/opencv.hpp>

namespace CameraUtils
{
/**
 * @brief Enum to specify different types of cameras
 *
 */
enum CameraType
{
    NONE,       // Represents no camera
    USB,        // Represents a standard USB camera
    RPI_USB,    // Represents a USB camera compatible with Raspberry Pi
    RPI_FLEX,   // Represents a flexible Raspberry Pi camera
    THERMAL,    // Represents a thermal camera
    DEPTH       // Represents a depth-sensing camera
};

/**
 * @brief  Map from string to CameraType enum
 *
 */
std::map<std::string, CameraType> cameraTypeMap = {
    {"NONE", CameraType::NONE},       {"USB", CameraType::USB},
    {"RPI_USB", CameraType::RPI_USB}, {"RPI_FLEX", CameraType::RPI_FLEX},
    {"THERMAL", CameraType::THERMAL}, {"DEPTH", CameraType::DEPTH}};

/**
 * @brief Converts a string representation of a camera type to a CameraType enum value.
 *
 * @param cameraTypeStr The string representation of the camera type.
 * @param cameraType A reference to a CameraType variable where the result will be stored if the
 * conversion is successful.
 * @return true if the conversion succeeds, false otherwise.
 */
bool convertStringToCameraType(const std::string &cameraTypeStr, CameraType &cameraType);

/**
 * @brief Struct to hold camera info
 *
 */
struct CameraInfo
{
    CameraType type;    // Type of the camera
    std::string name;   // Name of the camera
    int index;          // Index of the camera device
    int fps;            // Frames per second (FPS)
};
}   // namespace CameraUtils

class Camera
{
  private:
    CameraUtils::CameraType camera_type;           // Type of camera
    std::string camera_name;                       // Name of the camera
    int camera_index;                              // Index of the camera device
    int camera_frame_rate;                         // Frame rate for video capture
    std::unique_ptr<cv::VideoCapture> video_cap;   // Video capture object

  public:
    /**
     * @brief Default constructor for the Camera class.
     */
    Camera();

    /**
     * @brief Constructor for the Camera class.
     *
     * @param camera_name The name of the camera.
     * @param camera_index The index of the camera device.
     * @param frame_rate The frame rate for video capture.
     */
    Camera(const std::string &camera_name, CameraUtils::CameraType camera_type, int camera_index,
           int frame_rate);

    /**
     * @brief Destructor for the Camera class.
     */
    ~Camera();

    /**
     * @brief Captures a frame from the camera (static).
     *
     * @param preferred_api The API type for video capture (default is cv::CAP_ANY).
     * @return A pointer to the captured frame as a cv::Mat.
     */
    std::unique_ptr<cv::Mat> capture_frame(cv::VideoCaptureAPIs preferred_api = cv::CAP_ANY);

    /**
     * @brief Releases the camera.
     */
    void release_camera();

    /**
     * @brief Displays the video feed from the camera (static).
     *
     * @param camera_name The name of the camera (optional).
     * @param frame The frame to display as a cv::Mat.
     */
    static void display_video(const std::string &camera_name, cv::Mat &frame);

    /**
     * @brief Get the camera specs object
     *
     * @return camera specs as CameraInfo struct.
     */
    CameraUtils::CameraInfo get_camera_specs();
};