/**
 * @file camera_utils.h
 * @brief A collection of utility functions and structures for camera-related operations.
 *
 * This header file defines the `CameraUtils` namespace, which contains utility functions
 * and structures commonly used for camera configuration and information retrieval.
 *
 * @date 2023-09-11
 * @version 1.0.0
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @note This code is under copyright (c) 2023.
 */
#ifndef CAMERA_UTILS_H
#define CAMERA_UTILS_H

#include <map>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>

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
extern std::map<std::string, CameraType> cameraTypeMap;

/**
 * @brief A map that defines parameter names for camera configuration.
 *
 */
extern std::map<std::string, std::string> cameraParams;

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
std::string DumpCamInfo(const CameraInfo &camera);

/**
 * @brief Converts a string representation of a camera type to a CameraType enum value.
 *
 * @param cameraTypeStr The string representation of the camera type.
 * @param cameraType A reference to a CameraType variable where the result will be stored
 *                  if the conversion is successful.
 * @throws std::invalid_argument if the input string doesn't match any known camera type.
 */
void StrToCameraType(const std::string &cameraTypeStr, CameraType &cameraType);

/**
 * @brief Calculate and return the luminosity value of an image.
 *
 * This function takes an image as an input and calculates its luminosity value,
 * which represents the average pixel intensity normalized to the range [0, 1].
 *
 * @param frame The grayscale input image.
 * @return The luminosity value as a float in the range [0, 1].
 */
float GetLuminosityValue(cv::Mat frame);

}   // namespace CameraUtils

#endif
