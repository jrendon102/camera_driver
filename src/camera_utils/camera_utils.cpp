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
#include <camera_driver/camera_utils.h>

std::map<std::string, CameraUtils::CameraType> CameraUtils::cameraTypeMap{
    {"USB", CameraUtils::CameraType::USB},           /**< USB camera */
    {"RPI_USB", CameraUtils::CameraType::RPI_USB},   /**< Raspberry Pi-compatible USB camera */
    {"RPI_FLEX", CameraUtils::CameraType::RPI_FLEX}, /**< Raspberry Pi-compatible flexible camera */
    {"THERMAL", CameraUtils::CameraType::THERMAL},   /**< Thermal camera */
    {"DEPTH", CameraUtils::CameraType::DEPTH}        /**< Depth-sensing camera */
};

std::map<std::string, std::string> CameraUtils::cameraParams = {{"NAME", "hardware/camera/name"},
                                                                {"TYPE", "hardware/camera/type"},
                                                                {"INDEX", "hardware/camera/index"},
                                                                {"FPS", "hardware/camera/fps"}};

std::string CameraUtils::DumpCamInfo(const CameraUtils::CameraInfo &camera)
{
    std::string infoString;
    infoString += "DUMPING CAMERA INFO\n";
    infoString += "  Name: " + camera.name + "\n";
    infoString += "  Type: " + camera.type + "\n";
    infoString += "  Index: " + std::to_string(camera.index) + "\n";
    infoString += "  FPS: " + std::to_string(camera.fps);
    return infoString;
}

void CameraUtils::StrToCameraType(const std::string &cameraTypeStr,
                                  CameraUtils::CameraType &cameraType)
{
    if (cameraTypeMap.find(cameraTypeStr) != cameraTypeMap.end())
    {
        cameraType = cameraTypeMap[cameraTypeStr];
    }
    else
    {
        throw std::invalid_argument("Invalid camera type: " + cameraTypeStr);
    }
}