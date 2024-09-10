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
#include "camera.h"

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

float CameraUtils::GetLuminosityValue(cv::Mat image)
{
    cv::Mat grayImage;

    // Ensure the input image is in grayscale.
    // Gray scale images have only 1 channel.
    if (image.channels() != 1)
    {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    }

    cv::Scalar meanIntensity = cv::mean(grayImage);

    // The meanIntensity is a Scalar containing B, G, R, and A channels.
    // For grayscale images, only the first channel (B) is relevant.
    float avgPixelIntensity = static_cast<float>(meanIntensity[0]);

    // Normalize average pixel intensity between [0,1].
    float luminosityValue = avgPixelIntensity / 255.0f;

    return luminosityValue;
}