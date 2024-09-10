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
#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

constexpr float MAX_PIXEL_INTENSITY = 255.0f;

namespace camera_utils
{
    
    struct CameraInfo
    {
        std::string name;
        std::string type;
        int index;
        int fps;

        CameraInfo() : name("Camera"), type("N/A"), index(-1), fps(30) {}

        CameraInfo(std::string name, std::string type, int index, int fps)
            : name(name), type(type), index(index), fps(fps)
        {
        }
    };

    std::optional<float> GetLuminosity(cv::Mat &image);

}   // namespace camera_utils
