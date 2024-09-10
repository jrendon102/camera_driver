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
