#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

constexpr int DEFAULT_CAMERA_FPS = 30;
constexpr int DEFAULT_CAMERA_INDEX = 0;
constexpr float MAX_PIXEL_INTENSITY = 255.0f;

namespace camera_utils
{

    struct CameraInfo
    {
        std::string name;
        std::string type;
        int index;
        int fps;

        CameraInfo() : name("Camera"), type("N/A"), index(DEFAULT_CAMERA_INDEX), fps(DEFAULT_CAMERA_FPS) {}

        CameraInfo(const std::string &name, const std::string &type, int index, int fps)
            : name(name), type(type), index(index), fps(fps)
        {
        }
    };

    std::optional<float> GetLuminosity(cv::Mat &image);

} // namespace camera_utils
