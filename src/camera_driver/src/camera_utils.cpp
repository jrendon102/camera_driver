#include "camera.h"

std::optional<float> camera_utils::GetLuminosity(cv::Mat &image)
{
    if (image.empty())
    {
        std::cerr << __func__ << "::Error: Input image is empty.\n";
        return std::nullopt;
    }

    cv::Mat grayImage;
    if (image.channels() != 1)
    {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    }
    else
    {
        grayImage = image;
    }

    // Calculate the mean pixel intensity of the grayscale image
    cv::Scalar meanIntensity = cv::mean(grayImage);

    // Calculate the average pixel intensity and normalize it
    float avgPixelIntensity = static_cast<float>(meanIntensity[0]);
    float luminosityValue = avgPixelIntensity / MAX_PIXEL_INTENSITY;

    return luminosityValue;
}