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
#pragma once

#include "camera_utils.h"
#include <opencv2/opencv.hpp>
#include <string>

constexpr int DEFAULT_CAMERA_INDEX = 0;
constexpr int DEFAULT_CAMERA_FPS = 30;

namespace camera_driver
{
	class Camera
	{
	public:
		Camera();

		Camera(std::string name, std::string type, int index, int fps);

		~Camera();

		std::optional<cv::Mat> CaptureFrame();

		bool DisplayFrame(
			cv::Mat& frame,
			int duration = 3000,
			const std::optional<std::string>& windowName = std::nullopt);

		std::optional<camera_utils::CameraInfo> GetCameraSpecs() const;

		std::string DumpCamSpecs() const;

	private:
		std::string cameraName;
		std::string cameraType;
		int cameraIndex;
		int cameraFps;
		cv::VideoCapture vidCap;
	};

}	// namespace camera_driver
