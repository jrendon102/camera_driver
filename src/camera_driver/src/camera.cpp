/**
 * @file camera.cpp
 * @brief A class that facilitates communication and basic operations for a standard camera.
 *
 * This class is designed to streamline the process of capturing and displaying video feeds
 * from a camera source. It includes functionalities such as capturing frames, releasing
 * the camera, and displaying the video.
 *
 * @date 2023-09-11
 * @version 1.1.0
 * @author Julian Rendon (jarendon10@gmail.com)
 * @copyright Copyright (c) 2023
 */
#include "camera.h"

camera_driver::Camera::Camera()
	: cameraName("Camera"),
	cameraType("N/A"),
	cameraIndex(DEFAULT_CAMERA_INDEX),
	cameraFps(DEFAULT_CAMERA_FPS),
	vidCap(cameraIndex)
{
}

camera_driver::Camera::Camera(std::string name, std::string type, int index, int fps)
	: cameraName(std::move(name)),
	cameraType(std::move(type)),
	cameraIndex(index),
	cameraFps(fps),
	vidCap(cameraIndex)
{
}

camera_driver::Camera::~Camera()
{
	if (vidCap.isOpened())
	{
		vidCap.release();
		cv::destroyAllWindows();
	}
}

std::optional<cv::Mat> camera_driver::Camera::CaptureFrame()
{
	if (!vidCap.isOpened())
	{
		std::cerr << __func__ << "::Error: Could not open camera with index: "
			<< cameraIndex << ".\n";
		return std::nullopt;
	}

	cv::Mat frame;
	vidCap >> frame;

	if (frame.empty())
	{
		std::cerr << __func__ << "::Error: Frame is empty." << cameraIndex << "\n";
		return std::nullopt;
	}

	return frame;
}

bool camera_driver::Camera::DisplayFrame(
	cv::Mat& frame,
	int duration,
	const std::optional<std::string>& windowName)
{
	auto displayName = windowName.value_or(this->cameraName);

	cv::imshow(displayName, frame);
	cv::waitKey(duration);

	return cv::getWindowProperty(displayName, cv::WND_PROP_VISIBLE) != 0;
}

std::optional<camera_utils::CameraInfo> camera_driver::Camera::GetCameraSpecs() const
{
	return camera_utils::CameraInfo(cameraName, cameraType, cameraIndex, cameraFps);
}

std::string camera_driver::Camera::DumpCamSpecs() const
{
	auto camSpecs = GetCameraSpecs();
	if (!camSpecs)
	{
		return "Error: Failed to retrieve camera specs.";
	}

	std::string infoString;
	infoString += "DUMPING CAMERA INFO\n";
	infoString += "  Name: " + camSpecs->name + "\n";
	infoString += "  Type: " + camSpecs->type + "\n";
	infoString += "  Index: " + std::to_string(camSpecs->index) + "\n";
	infoString += "  FPS: " + std::to_string(camSpecs->fps);
	return infoString;
}