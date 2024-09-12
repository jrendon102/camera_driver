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
	cv::Mat &frame,
	int duration,
	const std::optional<std::string> &windowName)
{
	auto displayName = windowName.value_or(this->cameraName);

	cv::imshow(displayName, frame);
	cv::waitKey(duration);

	// Return false if the window has been closed by the user; ensures loop
	// termination if used for video streaming
	return cv::getWindowProperty(displayName, cv::WND_PROP_VISIBLE) != 0;
}

std::optional<camera_utils::CameraInfo> camera_driver::Camera::GetCameraSpecs() const
{
	return camera_utils::CameraInfo(this->cameraName, this->cameraType, this->cameraIndex, this->cameraFps);
}

void camera_driver::Camera::DumpCamSpecs() const
{
	auto camSpecs = GetCameraSpecs();
	if (!camSpecs)
	{
		std::cerr << __func__ << "::Error: Failed to retrieve camera specs.\n";
		return;
	}

	std::string infoString;
	infoString += "DUMPING CAMERA INFO\n";
	infoString += "  Name: " + camSpecs->name + "\n";
	infoString += "  Type: " + camSpecs->type + "\n";
	infoString += "  Index: " + std::to_string(camSpecs->index) + "\n";
	infoString += "  FPS: " + std::to_string(camSpecs->fps);
	std::cout << __func__ << "::" << infoString << ".\n";
}