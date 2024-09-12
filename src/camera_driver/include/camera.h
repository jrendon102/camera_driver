#pragma once

#include "camera_utils.h"
#include <opencv2/opencv.hpp>
#include <string>

constexpr int DEFAULT_DURATION_MS = 3000;

namespace camera_driver
{
	class Camera
	{
	public:
		Camera();

		Camera(std::string name, std::string type, int index, int fps);

		~Camera();

		// Disable copy constructor - copying Camera is not allowed.
		Camera(const Camera &) = delete;

		// Disable copy assignment - prevents copying of Camera.
		Camera &operator=(const Camera &) = delete;

		// Enable move constructor - allows moving resources efficiently.
		Camera(Camera &&) noexcept = default;

		// Enable move assignment - allows move assignment for resource transfer.
		Camera &operator=(Camera &&) noexcept = default;

		[[nodiscard]] std::optional<cv::Mat> CaptureFrame();

		bool DisplayFrame(
			cv::Mat &frame,
			int duration = DEFAULT_DURATION_MS,
			const std::optional<std::string> &windowName = std::nullopt);

		[[nodiscard]] std::optional<camera_utils::CameraInfo> GetCameraSpecs() const;

		void DumpCamSpecs() const;

	private:
		std::string cameraName;
		std::string cameraType;
		int cameraIndex;
		int cameraFps;
		cv::VideoCapture vidCap;
	};

} // namespace camera_driver
