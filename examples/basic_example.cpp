#include "camera.h"
#include <iostream>

constexpr int DISPLAY_DURATION_MS = 5000;

int main()
{
	try
	{
		camera_driver::Camera camera;

		camera.DumpCamSpecs();

		auto frame = camera.CaptureFrame();
		if (!frame)
		{
			std::cerr << "Error: Failed to capture frame.\n";
			return -1;
		}

		if (!camera.DisplayFrame(*frame, DISPLAY_DURATION_MS))
		{
			std::cerr << "Error: Failed to display the frame.\n";
			return -1;
		}
	}
	catch (const std::exception &e)
	{
		std::cerr << "Exception caught: " << e.what() << "\n";
		return -1;
	}
	catch (...)
	{
		std::cerr << "Unknown exception caught\n";
		std::terminate();
	}

	return 0;
}