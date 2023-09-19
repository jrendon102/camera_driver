# Camera Driver
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

The Camera Driver Library is a simple and easy-to-use C++ library designed to interact with standard cameras. It simplifies the process of capturing frames, displaying video feeds, and obtaining camera information. This library can be used to integrate camera functionality into your projects effortlessly.

## Dependencies

The Camera Driver package relies on the following dependency:

- **OpenCV 4**

## Building the Package

To build the Camera Driver package, follow these steps:

1. **Clone the Repository**: Clone the Camera Driver repository to your local machine.

   ```bash
   git clone https://github.com/jrendon102/ros_utils.git
   ```
2. **Navigate to the Project Directory**: Go to the directory where you cloned the repository.
   
   ```bash
   cd camera_driver
   ```

3. **Build the Package**: Use CMake and your preferred C++ compiler to build the package.
   
   ```bash
   mkdir build
   cd build
   cmake ../
   cmake --build .
   ```

4. **Install the Package (Optional)**: If you want to install the package system-wide, you can use the following command:
   
   ```bash
   sudo make install
   ```

After installation, the Camera Driver package should be available for other CMake projects on your system.

5. **Use `find package(camera_driver)` in Your Project**: In your project's `CMakeLists.txt`, use `find_package`
to locate the Camera Driver package:

   ```bash
   find_package(camera_driver REQUIRED)
   ```

Finally, you can now link your project's targets to the Camera Driver package, build and execute your code.

## Uninstalling the Package
To uninstall the Camera Driver package, follow these steps:

1. **Navigate to the Project Directory**: Go to the directory where you cloned the repository.
   
   ```bash
   cd camera_driver
   ```
2. **Uninstall the package**: Uninstall using the following command:

   ```bash
   cd build/
   sudo make uninstall
   ```

## Quick Example

Here's a basic example of how to use the Camera Driver Library:

```cpp
#include <iostream>
#include <stdexcept>
#include <camera_driver/camera.h>

int main() {
    // Create a Camera instance
    Camera camera("Camera", "USB", 0, 60);

    // Get camera specs
    CameraInfo cameraInfo = camera.GetCameraSpecs();

    // Print camera info
    std::cout << "Camera Info:" << std::endl;
    std::cout << "Name: " << cameraInfo.name << std::endl;
    std::cout << "Type: " << cameraInfo.type << std::endl;
    std::cout << "Index: " << cameraInfo.index << std::endl;
    std::cout << "FPS: " << cameraInfo.fps << std::endl;

    try
    {
        cv::VideoCapture videoCapture(0);
        while (true)
        {
            // Capture a frame from the camera (assuming the camera is at index 0)
            cv::Mat frame = camera.CaptureFrame(videoCapture, cameraInfo.index);

            // Display the captured frame for 5 seconds
            camera.DisplayFrame("Camera Feed", frame);
        }
        
    }
    catch(std::exception &e)
    {
        std::cout << "Caught Exception: " << e.what() << std::endl;
    }

    return 0;
}
```
This example demonstrates the basic usage of the Camera Driver Library, capturing frames from a camera and displaying a video stream.

## Author and Maintainer
- Julian Rendon 
- Email: julianrendon514@gmail.com