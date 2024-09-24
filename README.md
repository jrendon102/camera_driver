# Camera Driver
[![License](https://img.shields.io/badge/license-GNU%20GPL-blue.svg)](LICENSE)

The Camera Driver Library is a simple and easy-to-use C++ library designed to interact with standard cameras. It simplifies the process of capturing frames, displaying video feeds, and obtaining camera information. This library can be used to integrate camera functionality into your projects effortlessly.

## Dependencies

The Camera Driver package relies on the following dependency:

- **OpenCV 4**

## Version

Current Version: 2.0.1

## Building the Package

### For Linux OS

1. **Clone the Repository**: Clone the Camera Driver repository to your local machine.

   ```bash
   git clone git@github.com:jrendon102/camera_driver.git
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

4. **Install the Package (Optional)**: If you want to use the package with other CMake projects or system-wide, you can install it by running the following command from within the build directory:
   
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

### For Windows OS
1. **Open Developer Command Prompt or PowerShell for VS 2022**: Ensure you use the specialized command prompt or PowerShell for Visual Studio 2022 as it automatically sets up the environment for using cl.exe and other related tools.

2. **Clone the Repository**: Clone the Camera Driver repository to your local machine.

   ```powershell
   git clone git@github.com:jrendon102/camera_driver.git
   ```

3. **Navigate to the Project Directory**: Go to the directory where you cloned the repository.
   
   ```powershell
   cd camera_driver
   ```

4. **Build the Package**: Use CMake and your preferred C++ compiler to build the package. (This example uses Visual Studio 17 2022)
   
   ```powershell
   mkdir build
   cd build
   cmake ../ --preset x64-release
   cmake --build x64-release --config Release
   ```

5. **Install the Package (Optional)**: If you want to use the package with other CMake projects or system-wide, you can install it by running the following command from within the build directory:
   
   ```powershell
   cmake --install x64-release
   ```

After installation, the Camera Driver package should be available for other CMake projects on your system.

6. **Use `find package(camera_driver)` in Your Project**: In your project's `CMakeLists.txt`, use `find_package`
to locate the Camera Driver package:

   ```powershell
   find_package(camera_driver REQUIRED)
   ```

Finally, you can now link your project's targets to the Camera Driver package, build and execute your code.

## Uninstalling the Package

### For Linux OS

1. **Navigate to the Project Directory**: Go to the directory where you cloned the repository.
   
   ```bash
   cd camera_driver
   ```
2. **Uninstall the package**: Uninstall using the following command:

   ```bash
   cd build/
   sudo make uninstall
   ```

### For Windows OS
1. **Open Developer Command Prompt or PowerShell for VS 2022**: Ensure you use the specialized command prompt or PowerShell for Visual

2. **Navigate to the Project Build Directory**: Go to the directory where you cloned the repository.
   
   ```powershell
   cd camera_driver/build
   ```
3. **Uninstall the package**: Uninstall using the following command:

   ```powershell
	cmake --build x64-release --target uninstall --config Release
   ```

## Examples

### For Windows OS
After building the project, you need to navigate to the `build/x64-release/examples/Release` directory to find the `basic_example.exe` file. You can run the executable from this location to see the example in action.
## Author and Maintainer
- Julian Rendon 
- Email: julianrendon514@gmail.com