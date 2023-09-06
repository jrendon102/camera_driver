# Camera Driver ROS Package

## Overview

The **Camera Driver** ROS package enables seamless communication with USB-connected cameras, enhancing your robotic system's ability to capture and work with video feeds. This package simplifies camera integration within the ROS framework.

## Dependencies

The Camera Driver package relies on the following dependency:

- **OpenCV 4**: To install and set up OpenCV 4, refer to [this installation guide](http://www.codebind.com/linux-tutorials/how-to-install-opencv-in-ubuntu-20-04-lts-for-c-c/).

## Setup

Follow these steps to set up and use the Camera Driver package:

1. Begin by creating a ROS workspace and navigating to its `src/` directory. Use the following command to clone the Camera Driver package into your `src/` directory:

    ```bash
    git clone git@github.com:jrendon102/camera_driver.git
    ```

2. Build and source your ROS workspace.

3. Navigate to the camera configuration file and fill out the necessary configuration:

    - Before editing the configuration file, find the index of your camera by running the following command in your terminal:

        ```bash
        # List all available webcams (You might need to install v4l2-utils).
        v4l2-ctl --list-devices
        ```

        Locate the camera device; it should resemble something like `/dev/video/number`. The index is usually just `/dev/video/0`.

    - Once you have the camera index, navigate to the `camera.yaml` file:

        ```bash
        # From the top-level directory of the camera_driver repo.
        cd config/

        # Edit the config file.
        vim camera.yaml
        ```

        **Note:** The config file already contains default parameters that should work for most cases.

## Example

Follow these steps to run the Camera Driver package and view the video feeds:

1. After setting up your ROS workspace, building, and sourcing it, run the following command on the machine that you want to capture video feeds from (where the camera is connected):

    ```bash
    # Set camera parameters.
    rosparam set /hardware/camera/fps 30
    rosparam set /hardware/camera/index -1
    rosparam set /hardware/camera/name "RPi camera"

    # Start the node to send camera feeds to stream.
    rosrun camera_driver camera_server_node
    ```

    - **Important:** If you are trying to access camera feeds from a remote machine, ensure you use the `-X` or `-Y` option when SSHing into the remote machine to enable graphical applications:

        ```bash
        ssh -X <remote_ip_address>
        ```

2. Open a new terminal on the machine where you want to view the video feeds:

    - **Important:** If you are trying to view feeds from a remote server, you must export the ROS MASTER URI to allow nodes to locate the master:

        ```bash
        # On the remote machine, run the following and copy the output.
        echo $ROS_MASTER_URI

        # On the local machine, run the following:
        export ROS_MASTER_URI=<paste output here>
        ```

3. Run the following command to display the video feeds in a new window:

    ```bash
    rosrun camera_driver camera_client_node
    ```

    The video feeds will be displayed in a new window.

    - **Note:** You can terminate the window by pressing the `esc` key.

## Videos

Coming soon...

## Author & Maintainer

Julian Rendon (julianrendon514@gmail.com)