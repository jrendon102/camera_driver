# CAMERA USB DRIVER

### Overview

ROS package that is used to communicate with usb connected cameras.

### Dependencies

OpenCV 4

- Download, installation and set up can be found [here](http://www.codebind.com/linux-tutorials/how-to-install-opencv-in-ubuntu-20-04-lts-for-c-c/).

### Setup

1.  Start by creating a ROS workspace and navigate to your src/ directory. Run the following to clone the directory inside your src/ directory.
    ```
    git clone git@github.com:jrendon102/camera_usb_driver.git
    ```
2.  Build and source your workspace.
3.  Navigate to the camera config file and fill out the configuration file.

    - Before editing the config file find the index of your camera by running the following command on your terminal

      ```
      # Will list all available webcams. (Might need to install v412-utils)
      v412-ctl --list-devices
      ```

      Locate the camera device. If you don't know, disconnect the camera and run the command again to see which device is not listed.
      <br>
      Your camera index should be something along the lines of **/dev/video/number**.
      <br>
      It is usually just **/dev/video/0** and your index would be 0.

    - Once you have that saved somewhere you can go ahead and navigate to the camera.yaml file.

      ```
      #From the top level directory of camera_usb_driver repo.
       cd config/

      # Fill out config file and save.
       vim camera.yaml
      ```

      <mark>**NOTE:**</mark> The config file is already set to parameters that are most likely correct just for convenience.

### Example

1.  After setting up your ROS workspace, building and sourcing, run the following command on the machine <mark>send the video feeds from</mark>. (camera is connected to this machine).

    ```
    # Sets the parameters for camera.
     rosparam set /hardware/camera/fps 30
     rosparam set /hardware/camera/index -1
     rosparam set /hardware/camera/name RPi camera

    # Node to send camera feeds to stream.
     rosrun camera_usb_driver send_video_feeds
    ```

    - <mark>**_IMPORTANT_**</mark> If you are trying to access the camera feeds of a remote machine make sure when you ssh into the remote machine you use the **-X** option. This will allow users to run graphical applications on a remote server.
      ```
      ssh -X <remote_ip_address>
      ```

2.  Next, open a new terminal on the machine that you want to <mark>view the video feeds on </mark>.

    - <mark>**_IMPORTANT_**</mark> Again, if you are trying to view feeds from a remote server you must first export the ROS MASTER URI. This will allow nodes to know where they can locate the master.

      ```
      # On remote machine run the following and copy the output.
       echo $ROS_MASTER_URI

      # On local machine run the following:
       export ROS_MASTER_URI=<paste output here>
      ```

3.  Run the following command:
    ```
    rosrun camera_usb_driver view_video_feeds
    ```
    This will display the video feeds in a new window.
    <br>
    <mark>**NOTE:**</mark> You can terminate the window by pressing the **esc** key.

### Video(s)

**_Coming soon_**

### Author & Maintainer

Julian Rendon (julianrendon514@gmail.com)
