# Vison ROS Node
This project is a proof of concept for detecting pose estimates for AprilTags in ROS. Although this was built using ROS2 jazzy, it can easily be converted to any distro.

## Overview
`camera -> /camera_frames`

There are two nodes in this package. The camera node is simply for demonstration, as it's advised to use ROS 2 camera drivers [like v4l2_camera](https://index.ros.org/p/v4l2_camera/) for better performance.
This node opens the first available camera device using opencv and publishes Image messages to the topic: /camera_frames. Since this is used specifically for detecting AprilTags, we convert the image to GRAYSCALE as our detector node requires a simple two-dimensional image without color channels.

`/camera_frames -> detector -> /tag36h11`

The detector node leverages [lib-dt-apriltags](https://github.com/duckietown/lib-dt-apriltags) for python bindings to the [Official AprilTag 3 library](https://github.com/AprilRobotics/apriltag). Subscribing to the topic: /camera_frames, this node checks if there are any AprilTags, of a specific family, within the image. If tags are found, the node publishes the pose estimates to a topic of the same name as the AprilTag family.

For this demo, we only check for a single family 'tag36h11', however this can be extended for any [supported family](https://github.com/AprilRobotics/apriltag-imgs).

The pose generated for every tag comes from the lib-dt-apriltags library, but can easily be sent as messages using [apriltags_msgs](https://github.com/Katie-BL/apriltags_msgs.git). Below is the message used for indicating apriltags are present:

```
AprilTagPose[] detections
        std_msgs/String family
                string data
        int32 tag_id
        float64 decision_margin
        float64 pose_err
        float64[9] homography
        float64[2] center
        float64[8] corners
        float64[9] pose_r
        float64[3] pose_t
```

NOTICE: All multi-dimensional arrays are flattened, so the homography and pose_r attributes are 3x3 matrices. The corners attribute is a 4x2 matrix.

### Setup
This repo was created to be cloned within an existing ROS 2 workspace. We assume you have already installed a distro and created a workspace.

Clone this repo
```
git clone https://github.com/Katie-BL/vision.git
```

Clone the repo containing AprilTags msgs
```
git clone https://github.com/Katie-BL/apriltags_msgs.git
```

Since opencv uses a different version of numpy than dt-apriltags, we opt for system installation of dependencies. Although this requires BREAK_SYSTEM_PACKAGES, it was the easiest to work with.
```
PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

Build and source
```
colcon build
source install/setup.bash
```

### Camera Calibration
Hardcoded in the detector node is a camera_params, representing the distortion from the particular camera used. This is very much unique to the hardware, requiring us to generate our own calibration array.

Provided in this repo is a calibrate.py script used for this specific purpose. It requires at least 10 images (.jpg) of the provided calibration checkerboard (pdfs/calibration/checkerboard.jpg). It's very important to use this specific checkerboard and take multiple pictures of the checkerboard at different angles using the camera intended for use in detection. Taking more than 10 photos will only improve the accuracy of the detections. Place the photos within the photos/calibrate directory, as this is where the calibrate.py script expects them.

Once the photos are in the correct directory, you can simply invoke the script:
```
python3 calibrate.py
```

This generates a camera_params.csv in the current directory. Use these numbers for your camera_params within the detection node.


### Demo
Open the camera
```
ros2 run vision camera
```

In a new terminal, run the detector
```
source install/setup.bash
ros2 run vision detector
```

You can now hold AprilTags (of the specified family) in front of the camera and check for pose estimates.
```
ros2 topic echo /tag36h11
```


