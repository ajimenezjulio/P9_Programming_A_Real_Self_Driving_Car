## Programming a Real Self-Driving Car
[![C++](https://img.shields.io/badge/C++-Solutions-blue.svg?style=flat&logo=c%2B%2B)](http://www.cplusplus.org/)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
[![Python 3.6](https://img.shields.io/badge/python-3.6-blue.svg)](https://www.python.org/downloads/release/python-360/)

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9). 

<img src="https://github.com/ajimenezjulio/P9_Programming_A_Real_Self_Driving_Car/blob/master/imgs/iOS13-Course-Syllabus.gif">
</p>

## Goal
This project aims to drive a real car by following the same architecture of nodes such as the route follower, classification of traffic lights and control signals for acceleration, brake and steering wheel rotation. The functionality was implemented using Robot Operative System (ROS) as the main framework for node and communication development.

## Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

## Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

## Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

## Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Download the classifier models for simulator and real images from [here](https://drive.google.com/drive/folders/1I7Ns8ZYgLdNgZfrUlwc2_X-82oy_ASlb?usp=sharing) and place them inside `/ros/src/tl_detector/light_classification/models`.

3. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```

4. Inside `/ros/src/tl_detector/tl_detector.py`
    - Change flag in line `17` for mode setup (`True` for simulator traffic light data, `False` for using classifier models. By default is `True`)
    - Change model name on line `18` (`final_frozen_graph_sim.pb` for simulator images model, and `final_frozen_graph_sim.pb` for real images model. By default simulator images is selected).

5. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
6. Run the simulator

## Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

## Project Overview

### Udacity's Car (Carla) Architecture
Carla is the custom automobile that Udacity has converted into a self-driving car. It's self-driving system is broken down into four major sub-systems: Sensors, Perception, Planning and Control as presented below.

<img src="https://github.com/ajimenezjulio/P9_Programming_A_Real_Self_Driving_Car/blob/master/imgs/architecture.png">
</p>

**Sensors**
Includes everything needed to understand its surroundings and location including cameras, lidar, GPS, radar, and IMU.

**Perception**
Abstracts sensor inputs into object detection and localization.

- **Detection**
    - Includes software pipelines for vehicle detection, traffic light detection, obstacle detection, etc.
    - Techniques in image manipulation include Histogram of Oriented Gradients (HOG) feature extraction, color transforms, spacial binning.
    - Methods of classification include sliding-window or sub-sampling along with heat maps and bounding boxes for recurring detections.
    
- **Localization**
    - Answers the question: “Where is our car in a given map with an accuracy of 10cm or less?”
    - Based on the notion that GPS is not accurate enough.
    - Onboard sensors are used to estimate transformation between measurements and a given map.


**Planning**
Path planning is broken down into for sub-components: route planning, prediction, behavioral planning, and trajectory planning.

- **Route Planning**
    - The route planning component is responsible for high-level decisions about the path of the vehicle between two points on a map; for example which roads, highways, or freeways to take. This component is similar to the route planning feature found on many smartphones or modern car navigation systems.
    
- **Prediction**
    - The prediction component estimates what actions other objects might take in the future. For example, if another vehicle were identified, the prediction component would estimate its future trajectory.

- **Behavioral Planning**
    - The behavioral planning component determines what behavior the vehicle should exhibit at any point in time. For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component.

- **Trajectory Planning**
    - Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.

**Control**
The control component takes trajectory outputs and processes them with a controller algorithm like PID or MPC to adjust the control inputs for smooth operation of the vehicle.

### ROS Architechture
The ROS Architecture consists of different nodes (written in Python or C++) that communicate with each other via ROS messages. 

<img src="https://github.com/ajimenezjulio/P9_Programming_A_Real_Self_Driving_Car/blob/master/imgs/ros_architechture.png">
</p>

**Node Design**
Some of the crucial nodes developed in this project are waypoint updater(`waypoint_updater.py`), traffic light detector (`tl_detector.py`) and the drive by wire node (`dbw_node.py`).

**Waypoint Updater**
The waypoint updater node takes a central role in the planning task because it determines which waypoints the car should follow. Some functions of this node are:
- `init-function`: Defines the attributes of the class and determines which topics the class subscribes to and which ones it publishes on. 
- `callback functions`: They are invoked repeatedly by the subscribers in the init-function. Repeatedly called are the base waypoints (output of waypoint loader), car's pose (simulator / car) and traffic waypoint (output of tl_detector).
- `decelerate_waypoints-function` which incorporates a square-root shaped deceleration towards a predetermined stopline location in case of red traffic lights. 

**Traffic Light Detection**
The traffic light detector is responsible for processing raw images from the camera and detect traffic lights, its role is to publish the waypoint where a traffic light is detected along with the state of the respective light.
- Includes the subscriptions to the current position base waypoints, the given traffic light array with the ground-truth coordinates of the traffic lights, along with the identified color of the traffic light (for simulator). 
- The color of the traffic light is the output of the traffic light classifier, the ANN model is explained in the further section.
- The topic `image_color` gets updated by the callback `image_cb`, which itself calls via the `process_traffic_lights()` function, who in turn utilizes the function `get_light_state()` that receives the traffic light classification. 
- Eventually, the waypoint to stop at for any upcoming identified red traffic light is published in this subroutine.

**Drive-By-Wire (DBW) Node**
The third node written by us is the `dbw_node` which is responsible for driving the car. It subscribes to a twist controller which outputs throttle, brake and steering values with the help of a `PID-controller` and `Lowpass filter`. The dbw node directly publishes throttle, brake and steering commands for the car/simulator, in case `dbw_enabled` is set to `True`.


### ANN Model
**Model**
The traffic light classification model is based on the pre-trained on the COCO dataset model "faster_rcnn_resnet101_coco" from [Tensorflow detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md). Using the [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection), the simulator data model and real data model were trained. The models are available [here](https://drive.google.com/drive/folders/1I7Ns8ZYgLdNgZfrUlwc2_X-82oy_ASlb?usp=sharing).

**Dataset**
[Step-by-step Tensorflow Object Detection API tutorial](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e) was a good guide of using the Tensorflow object detection API for traffic light classification. The simulator dataset was from [here](https://drive.google.com/file/d/0Bw5abyXVejvMci03bFRueWVXX1U), and the real dataset was from [here](https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE).

**Classification**
The classification output has four categories: Red, Green, Yellow and off. To simplify, the final output will be Red or Non-Red, that is only the Red will be classified as `TrafficLight.RED`, and the other cases will be classified as `TrafficLight.GREEN`.

| <img src="https://github.com/ajimenezjulio/P9_Programming_A_Real_Self_Driving_Car/blob/master/imgs/Screen%20Shot%202020-10-08%20at%2010.39.58%20AM.png"> | <img src="https://github.com/ajimenezjulio/P9_Programming_A_Real_Self_Driving_Car/blob/master/imgs/Screen%20Shot%202020-10-08%20at%2010.40.08%20AM.png"> |
|---|---|
| <img src="https://github.com/ajimenezjulio/P9_Programming_A_Real_Self_Driving_Car/blob/master/imgs/Screen%20Shot%202020-10-08%20at%2010.40.24%20AM.png"> | <img src="https://github.com/ajimenezjulio/P9_Programming_A_Real_Self_Driving_Car/blob/master/imgs/Screen%20Shot%202020-10-08%20at%2010.40.41%20AM.png"> |
