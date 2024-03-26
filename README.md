# Fast slam project

This repo is an implementation of the [fast slam v2.0 algorithm](http://robots.stanford.edu/papers/Montemerlo03a.pdf). The goal for this implementation is versatility over, but including efficiency. This was tested with ros humble and gazebo harmonic.

## Project layout

### [fast_slam]

This is where the algorithm lives

### [fast_slam_gz]
- **[fast_slam_gz_plugins]**  
Simple gazebo plugins that publish fake landmarks in the vicinity based on the obstacle locations.
    - `FakeLandmarkLines`
    - `FakeLandmarkPoses`
    - `FakeLandmarkPoints`


- **[fast_slam_gz_description]**  
A simple description of a differential drive robot and a simple world with arbitrary landmarks.

### [fast_slam_ros]
- **[fast_slam_ros_core]**  
The fast slam ros interface. It takes the fake landmarks updates the robot pose with a call to the `update` and broadcasts the drift correction of the diff drive odometry (map->odom transform).

- **[fast_slam_ros_msgs]**  
Additional message defintions for this project. This includes lines and arrays that are not covered by standard message packages.
    - `LineLandmark.msg`
    - `LineLandmarkArray.msg`
    - `LineLandmarkStamped.msg`
    - `PointArray.msg`

- **[fast_slam_ros_visualization]**  
This repo includes rviz plugins for the message descriptions.  
    - `LineLandmarkDisplay`
    - `PointArrayDisplay`


## Roadmap

In no particular order

- Add support of combined association with different types of measurements
- Keep kd tree alive for nn search, dynamically update when map tree is changing
- Maybe replace binary tree with kd tree directly, but for localization directly for sure. [see iterative dynamic kd trees](https://arxiv.org/pdf/2102.10808.pdf)
- Add support for direct localization without map building
- Add multithreading support for particle updates
- Add global localization by sampling in ROI
- Add proper motion model to the particle set
- Add propagation without update
- Add conditional update
- Add a dense version using scan matching [see iris lama scan-matching for efficient implementation](https://github.com/iris-ua/iris_lama)
- Make the ros interface dynamically switch between dense and sparse localization
