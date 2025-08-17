#include "fast_slam_ros_core/fast_slam_ros.hpp"

FastSlamRos::FastSlamRos(): 
            rclcpp::Node("fast_slam_ros"),
            p_set_(50, Eigen::Vector3d::Zero(), 0.01 * Eigen::Matrix3d::Identity())
            {

    std::cout << "setting up  \n";
    odom_frame_ = "odom";
    robot_frame_ = "base_footprint";
    map_frame_ = "map";

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(rclcpp::Node::get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    particle_pub_ = rclcpp::Node::create_publisher<geometry_msgs::msg::PoseArray>("particles", 1);
    landmark_pub_ = rclcpp::Node::create_publisher<fast_slam_ros_msgs::msg::PointArray>("map", 1);
    path_pub_ = rclcpp::Node::create_publisher<nav_msgs::msg::Path>("path", 1);
    distribution_pub_ = rclcpp::Node::create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("distribution", 1);

    landmark_maesurement_sub_ = rclcpp::Node::create_subscription<fast_slam_ros_msgs::msg::PointArray>("/landmark_points", 10, std::bind(&FastSlamRos::landmarkCallback, this, std::placeholders::_1));
    odom_sub_ = rclcpp::Node::create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&FastSlamRos::odomCallback, this, std::placeholders::_1));

    timer_ = rclcpp::Node::create_wall_timer(500ms, std::bind(&FastSlamRos::timerCallback, this));
    path_.header.frame_id = map_frame_;
}

FastSlamRos::~FastSlamRos(){}

void FastSlamRos::broadCastTransform(){
    geometry_msgs::msg::TransformStamped transform_ro;

    bool success = false;
    while(!success){
        try{
            transform_ro = tf_buffer_->lookupTransform(odom_frame_, robot_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(5));
            success = true;
        }
        catch(tf2::LookupException &ex){

        }
    }
    
    tf2::Transform robot_in_odom(tf2::Quaternion(0, 0, transform_ro.transform.rotation.z, transform_ro.transform.rotation.w),
                                 tf2::Vector3(transform_ro.transform.translation.x, transform_ro.transform.translation.y, 0));

    auto fast_slam_pose = p_set_.getLatestPoseEstimate();
    tf2::Quaternion pose_q;
    pose_q.setRPY(0, 0, fast_slam_pose(2));

    tf2::Transform robot_in_map(pose_q, tf2::Vector3(fast_slam_pose(0), fast_slam_pose(1), 0));
    tf2::Transform odom_in_map = robot_in_map * robot_in_odom.inverse();

    geometry_msgs::msg::TransformStamped t_out;
    t_out.child_frame_id = odom_frame_;
    t_out.header.frame_id = map_frame_;
    t_out.header.stamp = rclcpp::Node::get_clock()->now();
    t_out.transform.translation.x = odom_in_map.getOrigin().getX();
    t_out.transform.translation.y = odom_in_map.getOrigin().getY();
    t_out.transform.rotation.z = odom_in_map.getRotation().getZ();
    t_out.transform.rotation.w = odom_in_map.getRotation().getW();
    if(success){
        tf_broadcaster_->sendTransform(t_out);
    }
}

void FastSlamRos::publishDistribution(){
    auto latest_pose = p_set_.getLatestPoseEstimate();
    auto latest_covariance = p_set_.getLatestCovarianceEstimate();
    geometry_msgs::msg::PoseWithCovarianceStamped distribution_msg;
    distribution_msg.header.frame_id = map_frame_;
    distribution_msg.header.stamp = rclcpp::Node::get_clock()->now();

    tf2::Quaternion pose_q;
    pose_q.setRPY(0, 0, latest_pose(2));
    
    distribution_msg.pose.pose.position.x = latest_pose(0);
    distribution_msg.pose.pose.position.y = latest_pose(1);
    distribution_msg.pose.pose.orientation.w = pose_q.getW();
    distribution_msg.pose.pose.orientation.z = pose_q.getZ();

    distribution_msg.pose.covariance[0] = latest_covariance(0,0);
    distribution_msg.pose.covariance[1] = latest_covariance(0,1);
    distribution_msg.pose.covariance[5] = latest_covariance(0,2);
    distribution_msg.pose.covariance[6] = latest_covariance(1,0);
    distribution_msg.pose.covariance[7] = latest_covariance(1,1);
    distribution_msg.pose.covariance[11] = latest_covariance(1,2);
    distribution_msg.pose.covariance[30] = latest_covariance(2,0);
    distribution_msg.pose.covariance[31] = latest_covariance(2,1);
    distribution_msg.pose.covariance[35] = latest_covariance(2,2);

    distribution_pub_->publish(distribution_msg);
}

void FastSlamRos::publishParticles(){
    auto particle_poses = p_set_.getAllParticlePoseEstimates();

    geometry_msgs::msg::PoseArray particles;
    particles.header.frame_id = "map";
    particles.header.stamp = rclcpp::Node::get_clock()->now();

    for(auto &pose : particle_poses){
        geometry_msgs::msg::Pose pose_msg;
        tf2::Quaternion particle_pose_q;
        particle_pose_q.setRPY(0,0,pose(2));
        pose_msg.position.x = pose(0);
        pose_msg.position.y = pose(1);
        pose_msg.orientation.w = particle_pose_q.getW();
        pose_msg.orientation.z = particle_pose_q.getZ();
        particles.poses.push_back(pose_msg);
    }

    particle_pub_->publish(particles);
}

void FastSlamRos::publishMap(){
    // get map of the best particle
    auto map_pair = p_set_.getBestParticle();

    fast_slam_ros_msgs::msg::PointArray map;
    map.header.stamp = rclcpp::Node::get_clock()->now();
    map.header.frame_id = "map";

    for(auto &landmark : map_pair.second){
        geometry_msgs::msg::Point pose_msg;
        pose_msg.x = landmark->landmark_pose(0);
        pose_msg.y = landmark->landmark_pose(1);

        map.points.push_back(pose_msg);
    }

    landmark_pub_->publish(map);
}

void FastSlamRos::publishPath(){
    auto latest = p_set_.getLatestPoseEstimate();

    path_.header.stamp = rclcpp::Node::get_clock()->now();

    geometry_msgs::msg::PoseStamped new_pose;
    tf2::Quaternion pose_q;
    pose_q.setRPY(0,0,latest(2));
    
    new_pose.header = path_.header;
    new_pose.pose.position.x = latest(0);
    new_pose.pose.position.y = latest(1);
    new_pose.pose.orientation.w = pose_q.getW();
    new_pose.pose.orientation.z = pose_q.getZ();

    path_.poses.push_back(new_pose);

    path_pub_->publish(path_);
}

double FastSlamRos::getYaw(tf2::Quaternion q){
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void FastSlamRos::timerCallback(){
    StateVectorDerivative u;

    rclcpp::Time at_time = latest_landmarks_.header.stamp;

    geometry_msgs::msg::TransformStamped transform;
    bool success = false;
    while(!success){
        try{
            transform = tf_buffer_->lookupTransform(odom_frame_, robot_frame_, at_time, rclcpp::Duration::from_seconds(5));
            success = true;
        }
        catch(tf2::LookupException &ex){

        }
    }

    if(initialized_){
        tf2::Quaternion q, q_prev, q_trans;
        q.setX(0.0);
        q.setY(0.0);
        q.setW(transform.transform.rotation.w);
        q.setZ(transform.transform.rotation.z);

        q_prev.setX(0.0);
        q_prev.setY(0.0);
        q_prev.setW(last_transform_.transform.rotation.w);
        q_prev.setZ(last_transform_.transform.rotation.z);
        q_trans = q * q_prev.inverse();
        double d_yaw = getYaw(q_trans);


        u << transform.transform.translation.x - last_transform_.transform.translation.x,
            transform.transform.translation.y - last_transform_.transform.translation.y,
            d_yaw;


        auto delta_time = std::chrono::nanoseconds((at_time - time_old_).nanoseconds());

        std::shared_ptr<fastslam::MeasurementSet> m_set = std::make_shared<fastslam::MeasurementSet>();

        uint32_t i = 1;
        for(auto &point : latest_landmarks_.points){
            Eigen::Vector2d lm_input;
            lm_input << point.x, point.y;
            std::shared_ptr<fastslam::Measurement> m_landmark = std::make_shared<fastslam::LandmarkXYMeasurement>(i, lm_input);
            m_set->addMeasurement(m_landmark);
            i++;
        }

        std::cout << "number of ros measurements:  " << latest_landmarks_.points.size() << "  number of added measuremnts:  " << m_set->getNumberOfMeasurements() << "\n";

        p_set_.updateParticleSet(m_set, u, delta_time);

        publishPath();
        publishMap();
        publishParticles();
        publishDistribution();

        broadCastTransform();
    }

    last_transform_ = transform;
    last_pose_ = current_pose_;
    initialized_ = true;
    time_old_ = at_time;
}

void FastSlamRos::landmarkCallback(const std::shared_ptr<fast_slam_ros_msgs::msg::PointArray> msg) {
    latest_landmarks_.header = msg->header;
    latest_landmarks_.points = msg->points;
}

void FastSlamRos::odomCallback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    current_pose_ = msg->pose.pose;
}