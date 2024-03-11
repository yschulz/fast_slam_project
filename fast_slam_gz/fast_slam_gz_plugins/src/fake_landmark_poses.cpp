#include "fast_slam_gz_plugins/fake_landmark_poses.hpp"

#include <sstream>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

GZ_ADD_PLUGIN(FakeLandmarkPoses, gz::sim::System, FakeLandmarkPoses::ISystemConfigure, FakeLandmarkPoses::ISystemPostUpdate)

class FakeLandmarkPosesPrivate{
public:
    gz::sim::Model model{gz::sim::kNullEntity};

    rclcpp::Node::SharedPtr ros_node_ptr;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ros_landmark_pub;

    std::vector<gz::sim::Entity> landmark_entities;
    std::vector<gz::math::Pose3d> all_landmark_poses;

    double threshold;

    std::string ros_frame;

    void getLandmarkMeasurements(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm);
};

void FakeLandmarkPosesPrivate::getLandmarkMeasurements(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm){
    gz::math::Pose3d gt_pose = gz::sim::worldPose(model.Entity(), _ecm);

    geometry_msgs::msg::PoseArray msg;
    
    msg.header.frame_id = ros_frame;
    msg.header.stamp = rclcpp::Time(_info.simTime.count());

    for(const auto &landmark_pose :  all_landmark_poses){
        geometry_msgs::msg::Pose pose;

        double dx = landmark_pose.Pos().X() - gt_pose.Pos().X();
        double dy = landmark_pose.Pos().Y() - gt_pose.Pos().Y();

        auto  distance = std::hypot(dx, dy);

        if(distance < threshold){
            double yaw = gt_pose.Rot().Yaw();
            auto rot_diff = gt_pose.CoordRotationSub(landmark_pose.Rot());
            pose.position.x = std::cos(yaw) * dx + std::sin(yaw) * dy;
            pose.position.y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

            pose.orientation.z = rot_diff.Z();
            pose.orientation.w = rot_diff.W();

            msg.poses.push_back(pose);
        }
    }

    ros_landmark_pub->publish(msg);
}

FakeLandmarkPoses::FakeLandmarkPoses(): data_ptr(std::make_unique<FakeLandmarkPosesPrivate>()){}

void FakeLandmarkPoses::Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &/*_event_mgr*/){

    GZ_PROFILE("FakeLandmarkPoses::Configure");

    data_ptr->model = gz::sim::Model(_entity);

    // init if ros is not up
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    std::string pose_array_topic;
    if (!_sdf->HasElement("landmark_topic")){
        gzmsg << "SDF missing tag <landmark_topic>, initializing to default landmark_topic topic";
        pose_array_topic = "landmark_topic";
    } else {
        pose_array_topic = _sdf->Get<std::string>("landmark_topic");
    }

    if (!_sdf->HasElement("ros_frame")){
        gzmsg << "SDF missing tag <ros_frame>, initializing to default base_footprint";
        data_ptr->ros_frame = "base_footprint";
    } else {
        data_ptr->ros_frame = _sdf->Get<std::string>("ros_frame");
    }

    if (!_sdf->HasElement("threshold")){
        gzmsg << "SDF missing tag <threshold>, initializing to default 10";
        data_ptr->threshold = 10.;
    } else {
        data_ptr->threshold = _sdf->Get<double>("threshold");
    }

    std::string landmark_identifier;
    if (!_sdf->HasElement("landmark_identifier")){
        gzmsg << "SDF missing tag <landmark_identifier>, initializing to default landmark";
        landmark_identifier = "landmark";
    } else {
        landmark_identifier = _sdf->Get<std::string>("landmark_identifier");
    }


    bool found_first_landmark = false;
    for(size_t i=0; i< std::numeric_limits<size_t>::max(); i++){
        std::stringstream ss;
        ss << landmark_identifier << "_" << i << "_link";

        auto landmark_entity = _ecm.EntityByComponents(gz::sim::components::Name(ss.str()));
        if(!landmark_entity && !found_first_landmark)
            continue;
        else if(!landmark_entity && found_first_landmark)
            break;
        else if(landmark_entity && !found_first_landmark)
            found_first_landmark = true;

        gz::math::Pose3d landmark_pose = gz::sim::worldPose(landmark_entity, _ecm);
        data_ptr->all_landmark_poses.push_back(landmark_pose);
    }

    std::stringstream node_name_stream;
    node_name_stream << "gz_fake_landmark_" << landmark_identifier;

    data_ptr->ros_node_ptr = rclcpp::Node::make_shared(node_name_stream.str());
    data_ptr->ros_landmark_pub = data_ptr->ros_node_ptr->create_publisher<geometry_msgs::msg::PoseArray>(pose_array_topic, 1);

}

void FakeLandmarkPoses::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm){
    GZ_PROFILE("FakeLandmarkPoses::PostUpdate");
    data_ptr->getLandmarkMeasurements(_info, _ecm);
}