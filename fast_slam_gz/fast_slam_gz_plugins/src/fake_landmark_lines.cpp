#include "fast_slam_gz_plugins/fake_landmark_lines.hpp"

#include <sstream>
#include <geometry_msgs/msg/point.hpp>
#include <fast_slam_ros_msgs/msg/line_landmark.hpp>

GZ_ADD_PLUGIN(FakeLandmarkLines, gz::sim::System, FakeLandmarkLines::ISystemConfigure, FakeLandmarkLines::ISystemPostUpdate)

struct GzLine{
    double distance;
    double heading;
};

double angleMod(double angle){
    auto x = fmod(angle + M_PI, 2 * M_PI);
    if (x < 0) x += 2 * M_PI;
    return x - M_PI;
}

class FakeLandmarkLinesPrivate{
public:
    gz::sim::Model model{gz::sim::kNullEntity};

    rclcpp::Node::SharedPtr ros_node_ptr;
    rclcpp::Publisher<fast_slam_ros_msgs::msg::LineLandmarkArray>::SharedPtr ros_landmark_pub;

    std::vector<gz::sim::Entity> landmark_entities;
    std::vector<GzLine> all_landmark_lines;

    double threshold;

    std::string ros_frame;

    void getLandmarkMeasurements(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm);
};

void FakeLandmarkLinesPrivate::getLandmarkMeasurements(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm){
    gz::math::Pose3d gt_pose = gz::sim::worldPose(model.Entity(), _ecm);

    fast_slam_ros_msgs::msg::LineLandmarkArray msg;
    
    msg.header.frame_id = ros_frame;
    msg.header.stamp = rclcpp::Time(_info.simTime.count());

    for(const auto &landmark_line :  all_landmark_lines){
        fast_slam_ros_msgs::msg::LineLandmark line;

        line.heading = angleMod(landmark_line.heading - gt_pose.Rot().Yaw());
        line.distance = landmark_line.distance + std::cos(landmark_line.heading) * gt_pose.Pos().X() - std::sin(landmark_line.heading) * gt_pose.Pos().Y();


        msg.line_landmarks.push_back(line);
    }

    ros_landmark_pub->publish(msg);
}

FakeLandmarkLines::FakeLandmarkLines(): data_ptr(std::make_unique<FakeLandmarkLinesPrivate>()){}

void FakeLandmarkLines::Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &/*_event_mgr*/){

    GZ_PROFILE("FakeLandmarkLines::Configure");

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

        double distance = std::hypot(landmark_pose.Pos().X(), landmark_pose.Pos().Y());
        double heading = std::atan2(landmark_pose.Pos().Y(), landmark_pose.Pos().X());

        heading = angleMod(heading + 0.5*M_PI);

        GzLine line;
        line.distance = distance;
        line.heading = heading;
        data_ptr->all_landmark_lines.push_back(line);
    }

    std::stringstream node_name_stream;
    node_name_stream << "gz_fake_landmark_" << landmark_identifier;

    data_ptr->ros_node_ptr = rclcpp::Node::make_shared(node_name_stream.str());
    data_ptr->ros_landmark_pub = data_ptr->ros_node_ptr->create_publisher<fast_slam_ros_msgs::msg::LineLandmarkArray>(pose_array_topic, 1);

}

void FakeLandmarkLines::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm){
    GZ_PROFILE("FakeLandmarkLines::PostUpdate");
    data_ptr->getLandmarkMeasurements(_info, _ecm);
}