#include "fast_slam_ros_visualization/point_array_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_rendering/objects/shape.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/logging.hpp"

namespace fast_slam_ros_visualization{
    
PointArrayDisplay::PointArrayDisplay(rviz_common::DisplayContext * context){
    context_ = context;
    scene_manager_ = context_->getSceneManager();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    setUpProperties();
}

PointArrayDisplay::PointArrayDisplay(){
    setUpProperties();
}

PointArrayDisplay::~PointArrayDisplay() = default;


void PointArrayDisplay::processMessage(fast_slam_ros_msgs::msg::PointArray::ConstSharedPtr msg){

    rclcpp::Time time_stamp(msg->header.stamp, RCL_ROS_TIME);
    if (!updateFrame(msg->header.frame_id, time_stamp)) {
        setMissingTransformToFixedFrame(msg->header.frame_id);
        return;
    }
    setTransformOk();

    visuals_.clear();

    for(const auto &point : msg->points){
        if (!rviz_common::validateFloats(point)) {
            setStatus(
            rviz_common::properties::StatusProperty::Error, "Topic",
            "Message contained invalid floating point values (nans or infs)");
            return;
        }
        createNewSphereVisual(point);
    }
}

void PointArrayDisplay::onInitialize(){
    MFDClass::onInitialize();
}

void PointArrayDisplay::reset(){
    MFDClass::reset();
    visuals_.clear();
}

void PointArrayDisplay::updateColorAndAlpha(){
    float alpha = alpha_property_->getFloat();
    float radius = radius_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();

    for (auto visual : visuals_) {
        visual->setColor(color.r, color.g, color.b, alpha);
        visual->setScale(Ogre::Vector3(radius, radius, radius));
    }
}

void PointArrayDisplay::setUpProperties(){
    color_property_ = new rviz_common::properties::ColorProperty(
        "Color", QColor(204, 41, 204), "Color of a point", this, SLOT(updateColorAndAlpha()));

    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 1.0f, "0 is fully transparent, 1.0 is fully opaque.",
        this, SLOT(updateColorAndAlpha()));

    radius_property_ = new rviz_common::properties::FloatProperty(
        "Radius", 0.2f, "Radius of a point", this, SLOT(updateColorAndAlpha()));
}

void PointArrayDisplay::createNewSphereVisual(const geometry_msgs::msg::Point & msg){
    std::shared_ptr<rviz_rendering::Shape> visual = std::make_shared<rviz_rendering::Shape>(
        rviz_rendering::Shape::Sphere, context_->getSceneManager(), scene_node_);

    float alpha = alpha_property_->getFloat();
    float radius = radius_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();
    visual->setColor(color.r, color.g, color.b, alpha);
    visual->setPosition(rviz_common::pointMsgToOgre(msg));
    visual->setScale(Ogre::Vector3(radius, radius, radius));

    visuals_.push_back(visual);
}

}

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(fast_slam_ros_visualization::PointArrayDisplay, rviz_common::Display)