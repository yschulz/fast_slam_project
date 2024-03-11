#include "fast_slam_ros_visualization/line_landmark_display.hpp"


#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreSceneManager.h>

#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/validate_floats.hpp"

#include "rviz_rendering/material_manager.hpp"



namespace fast_slam_ros_visualization{

FlatArrow::FlatArrow(Ogre::SceneManager * scene_manager):
    scene_manager_(scene_manager), 
    manual_object_(nullptr) {}

FlatArrow::~FlatArrow(){
    if (manual_object_) {
        scene_manager_->destroyManualObject(manual_object_);
    }
}

void FlatArrow::createAndAttachManualObject(Ogre::SceneNode * scene_node){
    manual_object_ = scene_manager_->createManualObject();
    manual_object_->setDynamic(true);
    scene_node->attachObject(manual_object_);
}

void FlatArrow::updateManualObject(Ogre::ColourValue color, float alpha, float length, const OgrePose & pose){
    clear();

    color.a = alpha;
    rviz_rendering::MaterialManager::enableAlphaBlending(material_, alpha);

    manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");
    setManualObjectVertices(color, length, pose);
    manual_object_->end();
}
void FlatArrow::clear(){
    if (manual_object_) {
        manual_object_->clear();
    }
}

void FlatArrow::setManualObjectMaterial(Ogre::MaterialPtr material){
    material_ = material;
}

void FlatArrow::setManualObjectVertices(const Ogre::ColourValue & color, float length, const OgrePose & pose){
    manual_object_->estimateVertexCount(6);

    Ogre::Vector3 vertices[6];
    vertices[0] = Ogre::Vector3(0);  // back of arrow
    vertices[1] = pose.orientation * Ogre::Vector3(pose.distance, 0, 0);  // tip of arrow
    vertices[2] = vertices[1];
    vertices[3] = pose.orientation * Ogre::Vector3(pose.distance - length, length, 0);
    vertices[4] = vertices[1];
    vertices[5] = pose.orientation * Ogre::Vector3(pose.distance - length, -length, 0);

    for (const auto & vertex : vertices) {
        manual_object_->position(vertex);
        manual_object_->colour(color);
    }
}













Line::Line(Ogre::SceneManager * scene_manager):
    scene_manager_(scene_manager), 
    manual_object_(nullptr) {}

Line::~Line(){
    if (manual_object_) {
        scene_manager_->destroyManualObject(manual_object_);
    }
}

void Line::createAndAttachManualObject(Ogre::SceneNode * scene_node){
    manual_object_ = scene_manager_->createManualObject();
    manual_object_->setDynamic(true);
    scene_node->attachObject(manual_object_);
}

void Line::updateManualObject(Ogre::ColourValue color, float alpha, float length, const OgrePose & pose){
    clear();

    color.a = alpha;
    rviz_rendering::MaterialManager::enableAlphaBlending(material_, alpha);

    manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");
    setManualObjectVertices(color, length, pose);
    manual_object_->end();
}

void Line::clear(){
    if (manual_object_) {
        manual_object_->clear();
    }
}

void Line::setManualObjectMaterial(Ogre::MaterialPtr material){
    material_ = material;
}

void Line::setManualObjectVertices(const Ogre::ColourValue & color, float length, const OgrePose & pose){
    manual_object_->estimateVertexCount(4);

    Ogre::Vector3 vertices[4];
    vertices[0] = pose.orientation * Ogre::Vector3(pose.distance, 0, 0);  // tip of arrow
    vertices[1] = pose.orientation * Ogre::Vector3(pose.distance, length, 0);
    vertices[2] = vertices[0];
    vertices[3] = pose.orientation * Ogre::Vector3(pose.distance, -length, 0);

    for (const auto & vertex : vertices) {
        manual_object_->position(vertex);
        manual_object_->colour(color);
    }
}















LineLandmarkDisplay::LineLandmarkDisplay(rviz_common::DisplayContext * display_context, Ogre::SceneNode * scene_node):
    LineLandmarkDisplay(){
    context_ = display_context;
    scene_node_ = scene_node;
    scene_manager_ = context_->getSceneManager();
}

LineLandmarkDisplay::LineLandmarkDisplay(){
    initializeProperties();

    arrow_alpha_property_->setMin(0);
    arrow_alpha_property_->setMax(1);

    static int material_count = 0;
    std::string material_name = "Material" + std::to_string(material_count++);
    material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
}

LineLandmarkDisplay::~LineLandmarkDisplay() = default;

void LineLandmarkDisplay::processMessage(fast_slam_ros_msgs::msg::LineLandmarkArray::ConstSharedPtr msg){

    rclcpp::Time time_stamp(msg->header.stamp, RCL_ROS_TIME);
    if (!updateFrame(msg->header.frame_id, time_stamp)) {
        setMissingTransformToFixedFrame(msg->header.frame_id);
        return;
    }
    setTransformOk();

    poses_.resize(msg->line_landmarks.size());

    for (std::size_t i = 0; i < msg->line_landmarks.size(); ++i){
        if (!rviz_common::validateFloats(msg->line_landmarks[i].heading) || !rviz_common::validateFloats(msg->line_landmarks[i].distance)) {
            setStatus(
                rviz_common::properties::StatusProperty::Error,
                "Topic",
                "Message contained invalid floating point values (nans or infs)");
            return;
        }

        poses_[i].distance = msg->line_landmarks[i].distance;
        poses_[i].orientation = Ogre::Quaternion(Ogre::Radian(msg->line_landmarks[i].heading), Ogre::Vector3(0,0,1));
    }

    updateArrows2d();

    context_->queueRender();
}


void LineLandmarkDisplay::onInitialize(){
    MFDClass::onInitialize();
}

void LineLandmarkDisplay::reset(){
    MFDClass::reset();
}

void LineLandmarkDisplay::updateArrowColor(){
    updateArrows2d();
    context_->queueRender();
}

void LineLandmarkDisplay::initializeProperties(){
    arrow_color_property_ = new rviz_common::properties::ColorProperty(
        "Color", QColor(255, 25, 0), "Color to draw the arrows.", this, SLOT(updateArrowColor()));

    arrow_alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha",
        1.0f,
        "Amount of transparency to apply to the displayed poses.",
        this,
        SLOT(updateArrowColor()));

    line_length_property_ = new rviz_common::properties::FloatProperty(
        "Arrow Length", 2.0f, "Length of the arrows.", this, SLOT(updateArrow2dGeometry()));

    arrow_tip_length_property_ = new rviz_common::properties::FloatProperty(
        "Arrow tip length", 0.2f, "Length of the arrows.", this, SLOT(updateArrow2dGeometry()));

}
void LineLandmarkDisplay::updateArrow2dGeometry(){
    updateArrows2d();
    context_->queueRender();
}

void LineLandmarkDisplay::updateArrows2d(){
    while (arrows_.size() < poses_.size()) {
        std::unique_ptr<FlatArrow> new_arrow = std::make_unique<FlatArrow>(scene_manager_);
        new_arrow->createAndAttachManualObject(scene_node_);

        arrows_.push_back(std::move(new_arrow));
    }
    while (arrows_.size() > poses_.size()) {
        arrows_.pop_back();
    }

    while (lines_.size() < poses_.size()) {
        std::unique_ptr<Line> new_line = std::make_unique<Line>(scene_manager_);
        new_line->createAndAttachManualObject(scene_node_);

        lines_.push_back(std::move(new_line));
    }
    while (lines_.size() > poses_.size()) {
        lines_.pop_back();
    }

    for(size_t i = 0; i < poses_.size(); ++i){
        arrows_[i]->setManualObjectMaterial(material_);
        arrows_[i]->updateManualObject(
            arrow_color_property_->getOgreColor(),
            arrow_alpha_property_->getFloat(),
            arrow_tip_length_property_->getFloat(),
            poses_[i]);

        lines_[i]->setManualObjectMaterial(material_);
        lines_[i]->updateManualObject(
            arrow_color_property_->getOgreColor(),
            arrow_alpha_property_->getFloat(),
            line_length_property_->getFloat(),
            poses_[i]);
    }
}

}
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(fast_slam_ros_visualization::LineLandmarkDisplay, rviz_common::Display)