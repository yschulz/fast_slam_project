#ifndef FAST_SLAM_ROS_VISUALIZATION__LINE_LANDMARK_DISPLAY_HPP
#define FAST_SLAM_ROS_VISUALIZATION__LINE_LANDMARK_DISPLAY_HPP

#include <fast_slam_ros_msgs/msg/line_landmark_array.hpp>
#include "rviz_common/message_filter_display.hpp"

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <OgreQuaternion.h>

namespace Ogre
{
class ManualObject;
}  // namespace Ogre

namespace rviz_common
{
namespace properties
{
class EnumProperty;
class ColorProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_rendering
{
class Arrow;
class Axes;
}  // namespace rviz_rendering

namespace fast_slam_ros_visualization{
    
struct OgrePose{
    double distance;
    Ogre::Quaternion orientation;
};

class FlatArrow{
public:
    explicit FlatArrow(Ogre::SceneManager * scene_manager);
    ~FlatArrow();

    void createAndAttachManualObject(Ogre::SceneNode * scene_node);
    void updateManualObject(Ogre::ColourValue color, float alpha, float length, const OgrePose & pose);
    void clear();

private:
    void setManualObjectMaterial();
    void setManualObjectVertices(const Ogre::ColourValue & color, float length, const OgrePose & pose);

    Ogre::SceneManager * scene_manager_;
    Ogre::ManualObject * manual_object_;
    Ogre::MaterialPtr material_;
};

class Line{
public:
    explicit Line(Ogre::SceneManager * scene_manager);
    ~Line();

    void createAndAttachManualObject(Ogre::SceneNode * scene_node);
    void updateManualObject(Ogre::ColourValue color, float alpha, float length, const OgrePose & pose);
    void clear();

private:
    void setManualObjectMaterial();
    void setManualObjectVertices(const Ogre::ColourValue & color, float length, const OgrePose & pose);

    Ogre::SceneManager * scene_manager_;
    Ogre::ManualObject * manual_object_;
    Ogre::MaterialPtr material_;
};

class LineLandmarkDisplay : public rviz_common::MessageFilterDisplay<fast_slam_ros_msgs::msg::LineLandmarkArray>{
    Q_OBJECT

public:
    LineLandmarkDisplay(rviz_common::DisplayContext * display_context, Ogre::SceneNode * scene_node);
    LineLandmarkDisplay();
    ~LineLandmarkDisplay() override;

    void processMessage(fast_slam_ros_msgs::msg::LineLandmarkArray::ConstSharedPtr msg) override;

protected:
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    void updateArrowColor();
    void updateArrow2dGeometry();

private:
    void initializeProperties();
    void updateArrows2d();

    std::vector<OgrePose> poses_;
    std::vector<std::unique_ptr<FlatArrow>> arrows_;
    std::vector<std::unique_ptr<Line>> lines_;

    Ogre::SceneNode * arrow_node_;
    Ogre::SceneNode * axes_node_;

    rviz_common::properties::ColorProperty * arrow_color_property_;
    rviz_common::properties::FloatProperty * arrow_alpha_property_;
    rviz_common::properties::FloatProperty * arrow_tip_length_property_;
    rviz_common::properties::FloatProperty * line_length_property_;
};

}

#endif