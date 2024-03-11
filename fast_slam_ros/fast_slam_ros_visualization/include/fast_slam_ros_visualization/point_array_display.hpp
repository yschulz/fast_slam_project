#ifndef FAST_SLAM_ROS_VISUALIZATION__POINT_ARRAY_DISPLAY_HPP
#define FAST_SLAM_ROS_VISUALIZATION__POINT_ARRAY_DISPLAY_HPP

#include "rviz_common/message_filter_display.hpp"
#include <memory>
#include <vector>
#include <QtCore>
#include "fast_slam_ros_msgs/msg/point_array.hpp"


namespace Ogre
{
class SceneNode;
}

namespace rviz_rendering
{
class Shape;
}

namespace rviz_common
{
class DisplayContext;

namespace properties
{
class ColorProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace fast_slam_ros_visualization{
    
class PointArrayDisplay : public rviz_common::MessageFilterDisplay<fast_slam_ros_msgs::msg::PointArray>{
    Q_OBJECT

public:
    explicit PointArrayDisplay(rviz_common::DisplayContext * context);
    PointArrayDisplay();
    ~PointArrayDisplay() override;

    void processMessage(fast_slam_ros_msgs::msg::PointArray::ConstSharedPtr msg) override;

protected:
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    void updateColorAndAlpha();

private:
    void setUpProperties();
    void createNewSphereVisual(const geometry_msgs::msg::Point & msg);

    std::vector<std::shared_ptr<rviz_rendering::Shape>> visuals_;

    rviz_common::properties::ColorProperty * color_property_;
    rviz_common::properties::FloatProperty * alpha_property_;
    rviz_common::properties::FloatProperty * radius_property_;
};

}

#endif