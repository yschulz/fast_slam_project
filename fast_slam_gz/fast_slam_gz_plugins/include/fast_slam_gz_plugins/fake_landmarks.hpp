#ifndef GZ_FAKE_LANDMARKS_HPP
#define GZ_FAKE_LANDMARKS_HPP

#include <gz/plugin/Register.hh>
#include <gz/common/Profiler.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class FakeLandmarksPrivate;

class FakeLandmarks : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate{
public:
    FakeLandmarks();
    ~FakeLandmarks() override = default;

    void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_event_mgr) override;

    void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override;

private:
    std::shared_ptr<FakeLandmarksPrivate> data_ptr;
};

#endif