#include <gtest/gtest.h>

#include "fast_slam/Particle.hpp"
#include "fast_slam/Measurement.hpp"
#include "fast_slam/MeasurementSet.hpp"
#include <eigen3/Eigen/Core>
#include <chrono>

class ParticleTester : public ::testing::Test {
protected:
    Eigen::Vector3d initial_pose = {1.0 , 2.0 , 3.0};
    std::unique_ptr<fastslam::Particle> particle = std::make_unique<fastslam::Particle>();

    virtual void SetUp(){

    }
};


TEST_F(ParticleTester, update_test){
    Eigen::Vector2d m_m_1 = {1.0, 1.0};
    Eigen::Vector2d m_m_2 = {-1.0, -1.0};
    Eigen::Vector2d m_m_3 = {1.0, -1.0};
    Eigen::Vector2d m_m_4 = {-1.0, 1.0};

    std::shared_ptr<fastslam::Measurement> m_1 = std::make_shared<fastslam::LandmarkXYMeasurement>(1, m_m_1);
    std::shared_ptr<fastslam::Measurement> m_2 = std::make_shared<fastslam::LandmarkXYMeasurement>(2, m_m_2);
    std::shared_ptr<fastslam::Measurement> m_3 = std::make_shared<fastslam::LandmarkXYMeasurement>(3, m_m_3);
    std::shared_ptr<fastslam::Measurement> m_4 = std::make_shared<fastslam::LandmarkXYMeasurement>(4, m_m_4);

    std::shared_ptr<fastslam::MeasurementSet> m_set = std::make_shared<fastslam::MeasurementSet>();

    m_set->addMeasurement(m_1);
    m_set->addMeasurement(m_2);
    m_set->addMeasurement(m_3);
    m_set->addMeasurement(m_4);

    Eigen::Vector3d u = {0.1, 0.1, 0.1};
    std::chrono::nanoseconds delta_time = std::chrono::nanoseconds(100);

    particle->updateParticle(m_set, &u, 1, delta_time, false);

    auto weight = particle->getWeight();
    auto pose = particle->getPose();
    auto map = particle->getMap();

    for(auto lm : map){
        std::cout << lm->landmark_pose.transpose() << "\n";
    }
}

TEST_F(ParticleTester, copy_test){
    std::unique_ptr<fastslam::Particle> particle_copy = std::make_unique<fastslam::Particle>(*particle);
}