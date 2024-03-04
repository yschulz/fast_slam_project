#include <gtest/gtest.h>

#include "fast_slam/Measurement.hpp"
#include "fast_slam/MeasurementSet.hpp"
#include <eigen3/Eigen/Core>
#include <chrono>

class MeasurementTester : public ::testing::Test {
protected:

    Eigen::Vector3d initial_pose = {1.0 , 2.0 , 3.0};
    std::unique_ptr<fastslam::MeasurementSet> m_set = std::make_unique<fastslam::MeasurementSet>();

    virtual void SetUp(){
        Eigen::Vector3d measurement_3d;
        Eigen::Vector2d measurement_2d;
        measurement_3d << 1.0, 2.0, 0.0;
        measurement_2d << 1.0, 2.0;

        std::shared_ptr<fastslam::Measurement> got_measurement = std::make_shared<fastslam::GOTMeasurement>(1, measurement_3d);
        std::shared_ptr<fastslam::Measurement> xy_measurement = std::make_shared<fastslam::LandmarkXYMeasurement>(2, measurement_2d);
        std::shared_ptr<fastslam::Measurement> xy_yaw_measurement = std::make_shared<fastslam::LandmarkXYYawMeasurement>(3, measurement_3d);

        m_set->addMeasurement(got_measurement);
        m_set->addMeasurement(xy_measurement);
        m_set->addMeasurement(xy_yaw_measurement);
    }
};

TEST_F(MeasurementTester, getMeasurement_test){
    auto m_test = m_set->getMeasurement(1)->getMeasurement();

    EXPECT_DOUBLE_EQ(m_test(0), 1.0);
    EXPECT_DOUBLE_EQ(m_test(1), 2.0);
    EXPECT_DOUBLE_EQ(m_test(2), 0.0);
}

TEST_F(MeasurementTester, nMeasurements_test){
    auto n_measurements = m_set->getNumberOfMeasurements();

    EXPECT_EQ(n_measurements, 3);
}

TEST_F(MeasurementTester, GOTMeasurement_test){
    auto got_m = m_set->getMeasurement(1);

    Eigen::Vector3d pose;
    pose << 1.0, 2.0, 3.0;
}

TEST_F(MeasurementTester, XYMeasurement_test){
    auto xy_m = m_set->getMeasurement(2);
    auto measurement = xy_m->getMeasurement();

    Eigen::Vector2d landmark;
    Eigen::Vector3d pose;
    pose << 1.0, 2.0, M_PI / 2;
    landmark << -1.0, 3.0;

    auto inverse_m = xy_m->inverseMeasurementModel(pose);
    auto measurement_m = xy_m->MeasurementModel(pose, landmark);


    EXPECT_DOUBLE_EQ(landmark(0), inverse_m(0));
    EXPECT_DOUBLE_EQ(landmark(1), inverse_m(1));

    EXPECT_DOUBLE_EQ(measurement(0), measurement_m(0));
    EXPECT_DOUBLE_EQ(measurement(1), measurement_m(1));

    auto Hl = xy_m->calculateHl(pose, landmark);
}

TEST_F(MeasurementTester, XYYawMeasurement_test){
    auto xy_yaw_m = m_set->getMeasurement(3);
    auto measurement = xy_yaw_m->getMeasurement();

    Eigen::Vector3d landmark;
    Eigen::Vector3d pose;
    pose << 1.0, 2.0, M_PI / 2;
    landmark << -1.0, 3.0, M_PI / 2;

    auto inverse_m = xy_yaw_m->inverseMeasurementModel(pose);
    auto measurement_m = xy_yaw_m->MeasurementModel(pose, landmark);


    EXPECT_DOUBLE_EQ(landmark(0), inverse_m(0));
    EXPECT_DOUBLE_EQ(landmark(1), inverse_m(1));
    EXPECT_DOUBLE_EQ(landmark(2), inverse_m(2));

    EXPECT_DOUBLE_EQ(measurement(0), measurement_m(0));
    EXPECT_DOUBLE_EQ(measurement(1), measurement_m(1));
    EXPECT_DOUBLE_EQ(measurement(2), measurement_m(2));
}