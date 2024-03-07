#include "fast_slam/Measurement.hpp"

namespace fastslam{

GOTMeasurement::GOTMeasurement(uint32_t identifier, Eigen::VectorXd GOT_meas):
    Measurement(){
    c_id_ = identifier;
    measurement_ = GOT_meas;
}

Eigen::VectorXd GOTMeasurement::MeasurementModel(StateVector pose, Eigen::VectorXd landmark){
    return pose - landmark;
}

Eigen::VectorXd GOTMeasurement::inverseMeasurementModel(StateVector pose){
    return pose - measurement_;
}

Eigen::MatrixXd GOTMeasurement::calculateHs(StateVector pose, Eigen::VectorXd landmark){
    return Eigen::Matrix3d::Identity();
}

Eigen::MatrixXd GOTMeasurement::calculateHl(StateVector pose, Eigen::VectorXd landmark){
    return -1.0*Eigen::Matrix3d::Identity();
};

Eigen::MatrixXd GOTMeasurement::measurement_covariance_ = 0.05*Eigen::Matrix3d::Identity(); // static variable - has to be declared outside class!



LandmarkXYMeasurement::LandmarkXYMeasurement(uint32_t identifier, Eigen::VectorXd xy_measurement):
    Measurement(){
    c_id_ = identifier;
    measurement_ = xy_measurement;
}

Eigen::VectorXd LandmarkXYMeasurement::MeasurementModel(StateVector pose, Eigen::VectorXd landmark){
    Eigen::Matrix2d T;
    Eigen::Vector2d d;

    d << landmark(0) - pose(0), landmark(1) - pose(1); 

    auto cos_p = std::cos(pose(2));
    auto sin_p = std::sin(pose(2));

    T  <<   cos_p,  sin_p,
            -sin_p,  cos_p;

    return T * d;
}

Eigen::VectorXd LandmarkXYMeasurement::inverseMeasurementModel(StateVector pose){
    Eigen::Matrix2d T;
    Eigen::Vector2d d;

    d << pose(0), pose(1);

    auto cos_p = std::cos(pose(2));
    auto sin_p = std::sin(pose(2));

    T  <<   cos_p,  -sin_p,
            sin_p,  cos_p;

    return T * measurement_ + d;    
}


Eigen::MatrixXd LandmarkXYMeasurement::calculateHs(StateVector pose, Eigen::VectorXd landmark){
    Eigen::MatrixXd Hs(2, 3);

    auto dx = landmark(0) - pose(0);
    auto dy = landmark(1) - pose(1); 
    auto cos_p = std::cos(pose(2));
    auto sin_p = std::sin(pose(2));

    Hs <<   -cos_p, -sin_p, -dx*sin_p + dy*cos_p,
            sin_p,  -cos_p, -dx*cos_p - dy*sin_p;

    return Hs;
}

Eigen::MatrixXd LandmarkXYMeasurement::calculateHl(StateVector pose, Eigen::VectorXd landmark){
    Eigen::MatrixXd Hl(2,2);

    auto cos_p = std::cos(pose(2));
    auto sin_p = std::sin(pose(2));

    Hl <<   cos_p,  sin_p,
            -sin_p, cos_p;

    return Hl;
}

Eigen::MatrixXd LandmarkXYMeasurement::measurement_covariance_ = 0.0005*Eigen::Matrix2d::Identity(); // static variable - has to be declared outside class!



LandmarkXYYawMeasurement::LandmarkXYYawMeasurement(uint32_t identifier, Eigen::VectorXd xy_yaw_measurement):
    Measurement(){
    c_id_ = identifier;
    measurement_ = xy_yaw_measurement;
}

Eigen::VectorXd LandmarkXYYawMeasurement::MeasurementModel(StateVector pose, Eigen::VectorXd landmark){
    Eigen::Matrix3d T;
    Eigen::Vector3d d;

    d << landmark(0) - pose(0), landmark(1) - pose(1), landmark(2) - pose(2); 

    auto cos_p = std::cos(pose(2));
    auto sin_p = std::sin(pose(2));

    T  <<   cos_p,  sin_p,  0,
            -sin_p, cos_p,  0,
            0,      0,      1;

    return T * d;
}

Eigen::VectorXd LandmarkXYYawMeasurement::inverseMeasurementModel(StateVector pose){
    Eigen::Matrix3d T;

    auto cos_p = std::cos(pose(2));
    auto sin_p = std::sin(pose(2));

    T  <<   cos_p,  -sin_p, 0,
            sin_p,  cos_p,  0,
            0,      0,      1;

    return T * measurement_ + pose;
}


Eigen::MatrixXd LandmarkXYYawMeasurement::calculateHs(StateVector pose, Eigen::VectorXd landmark){
    Eigen::MatrixXd Hs(3, 3);

    auto dx = landmark(0) - pose(0);
    auto dy = landmark(1) - pose(1); 
    auto cos_p = std::cos(pose(2));
    auto sin_p = std::sin(pose(2));

    Hs <<   -cos_p, -sin_p, -dx*sin_p + dy*cos_p,
            sin_p,  -cos_p, -dx*cos_p - dy*sin_p,
            0,      0,      -1;

    return Hs;
}

Eigen::MatrixXd LandmarkXYYawMeasurement::calculateHl(StateVector pose, Eigen::VectorXd landmark){
    Eigen::MatrixXd Hl(3,3);

    auto cos_p = std::cos(pose(2));
    auto sin_p = std::sin(pose(2));

    Hl <<   cos_p,  sin_p,  0,
            -sin_p, cos_p,  0,
            0,      0,      1;

    return Hl;
}

Eigen::MatrixXd LandmarkXYYawMeasurement::measurement_covariance_ = 0.05*Eigen::Matrix3d::Identity(); // static variable - has to be declared outside class!


} //namespace fastslam
