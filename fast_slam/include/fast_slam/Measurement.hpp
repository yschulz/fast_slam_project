#ifndef FASTSLAM_MEASUREMENT_H
#define FASTSLAM_MEASUREMENT_H

#include <eigen3/Eigen/Core>
#include <iostream>
#include "fast_slam/Helper.hpp"

namespace fastslam{

class Measurement
{
public:

    /* functions */
   Measurement(){}
   ~Measurement(){}

    inline void setC(uint32_t c_new){ c_id_ = c_new;}
    inline uint32_t getC() const {return c_id_;}
    inline Eigen::VectorXd getMeasurement() const {return measurement_;}

    virtual Eigen::MatrixXd calculateHl(StateVector pose, Eigen::VectorXd landmark) = 0;		/* calculates derivative of measurement model with respect to landmark variable - l */
    virtual Eigen::MatrixXd calculateHs(StateVector pose, Eigen::VectorXd landmark) = 0;		/* calculates derivative of measurement model with respect to pose variable - s */
    virtual Eigen::VectorXd inverseMeasurementModel(StateVector pose) = 0;
    virtual Eigen::VectorXd MeasurementModel(StateVector pose, Eigen::VectorXd landmark) = 0;
    virtual inline  Eigen::MatrixXd getzCov() const = 0;
protected:
    uint32_t c_id_; 	/* measurement identifier - 0 for pose measurement, 1 for GOT and 2...N for landmark identifier */
    Eigen::VectorXd measurement_;	/* actual measurement - can take different sizes! */
    static Eigen::MatrixXd measurement_covariance_;
};


/* ############################## Defines GOTMeasurement class ##############################  */
class GOTMeasurement : public Measurement{
public:

    GOTMeasurement(uint32_t identifier, Eigen::VectorXd GOT_meas);
    Eigen::MatrixXd calculateHs(StateVector pose, Eigen::VectorXd landmark) override;
    Eigen::MatrixXd calculateHl(StateVector pose, Eigen::VectorXd landmark) override;
    Eigen::VectorXd inverseMeasurementModel(StateVector pose) override;
    Eigen::VectorXd MeasurementModel(StateVector pose, Eigen::VectorXd landmark) override;
    inline Eigen::MatrixXd getzCov() const override {return measurement_covariance_;}
    

private:
    static Eigen::MatrixXd measurement_covariance_; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

};


class LandmarkXYMeasurement : public Measurement{
	public:
        LandmarkXYMeasurement(uint32_t identifier, Eigen::VectorXd landmark_measurement);
        Eigen::MatrixXd calculateHs(StateVector pose, Eigen::VectorXd landmark) override;
        Eigen::MatrixXd calculateHl(StateVector pose, Eigen::VectorXd landmark) override;
        Eigen::VectorXd inverseMeasurementModel(StateVector pose) override;
        Eigen::VectorXd MeasurementModel(StateVector pose, Eigen::VectorXd landmark) override;
        inline Eigen::MatrixXd getzCov() const override {return measurement_covariance_;}
	private:
	    static Eigen::MatrixXd measurement_covariance_; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

};

class LandmarkXYYawMeasurement : public Measurement{
	public:

        LandmarkXYYawMeasurement(uint32_t identifier, Eigen::VectorXd landmark_measurement);
        Eigen::MatrixXd calculateHs(StateVector pose, Eigen::VectorXd landmark) override;
        Eigen::MatrixXd calculateHl(StateVector pose, Eigen::VectorXd landmark) override;
        Eigen::VectorXd inverseMeasurementModel(StateVector pose) override;
        Eigen::VectorXd MeasurementModel(StateVector pose, Eigen::VectorXd landmark) override;
        inline Eigen::MatrixXd getzCov() const override {return measurement_covariance_;}
	private:
        static Eigen::MatrixXd measurement_covariance_; 	/* measurement covariance - can take different sizes! static such that only one copy is saved in memory - also why it is placed in the subclass*/

};

} //namespace fastslam


#endif
