#ifndef FASTSLAM_MEASUREMENTSET_H
#define FASTSLAM_MEASUREMENTSET_H

#include "fast_slam/Measurement.hpp"

#include <memory>

namespace fastslam{

struct MeasurementSetNode {
    std::shared_ptr<Measurement> measurement;
    std::unique_ptr<MeasurementSetNode> next_node;
    uint32_t measurement_identifier;
};

class MeasurementSet
{
public:
    /* variables */


    /* functions */
    MeasurementSet();
    // MeasurementSet(Measurement *meas);
    ~MeasurementSet();
    inline void emptyMeasurementSet(){  n_measurements_ = 0; 
                                        first_measurement_node_.reset();}
    void addMeasurement(std::shared_ptr<Measurement> measurement);
    // int countNumberOfMeasurements();

    std::shared_ptr<Measurement> getMeasurement(uint32_t identifier) const;
    inline uint32_t getNumberOfMeasurements() const {return n_measurements_;}

    inline std::unique_ptr<MeasurementSetNode>* getHeadPointer() {return &first_measurement_node_;}

private:
    std::shared_ptr<Measurement> getMeasurement(const std::unique_ptr<MeasurementSetNode> &next_ptr, uint32_t identifier) const;
    void addMeasurement(std::unique_ptr<MeasurementSetNode> &next_ptr, std::shared_ptr<Measurement> &measurement);
    
    std::unique_ptr<MeasurementSetNode> first_measurement_node_;
    uint32_t n_measurements_;
};

} //namespace fastslam

#endif
