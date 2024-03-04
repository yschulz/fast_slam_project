#include "fast_slam/MeasurementSet.hpp"

namespace fastslam{

MeasurementSet::MeasurementSet(){
    first_measurement_node_ = nullptr;
    n_measurements_ = 0;
}

// MeasurementSet::MeasurementSet(Measurement *meas){
//     first_measurement_node_ = new MeasurementSetNode;
//     first_measurement_node_->meas = meas;
//     first_measurement_node_->next_node = NULL;
//     nMeas = 1;
//     first_measurement_node_->measIdentifier = nMeas;
//     //cout << "n1: " << nMeas << endl;
// }

MeasurementSet::~MeasurementSet(){}

void MeasurementSet::addMeasurement(std::shared_ptr<Measurement> measurement){
    if(first_measurement_node_ == nullptr){
        std::unique_ptr<MeasurementSetNode> new_node = std::make_unique<MeasurementSetNode>();
        new_node->measurement = measurement;
        new_node->next_node = nullptr;
        new_node->measurement_identifier = n_measurements_ + 1;
        first_measurement_node_ = std::move(new_node);
        n_measurements_++;

    }
    else
        addMeasurement(first_measurement_node_, measurement);
}

void MeasurementSet::addMeasurement(std::unique_ptr<MeasurementSetNode> &next_ptr, std::shared_ptr<Measurement> &measurement){
    if (next_ptr->next_node == nullptr){
        std::unique_ptr<MeasurementSetNode> new_node = std::make_unique<MeasurementSetNode>();
        new_node->measurement = measurement;
        new_node->next_node = nullptr;
        new_node->measurement_identifier = n_measurements_ + 1;
        next_ptr->next_node = std::move(new_node);
        n_measurements_++;
    }
    else{
        addMeasurement(next_ptr->next_node, measurement);
    }
}

// int MeasurementSet::countNumberOfMeasurements(){

//     if (first_measurement_node_==NULL){
//         nMeas = 0;
//         return 0;
//     }
//     else{
//         int i = 1;
//         MeasurementSetNode* tmp_pointer = first_measurement_node_;
//         while(tmp_pointer->next_node != NULL){
//             tmp_pointer = tmp_pointer->next_node;
//             i++;
//         }
//         nMeas = i;
//         return i;
//     }

// }


std::shared_ptr<Measurement> MeasurementSet::getMeasurement(uint32_t identifier) const{
    return getMeasurement(first_measurement_node_, identifier);
}

std::shared_ptr<Measurement> MeasurementSet::getMeasurement(const std::unique_ptr<MeasurementSetNode> &next_ptr, uint32_t identifier) const{
    if(next_ptr->measurement_identifier != identifier)
        return getMeasurement(next_ptr->next_node, identifier);
    else
        return next_ptr->measurement;
}


} //namespace fastslam
