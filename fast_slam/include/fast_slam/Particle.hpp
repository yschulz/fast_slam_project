#ifndef FASTSLAM_PARTICLE_H
#define FASTSLAM_PARTICLE_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/random/mersenne_twister.hpp>
#include <boost/filesystem.hpp>

#include "nabo/nabo.h"

#include "fast_slam/MapTree.hpp"
#include "fast_slam/PathList.hpp"
#include "fast_slam/MeasurementSet.hpp"
#include "fast_slam/Helper.hpp"

namespace fastslam{


class Particle{
public:
    // Initialize a standard particle with "zero-pose" or custom pose
    Particle(StateVector initial_state = StateVector::Constant(0), StateMatrix initial_covariance = 0.01*StateMatrix::Identity(), uint32_t iteration = 0);

    // Copy constructer used in case where we need to make a copy of a Particle
    Particle(const Particle &particle_to_copy);       
    ~Particle();

    inline double getWeight() const {return weight_;}
    inline StateVector getPose() const {return path_->getPose();}
    std::vector<std::shared_ptr<Landmark>> getMap();

    void updateParticle(std::shared_ptr<MeasurementSet> measurement_set, StateVectorDerivative* state_dot, uint32_t iteration, std::chrono::nanoseconds delta_time, bool loop_closure);

private:
    void handleExMeas(MeasurementSet &measurent_set_existing, StateVector &state_proposal, std::vector<int> &id_exiting);
    void handleNewMeas(MeasurementSet &measurement_set_new, StateVector &state_proposal, std::vector<int> &id_new); // only moved up here to allow new landmarks to be added by Particle Set function

    void associateData(StateVector &state_proposal, std::shared_ptr<MeasurementSet> &measurement_set, MeasurementSet &measurement_set_existing, MeasurementSet &measurement_set_new, std::vector<int> &id_existing, std::vector<int> &id_new);

    StateVector drawSampleFromProposaleDistribution(StateVector &state_propagated, StateMatrix &Fs, StateMatrix &Fw, MeasurementSet &measurement_set_existing, std::vector<int> &id_existing);
    StateVector motionModel(StateVector &state_old, StateVectorDerivative &state_dot, std::chrono::nanoseconds delta_time);
    StateVector drawSampleRandomPose(StateVector &state_proposal_mean, StateMatrix &state_proposal_covariance);
    void calculateImportanceWeight(MeasurementSet &measurment_set_existing, StateVector &state_proposal, StateMatrix &Fw, std::vector<int> &id_existing);
    StateMatrix calculateFs(StateVector &state_old, StateVectorDerivative &state_dot, std::chrono::nanoseconds &delta_time);
    StateMatrix calculateFw(StateVector &state_old, StateVectorDerivative &state_dot, std::chrono::nanoseconds &delta_time);

    void KFCholeskyUpdate(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::VectorXd &v, Eigen::MatrixXd &R, Eigen::MatrixXd &H);
    void updateSpatialIndex() const;

    std::unique_ptr<Path> path_;
    std::unique_ptr<MapTree> map_;
    
    // Persistent spatial index for efficient data association
    mutable std::unique_ptr<Nabo::NNSearchD> spatial_index_;
    mutable bool spatial_index_dirty_;

    uint32_t current_iteration_;
    double weight_;
    double weight_distance_;

    bool loop_is_closed_;

    bool slow_init_ = false;
    bool use_motion_model_jacobian_ = false;
    bool always_reset_particle_covariance_ = true;
    bool use_numerical_stabilized_kalman_filters_ = true;
    bool force_covariance_symmetry_ = true;
    bool add_landmarks_after_resampling_ = true;

    // particle covariance
    StateMatrix particle_covariance_; 

    // motion model covariance
    static StateMatrix motion_model_covariance_;

    static boost::mt19937 rng; // Creating a new random number generator every time could be optimized
    //rng.seed(static_cast<unsigned int>(time(0)));
};

}

#endif
