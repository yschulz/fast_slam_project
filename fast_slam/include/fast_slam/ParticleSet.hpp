#ifndef FASTSLAM_PARTICLESET_HPP
#define FASTSLAM_PARTICLESET_HPP

#include <random>
#include <fstream>
#include <cstdlib>
#include <string>
#include <vector>
#include <chrono>

#include "fast_slam/Particle.hpp"
#include "fast_slam/MeasurementSet.hpp"
#include "fast_slam/Helper.hpp"

namespace fastslam{

class ParticleSet{
    public:
        ParticleSet(uint32_t n_particles = 100, StateVector initial_state = StateVector::Constant(0), StateMatrix state_covariance = 0.1*StateMatrix::Identity());
        ~ParticleSet();

        void updateParticleSet(std::shared_ptr<MeasurementSet> measurement_set, StateVectorDerivative state_dot, std::chrono::nanoseconds delta_time);

        inline StateVector getLatestPoseEstimate() const {return state_mean_->getPose();}
        inline StateMatrix getLatestCovarianceEstimate() const {return state_covariance_;}

        std::vector<StateVector> getAllParticlePoseEstimates() const;
        std::pair<StateVector, std::vector<std::shared_ptr<Landmark>>> getBestParticle();

        inline uint32_t getNParticles() const {return n_particles_;}
        inline uint32_t getCurrentIteration() const {return iterations_since_start_;}


    private:
        void resample();
        void estimateDistribution(std::chrono::nanoseconds delta_time);

        std::vector<std::unique_ptr<Particle>> particles_;

        uint32_t n_particles_;
        StateMatrix state_covariance_;

        // number of interations since time zero
        uint32_t iterations_since_start_; 

        bool loop_is_closed_;

        // instance of path Class to keep track of the estimated mean of the Particle filter!
        std::unique_ptr<Path> state_mean_;
};

} //namespace fastslam

#endif
