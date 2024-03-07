#include "fast_slam/ParticleSet.hpp"

namespace fastslam{

ParticleSet::ParticleSet(uint32_t n_particles, StateVector initial_state, StateMatrix state_covariance):
        n_particles_(n_particles),
        state_covariance_(state_covariance),
        iterations_since_start_(0),
        loop_is_closed_(false){
    // makes new path to keep track of the estimated mean of the Particle filter!
    state_mean_ = std::make_unique<Path>(initial_state, iterations_since_start_); 

    particles_.reserve(n_particles_);

    for(size_t i = 0; i<n_particles_; i++){
        particles_.push_back(std::make_unique<Particle>(initial_state, state_covariance, iterations_since_start_));
    }
}

ParticleSet::~ParticleSet(){}

std::vector<StateVector> ParticleSet::getAllParticlePoseEstimates() const{
    std::vector<StateVector> out;
    for(const auto &particle : particles_)
        out.push_back(particle->getPose());
    
    return out;
}

std::pair<StateVector, std::vector<std::shared_ptr<Landmark>>> ParticleSet::getBestParticle(){
    std::pair<StateVector, std::vector<std::shared_ptr<Landmark>>> out;
    auto max_element = std::max_element(particles_.begin(), particles_.end(), [](std::unique_ptr<Particle> &a, std::unique_ptr<Particle> &b){
        return a->getWeight() < b->getWeight();
    });
    out.first = (*max_element)->getPose();

    out.second = (*max_element)->getMap();

    return out;
}

void ParticleSet::updateParticleSet(std::shared_ptr<MeasurementSet> measurement_set, StateVectorDerivative state_dot, std::chrono::nanoseconds delta_time){

    iterations_since_start_++;


    double weighted_sum = 0;
    double sum_squared_weights = 0;

    // update each particle
	for(auto &particle : particles_){
        particle->updateParticle(measurement_set, &state_dot, iterations_since_start_, delta_time, loop_is_closed_);
    	weighted_sum += particle->getWeight();
    	sum_squared_weights += pow(particle->getWeight(), 2);
	}
    double kishs_effective_sample_size = pow(weighted_sum,2)/sum_squared_weights;


    estimateDistribution(delta_time);
    

    std::cout << "Kishs effective sample size: " << kishs_effective_sample_size << std::endl;
    
    // resample only if Kishs effective sample size drops below threshold
    if(kishs_effective_sample_size < 0.8 * n_particles_)
        resample();
    
}

void ParticleSet::resample(){
    // Resampling wheel
    cout << "resampling..." << endl;

    vector<std::unique_ptr<Particle>> particles_tmp;
    particles_tmp.reserve(n_particles_);

    auto max_element = std::max_element(particles_.begin(), particles_.end(), [](std::unique_ptr<Particle> &a, std::unique_ptr<Particle> &b){
        return a->getWeight() < b->getWeight();
    });

    if ((*max_element)->getWeight() == 0){
        cout << "something went wrong, all particles have 0 weight" << endl;
        return;
    }

    // generate random index between 1 and number of particles
    std::default_random_engine generator;
    std::uniform_int_distribution<uint32_t> distribution_int(0, n_particles_);
    std::uniform_real_distribution<double> distribution_double(0, 1);

    uint32_t random_index = distribution_int(generator); // random index

    double beta = 0;


    for(size_t i = 0; i < n_particles_; i++){
        // generate random addition to beta
        double rand = distribution_double(generator);
        beta += rand * 2 * (*max_element)->getWeight();

        double weight = particles_.at(random_index)->getWeight();
        while (beta > weight){
            beta -= weight;

            random_index += 1;
            if (random_index >= n_particles_){
                random_index = 1;
            }
            weight = particles_.at(random_index)->getWeight();
        }
        particles_tmp.push_back(std::make_unique<Particle>(*(particles_.at(random_index))));
    }

    particles_.clear();
    for(auto &particle : particles_tmp){
        particles_.push_back(std::move(particle));
    }

    cout << "Done resampling!" << endl;
}

void ParticleSet::estimateDistribution(std::chrono::nanoseconds delta_time){
    double sum_of_weights = 0;
    
    // get sum of all weights
    for(auto &particle : particles_){
        if(particle->getWeight() != particle->getWeight()){
            cout << "Err NaN in Particle: " << endl;
        }
        sum_of_weights += particle->getWeight();
    }


    // get weighted mean of all state vector
    StateVector state_mean_estimate = StateVector::Zero();
    double sum_of_weights_squared = 0;

    for(auto &particle : particles_){
        auto normalized_weight = particle->getWeight() / sum_of_weights;

        // weighted mean!
        state_mean_estimate += normalized_weight * particle->getPose(); 
    }


    // get weighted mean of state covariance
    StateMatrix state_covariance_estimate = StateMatrix::Zero();

    for(auto &particle : particles_){
        StateMatrix state_covariance_estimate_particle = StateMatrix::Zero();
        double normalized_weight = particle->getWeight() / sum_of_weights;
        auto particle_distance_from_mean = particle->getPose() - state_mean_estimate;

        state_covariance_estimate_particle = particle_distance_from_mean * particle_distance_from_mean.transpose() * normalized_weight;

        state_covariance_estimate += state_covariance_estimate_particle;
        sum_of_weights_squared += normalized_weight * normalized_weight;
    }

    state_covariance_estimate *= 1/(1-sum_of_weights_squared);
    state_covariance_ = state_covariance_estimate;
    state_mean_->addPose(state_mean_estimate, iterations_since_start_, delta_time);
}

} //namespace fastslam
