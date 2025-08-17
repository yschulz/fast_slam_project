
#include "fast_slam/Particle.hpp"

namespace fastslam{


Particle::Particle(StateVector initial_state, StateMatrix initial_covariance, uint32_t iteration):
		// M(3,0), K(1), kt(0), w_DA(0.), dAindices(), lc(false)  // default Constructor definition
        current_iteration_(iteration),
        weight_(1),
        weight_distance_(0.0),
        loop_is_closed_(false), 
        particle_covariance_(initial_covariance),
        spatial_index_(nullptr),
        spatial_index_dirty_(true){

    // make path and map
    path_ = std::make_unique<Path>(initial_state, iteration);
    map_ = std::make_unique<MapTree>();
}

Particle::Particle(const Particle &particle_to_copy){

    path_ = std::make_unique<Path>(*(particle_to_copy.path_));
    map_ = std::make_unique<MapTree>(*(particle_to_copy.map_));
    weight_ = particle_to_copy.weight_;
    weight_distance_ = particle_to_copy.weight_distance_;
    loop_is_closed_ = particle_to_copy.loop_is_closed_;

    slow_init_ = particle_to_copy.slow_init_;
    use_motion_model_jacobian_ = particle_to_copy.use_motion_model_jacobian_;
    always_reset_particle_covariance_ = particle_to_copy.always_reset_particle_covariance_;
    use_numerical_stabilized_kalman_filters_ = particle_to_copy.use_numerical_stabilized_kalman_filters_;
    force_covariance_symmetry_ = particle_to_copy.force_covariance_symmetry_;
    add_landmarks_after_resampling_ = particle_to_copy.add_landmarks_after_resampling_;

    particle_covariance_ = particle_to_copy.particle_covariance_;
    
    // Spatial index will be rebuilt on demand - don't copy the index itself
    spatial_index_ = nullptr;
    spatial_index_dirty_ = true;
}

Particle::~Particle(){}

void Particle::updateSpatialIndex() const {
    if (!spatial_index_dirty_ && spatial_index_) {
        return; // Index is already up to date
    }
    
    if (map_->getNLandmarks() == 0) {
        spatial_index_.reset();
        spatial_index_dirty_ = false;
        return;
    }
    
    // Build landmark position matrix for spatial indexing
    Eigen::MatrixXd landmark_positions(2, map_->getNLandmarks());
    for(uint32_t i = 1; i <= map_->getNLandmarks(); i++){
        auto landmark_ptr = map_->extractLandmarkNodePointer(i);
        if (landmark_ptr != nullptr){
            landmark_positions.col(i-1) = landmark_ptr->landmark_pose;
        }
    }
    
    // Create new spatial index - properly manages memory with smart pointer
    spatial_index_.reset(Nabo::NNSearchD::createKDTreeLinearHeap(landmark_positions));
    spatial_index_dirty_ = false;
}

std::vector<std::shared_ptr<Landmark>> Particle::getMap(){
    std::vector<std::shared_ptr<Landmark>> out;

    for(uint32_t i = 1; i<=map_->getNLandmarks(); i++)
        out.push_back(map_->extractLandmarkNodePointer(i));

    return out;
}

void Particle::updateParticle(std::shared_ptr<MeasurementSet> measurement_set, StateVectorDerivative* state_dot, uint32_t iteration, std::chrono::nanoseconds delta_time, bool loop_closure){

	loop_is_closed_ = loop_closure;

    StateVector state_proposal;
    StateVector state_old = path_->getPose();
    current_iteration_ = iteration;

    MeasurementSet measurement_set_new;
    MeasurementSet measurement_set_existing;

	std::vector<int> id_new;
    std::vector<int> id_existing;

	StateVector state_propagated = motionModel(state_old, *state_dot, delta_time);


    associateData(state_propagated, measurement_set, measurement_set_existing, measurement_set_new, id_existing, id_new);



    StateMatrix Fs, Fw;
    if(use_motion_model_jacobian_){
        Fs = calculateFs(state_old, *state_dot, delta_time);
        Fw = calculateFw(state_old, *state_dot, delta_time);
    }
    else{
        Fs = StateMatrix::Identity();
        Fw = StateMatrix::Identity();
    }

    
    state_proposal = drawSampleFromProposaleDistribution(state_propagated, Fs, Fw, measurement_set_existing, id_existing);

    // we are done estimating our pose and add it to the path!
    path_->addPose(state_proposal, current_iteration_, delta_time); 

    calculateImportanceWeight(measurement_set_existing, state_proposal, Fw, id_existing);

    handleExMeas(measurement_set_existing, state_proposal, id_existing);

    handleNewMeas(measurement_set_new, state_proposal, id_new);

    

    // we are done estimating our pose and add it to the path!
    

    // handleExMeas(measurement_set_existing, state_propagated, id_existing);

    // handleNewMeas(measurement_set_new, state_propagated, id_new);

    // calculateImportanceWeight(measurement_set_existing, state_propagated, Fw, id_existing);

    // state_proposal = drawSampleFromProposaleDistribution(state_propagated, Fs, Fw, measurement_set_existing, id_existing);

    // path_->addPose(state_proposal, current_iteration_, delta_time); 

    std::cout << "state old:         " << state_old.transpose() << "\n";
    std::cout << "state propagated:  " << state_propagated.transpose() << "\n";
    std::cout << "state proposed:    " << state_proposal.transpose() << "\n";
    std::cout << "number of existing measurements:  " << measurement_set_existing.getNumberOfMeasurements() << "  number of new: " << measurement_set_new.getNumberOfMeasurements() << "  number of landmarks: " << map_->getNLandmarks() << "\n";
    std::cout << "Particle weight:  " << getWeight() << "\n\n";
}

void Particle::associateData(StateVector &state_propagated, std::shared_ptr<MeasurementSet> &measurement_set, MeasurementSet &measurement_set_existing, MeasurementSet &measurement_set_new, std::vector<int> &id_existing, std::vector<int> &id_new){
    // if no landmarks exist in map add all as new

    if(map_->getNLandmarks() == 0){
        std::unique_ptr<MeasurementSetNode> *curr;
        if(measurement_set != nullptr)
            curr = measurement_set->getHeadPointer();

        for(size_t i=1; i<=measurement_set->getNumberOfMeasurements(); i++){
            id_new.push_back(i);
            measurement_set_new.addMeasurement((*curr)->measurement);
            curr = &(*curr)->next_node;
        }
        return;
    }


    Eigen::MatrixXd map_landmark_matrix(2,0);
    Eigen::MatrixXd landmark_calculated(2,0);
    Eigen::MatrixXd measurement_calculated(2,0);

    //Extract Landmarks from map
    //TODO: not necessary, integrate kd tree directly 
    for(size_t i=1; i<= map_->getNLandmarks(); i++){
        auto landmark_ptr = map_->extractLandmarkNodePointer(i);
        if (landmark_ptr != NULL){
            map_landmark_matrix.conservativeResize(map_landmark_matrix.rows(), map_landmark_matrix.cols()+1);
            map_landmark_matrix.col(map_landmark_matrix.cols()-1) = landmark_ptr->landmark_pose;
        }
    }

    //Extract measurements from the set.
    std::unique_ptr<MeasurementSetNode> *curr;
    if(measurement_set != nullptr)
        curr = measurement_set->getHeadPointer();

    // if we have a non-emtpy measurement set update the particles, otherwise just do resampling
    while(*curr != nullptr){
        landmark_calculated.conservativeResize(landmark_calculated.rows(),landmark_calculated.cols()+1);
        landmark_calculated.col(landmark_calculated.cols()-1) = (*curr)->measurement->inverseMeasurementModel(state_propagated);

        curr = &(*curr)->next_node;
    }


    Eigen::MatrixXi nn_indices(1, landmark_calculated.cols());
    Eigen::MatrixXd nn_distances(1, landmark_calculated.cols());

    // Use persistent spatial index for efficient nearest neighbor search
    updateSpatialIndex();
    
    if (spatial_index_) {
        int n_nearest_neighbors = 1;
        spatial_index_->knn(landmark_calculated, nn_indices, nn_distances, n_nearest_neighbors, 0, Nabo::NNSearchD::ALLOW_SELF_MATCH, 1.0);
    } else {
        // No landmarks in map, all distances are infinity
        nn_distances.setConstant(std::numeric_limits<double>::infinity());
    }

    curr = measurement_set->getHeadPointer();

    //add measurements to the sets of existing and new measurements
    int j = 1;
    for(int i=0; i<nn_distances.cols(); i++){
        if(nn_distances(i) < 0.8 || loop_is_closed_){
            id_existing.push_back(nn_indices(i)+1);
            measurement_set_existing.addMeasurement((*curr)->measurement);
        }
        else{
            id_new.push_back(map_landmark_matrix.cols()+j);
            j++;
            measurement_set_new.addMeasurement((*curr)->measurement);
        }
        curr = &(*curr)->next_node;
    }
}

void Particle::handleExMeas(MeasurementSet &measurent_set_existing, StateVector &state_proposal, std::vector<int> &id_existing){
    if (measurent_set_existing.getNumberOfMeasurements() == 0 )
        return;


    for(size_t i = 1; i <= measurent_set_existing.getNumberOfMeasurements(); i++) {

        auto current_measurement = measurent_set_existing.getMeasurement(i);

        auto landmark_old = map_->extractLandmarkNodePointer(id_existing.at(i-1));

        Eigen::VectorXd measurement_calculated = current_measurement->MeasurementModel(state_proposal, landmark_old->landmark_pose);
        Eigen::VectorXd measurement_difference = current_measurement->getMeasurement() - measurement_calculated;

        Eigen::MatrixXd landmark_jacobian = current_measurement->calculateHl(state_proposal, landmark_old->landmark_pose);


        auto landmark_update = std::make_shared<Landmark>();
        if(use_numerical_stabilized_kalman_filters_){
            Eigen::VectorXd x = landmark_old->landmark_pose;
            Eigen::MatrixXd P = landmark_old->landmark_covariance;
            Eigen::MatrixXd H = landmark_jacobian;
            Eigen::MatrixXd R = current_measurement->getzCov();
            Eigen::VectorXd v = measurement_difference;

            KFCholeskyUpdate(x, P, v, R, H);

            landmark_update->landmark_identifier = id_existing.at(i-1);
            landmark_update->landmark_pose = x;
            landmark_update->landmark_covariance = P;
        }
        else{
            // innovation covariance
            Eigen::MatrixXd Zk = current_measurement->getzCov() + landmark_jacobian * landmark_old->landmark_covariance * landmark_jacobian.transpose();
            // kalman gain
            Eigen::MatrixXd Kk = landmark_old->landmark_covariance * landmark_jacobian.transpose() * Zk.inverse();

            landmark_update->landmark_identifier = id_existing.at(i-1);
            landmark_update->landmark_pose += Kk*(current_measurement->getMeasurement() - measurement_calculated);

            Eigen::MatrixXd tmp_matrix = Kk * landmark_jacobian;
            landmark_update->landmark_covariance = (Eigen::MatrixXd::Identity(tmp_matrix.rows(),tmp_matrix.cols()) - tmp_matrix) * landmark_update->landmark_covariance;
        }

        //make symmetric
        if(force_covariance_symmetry_)
            landmark_update->landmark_covariance = (landmark_update->landmark_covariance + landmark_update->landmark_covariance.transpose()) * 0.5;


        if (landmark_update->landmark_pose != landmark_update->landmark_pose) {
            cout << "landmark update NaN err, ID: " << current_measurement->getC() << endl;
        }

        map_->correctLandmark(landmark_update);
        spatial_index_dirty_ = true; // Mark spatial index as needing update
    }
}


void Particle::handleNewMeas(MeasurementSet &measurement_set_new, StateVector &state_proposal, std::vector<int> &id_new){

    if (measurement_set_new.getNumberOfMeasurements() == 0 )
        return;

    for(uint32_t i = 1; i <= measurement_set_new.getNumberOfMeasurements(); i++) {

        auto current_measurement = measurement_set_new.getMeasurement(i);

        auto landmark = std::make_shared<Landmark>();
        landmark->landmark_identifier = id_new.at(i-1);
        landmark->landmark_pose = current_measurement->inverseMeasurementModel(state_proposal);

        Eigen::MatrixXd landmark_jacobian = current_measurement->calculateHl(state_proposal, landmark->landmark_pose);

        landmark->landmark_covariance = (landmark_jacobian.transpose() * (current_measurement->getzCov()).inverse() * landmark_jacobian).inverse();

        //make symmetric
        if(force_covariance_symmetry_)
            landmark->landmark_covariance = (landmark->landmark_covariance + (landmark->landmark_covariance).transpose()) * 0.5; 

        map_->insertLandmark(landmark);
        spatial_index_dirty_ = true; // Mark spatial index as needing update

    }
}

StateVector Particle::drawSampleFromProposaleDistribution(StateVector &state_propagated, StateMatrix &Fs, StateMatrix &Fw, MeasurementSet &measurement_set_existing, std::vector<int> &id_existing){

    StateVector state_proposal = state_propagated;

    if(always_reset_particle_covariance_)
        particle_covariance_ = StateMatrix::Zero();

    StateMatrix state_proposal_covariance =    Fs.transpose() * particle_covariance_ * Fs + 
                                                Fw.transpose() * motion_model_covariance_ * Fw; 

    if (measurement_set_existing.getNumberOfMeasurements() != 0){

        for(size_t i = 1; i <= measurement_set_existing.getNumberOfMeasurements(); i++) {

            auto current_measurement = measurement_set_existing.getMeasurement(i);
            auto landmark_old = map_->extractLandmarkNodePointer(id_existing.at(i-1));

            //resizes automatically due to the "=" operator
            Eigen::MatrixXd landmark_jacobian = current_measurement->calculateHl(state_propagated, landmark_old->landmark_pose); 
            Eigen::MatrixXd state_jacobian = current_measurement->calculateHs(state_propagated, landmark_old->landmark_pose); 


            Eigen::MatrixXd Zki = current_measurement->getzCov() + landmark_jacobian * (landmark_old->landmark_covariance) * landmark_jacobian.transpose();

            Eigen::VectorXd measurement_calculated = current_measurement->MeasurementModel(state_propagated, landmark_old->landmark_pose);

            if(use_numerical_stabilized_kalman_filters_){
                Eigen::VectorXd x = state_proposal;
                Eigen::MatrixXd P = state_proposal_covariance;
                Eigen::VectorXd v = (current_measurement->getMeasurement() - measurement_calculated);
                Eigen::MatrixXd R = Zki;
                Eigen::MatrixXd H = state_jacobian;

                KFCholeskyUpdate(x, P, v, R, H);

                state_proposal = x;
                state_proposal_covariance = P;
            }
            else{
                Eigen::MatrixXd Kk;
                Kk = state_proposal_covariance * state_jacobian.transpose() * 
                        (state_jacobian * state_proposal_covariance * state_jacobian.transpose() + Zki).inverse();

                state_proposal_covariance = (state_jacobian.transpose() * Zki.inverse() * state_jacobian + state_proposal_covariance.inverse()).inverse();
                state_proposal = state_proposal + state_proposal_covariance * state_jacobian.transpose() * Zki.inverse() * 
                                (current_measurement->getMeasurement() - measurement_calculated); 
            }

            if(force_covariance_symmetry_){
                //make symmetric
                state_proposal_covariance = (state_proposal_covariance + state_proposal_covariance.transpose()) * 0.5; 
            }
        }
    }

    particle_covariance_ = state_proposal_covariance;

    return drawSampleRandomPose(state_proposal, state_proposal_covariance);

}


void Particle::KFCholeskyUpdate(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::VectorXd &v, Eigen::MatrixXd &R, Eigen::MatrixXd &H){
    Eigen::MatrixXd PHt = P*H.transpose();
    Eigen::MatrixXd S = H*PHt + R;

    // FIXME: why use conjugate()?
    //make symmetric
    if(force_covariance_symmetry_)
        S = (S+S.transpose()) * 0.5; 

    Eigen::MatrixXd SChol = S.llt().matrixU();

     //tri matrix
    Eigen::MatrixXd SCholInv = SChol.inverse();
    Eigen::MatrixXd W1 = PHt * SCholInv;
    Eigen::MatrixXd W = W1 * SCholInv.transpose();

    x = x + W*v;
    P = P - W1*W1.transpose();
}


//http://moby.ihme.washington.edu/bradbell/mat2cpp/randn.cpp.xml
Eigen::MatrixXd randn(int m, int n){
    // use formula 30.3 of Statistical Distributions (3rd ed)
    // Merran Evans, Nicholas Hastings, and Brian Peacock
    int urows = m * n + 1;
    Eigen::VectorXd u(urows);

    //u is a random matrix
    for (int r=0; r<urows; r++) {
        // FIXME: better way?
        u(r) = std::rand() * 1.0/RAND_MAX;
    }



    Eigen::MatrixXd x(m,n);

    int     i, j, k;
    double   square, amp, angle;

    k = 0;
    for(i = 0; i < m; i++) {
        for(j = 0; j < n; j++) {
            if( k % 2 == 0 ) {
                square = - 2. * std::log( u(k) );
                if( square < 0. )
                    square = 0.;
                amp = sqrt(square);
                angle = 2. * M_PI * u(k+1);
                x(i, j) = amp * std::sin( angle );
            }
            else
                x(i, j) = amp * std::cos( angle );

            k++;
        }
    }

    return x;
}

Eigen::MatrixXd rand(int m, int n)
{
    Eigen::MatrixXd x(m,n);
    int i, j;
    float rand_max = float(RAND_MAX);

    for(i = 0; i < m; i++) {
        for(j = 0; j < n; j++)
            x(i, j) = float(std::rand()) / rand_max;
    }
    return x;
}

StateVector Particle::drawSampleRandomPose(StateVector &state_proposal, StateMatrix &state_proposal_covariance){
    Eigen::MatrixXd Cov = state_proposal_covariance;

    //make symmetric
    if(force_covariance_symmetry_)
        Cov = (Cov+Cov.transpose()) * 0.5; 

    //choleksy decomposition
    Eigen::MatrixXd S = Cov.llt().matrixL();
    Eigen::MatrixXd X = randn(3,1);

    auto sample = S * X + state_proposal;
    return sample;
}

// Ts == sample time
StateVector Particle::motionModel(StateVector &state_old, StateVectorDerivative &state_dot, std::chrono::nanoseconds delta_time) {
    StateVector state_propagated(state_old);

    double theta = state_old(2);
    double dt_in_s = std::chrono::duration<double>(delta_time).count();

    if (dt_in_s > 3) {
        cout << "Motion model: Sample rate error" << endl;
        return state_propagated; // error with the sampling time, just return old pose estimate
    }

    // Extract velocity components from state_dot
    double vx = state_dot(0);
    double vy = state_dot(1);
    double omega = state_dot(2);

    // Convert to body frame velocity
    double v_forward = vx * cos(theta) + vy * sin(theta);
    double v_lateral = -vx * sin(theta) + vy * cos(theta);

    // Differential drive motion model with proper kinematics
    if (abs(omega) < 1e-6) {
        // Straight line motion
        state_propagated(0) += v_forward * cos(theta) * dt_in_s;
        state_propagated(1) += v_forward * sin(theta) * dt_in_s;
        state_propagated(2) += omega * dt_in_s;
    } else {
        // Curved motion with instantaneous center of rotation
        double radius = v_forward / omega;
        double dtheta = omega * dt_in_s;
        
        state_propagated(0) += radius * (sin(theta + dtheta) - sin(theta));
        state_propagated(1) += radius * (-cos(theta + dtheta) + cos(theta));
        state_propagated(2) += dtheta;
    }

    // Normalize angle to [-pi, pi]
    while (state_propagated(2) > M_PI) state_propagated(2) -= 2.0 * M_PI;
    while (state_propagated(2) < -M_PI) state_propagated(2) += 2.0 * M_PI;

    return state_propagated;
}

// Motion model Jacobian relative to pose
StateMatrix Particle::calculateFs(StateVector &state_old, StateVectorDerivative &state_dot, std::chrono::nanoseconds &delta_time){
    StateMatrix Fs = StateMatrix::Identity();
    double dt_in_s = std::chrono::duration<double>(delta_time).count();
    
    double theta = state_old(2);
    double vx = state_dot(0);
    double vy = state_dot(1);
    double omega = state_dot(2);
    
    // Convert to body frame velocity
    double v_forward = vx * cos(theta) + vy * sin(theta);
    
    if (abs(omega) < 1e-6) {
        // Straight line motion Jacobian
        Fs(0,2) = -v_forward * sin(theta) * dt_in_s;
        Fs(1,2) = v_forward * cos(theta) * dt_in_s;
    } else {
        // Curved motion Jacobian
        double radius = v_forward / omega;
        double dtheta = omega * dt_in_s;
        
        Fs(0,2) = radius * (cos(theta + dtheta) - cos(theta));
        Fs(1,2) = radius * (sin(theta + dtheta) - sin(theta));
    }
    
    return Fs;
}

// Motion model Jacobian relative to motion
StateMatrix Particle::calculateFw(StateVector &state_old, StateVectorDerivative &state_dot, std::chrono::nanoseconds &delta_time){
    StateMatrix Fw = StateMatrix::Zero();
    double dt_in_s = std::chrono::duration<double>(delta_time).count();
    
    double theta = state_old(2);
    double vx = state_dot(0);
    double vy = state_dot(1);
    double omega = state_dot(2);
    
    // Convert to body frame velocity
    double v_forward = vx * cos(theta) + vy * sin(theta);
    
    if (abs(omega) < 1e-6) {
        // Straight line motion Jacobian w.r.t control inputs
        Fw(0,0) = cos(theta) * cos(theta) * dt_in_s;
        Fw(0,1) = cos(theta) * sin(theta) * dt_in_s;
        Fw(1,0) = sin(theta) * cos(theta) * dt_in_s;
        Fw(1,1) = sin(theta) * sin(theta) * dt_in_s;
        Fw(2,2) = dt_in_s;
    } else {
        // Curved motion Jacobian w.r.t control inputs
        double radius = v_forward / omega;
        double dtheta = omega * dt_in_s;
        
        Fw(0,0) = (cos(theta) / omega) * (sin(theta + dtheta) - sin(theta));
        Fw(0,1) = (sin(theta) / omega) * (sin(theta + dtheta) - sin(theta));
        Fw(1,0) = (cos(theta) / omega) * (-cos(theta + dtheta) + cos(theta));
        Fw(1,1) = (sin(theta) / omega) * (-cos(theta + dtheta) + cos(theta));
        Fw(2,2) = dt_in_s;
    }
    
    return Fw;
}

void Particle::calculateImportanceWeight(MeasurementSet &measurment_set_existing, StateVector &state_proposal, StateMatrix &Fw, std::vector<int> &id_existing){
    if (measurment_set_existing.getNumberOfMeasurements() == 0)
        return;

    Eigen::MatrixXd weight_covariance;
    double wi = 1;

    for(uint32_t i = 1; i <= measurment_set_existing.getNumberOfMeasurements(); i++ ) {

        auto current_measurement = measurment_set_existing.getMeasurement(i);
        auto landmark_old = map_->extractLandmarkNodePointer(id_existing.at(i-1));

        //resizes automatically due to the "=" operator
        Eigen::MatrixXd landmark_jacobian = current_measurement->calculateHl(state_proposal, landmark_old->landmark_pose);
        Eigen::MatrixXd state_jacobian = current_measurement->calculateHs(state_proposal, landmark_old->landmark_pose);
        
        Eigen::VectorXd measurement_calculated = current_measurement->MeasurementModel(state_proposal, landmark_old->landmark_pose);

        Eigen::VectorXd measurement_difference = current_measurement->getMeasurement() - measurement_calculated;


        // weight_covariance = state_jacobian * Fw * motion_model_covariance_ * Fw.transpose() * state_jacobian.transpose() + 
        //                 landmark_jacobian * landmark_old->landmark_covariance * landmark_jacobian.transpose() + current_measurement->getzCov();
        weight_covariance = landmark_jacobian * landmark_old->landmark_covariance * landmark_jacobian.transpose() + current_measurement->getzCov();

        double exponent = measurement_difference.transpose() * weight_covariance.inverse() * measurement_difference;
        double determinant = (2 * M_PI * weight_covariance).determinant();

        wi *= exp(-0.5 * exponent) / (sqrt(determinant));

        // std::cout << "See here state component:  \n" << state_jacobian * Fw * motion_model_covariance_ * Fw.transpose() * state_jacobian.transpose() << "\n And see here the measurement component: \n" << landmark_jacobian * landmark_old->landmark_covariance * landmark_jacobian.transpose() + current_measurement->getzCov() << "\n";
        // std::cout << "partial weight:  " << exp(-0.5 * exponent) / (sqrt(determinant)) << "\n";
    }

    weight_ = wi;
}

// StateMatrix Particle::motion_model_covariance_ = 0.000001*StateMatrix::Identity(); // static variable - has to be declared outside class!
StateMatrix Particle::motion_model_covariance_ = (StateMatrix() << 0.001, 0.0, 0.0, 
                                                                    0.0, 0.001, 0.0, 
                                                                    0.0, 0.0, 0.000001).finished(); // static variable - has to be declared outside class!
// (Eigen::Matrix4d() << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16).finished();
boost::mt19937 Particle::rng; // Creating a new random number generator every time could be optimized
//rng.seed(static_cast<unsigned int>(time(0)));


} //namespace fastslam
