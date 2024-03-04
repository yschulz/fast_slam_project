#include "fast_slam/PathList.hpp"

namespace fastslam{

Path::Path(StateVector new_pose, uint32_t k){
    std::shared_ptr<NodePath> first_path_node = std::make_shared<NodePath>();

    first_path_node->pose = new_pose;
    first_path_node->k = k;
    first_path_node->delta_time = std::chrono::nanoseconds(0);
    first_path_node->next_node = nullptr;

    path_root_ = std::make_shared<NodePath>();

    first_path_node->referenced = 1;
    path_root_->next_node = std::move(first_path_node);
    

    path_length_ = 1;
}

Path::Path(const Path &path_to_copy){
    // create shared past from copy
    path_root_ = std::make_shared<NodePath>();
    path_root_->next_node = path_to_copy.path_root_->next_node;
    path_to_copy.path_root_->next_node->referenced++;

    path_length_ = path_to_copy.getLength();
}

Path::~Path(){}

void Path::addPose(StateVector pose, uint32_t k, std::chrono::nanoseconds delta_time){

    std::shared_ptr<NodePath> new_node = std::make_shared<NodePath>();
    new_node->pose = pose;
    new_node->k = k;
    new_node->delta_time = delta_time + path_root_->next_node->delta_time;
    new_node->referenced = 0;
    new_node->next_node = std::move(path_root_->next_node);


    path_root_->next_node = std::move(new_node);
    path_root_->next_node->referenced++;

    path_length_++;
}

uint32_t Path::countLengthOfPath(){

    if (path_root_ == nullptr){
        path_length_ = 0;
        return path_length_;
    }
    path_length_ = 1;
    return countLengthRecurse(path_root_->next_node);
}

uint32_t Path::countLengthRecurse(std::shared_ptr<NodePath> & next){
    path_length_++;
    if(next->next_node == nullptr) return path_length_;
    else return countLengthRecurse(next->next_node);
}

StateVector Path::getPose(uint32_t n_pose){
    // returns specific pose!
    if (path_root_ == nullptr)
        return StateVector::Zero();

    return getPoseNode(n_pose, path_root_)->pose;
}

std::shared_ptr<NodePath> Path::getPoseNode(uint32_t n_pose, std::shared_ptr<NodePath> &next){
    if(next == nullptr) return nullptr;
    else if(next->k == n_pose) return next;
    else return getPoseNode(n_pose, next->next_node);
}

void Path::printAllPathPoses(std::ostream &out){
    for(size_t i = 1 ; i<=path_length_ ; i++){
        auto pose_node = getPoseNode(i, path_root_);

        if (pose_node != nullptr){
             out <<" Pose_" << i << ": " << pose_node->pose.transpose() << "  at address: " <<  std::hex << pose_node << std::dec << std::endl;
        }
        else{
        	out <<"Error: NULL pointer!";
        }
    }
}

} //namespace fastslam
