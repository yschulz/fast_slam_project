#ifndef FASTSLAM_PATH_H
#define FASTSLAM_PATH_H

#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include "fast_slam/Helper.hpp"

namespace fastslam{

struct NodePath {
    StateVector pose;
    uint32_t k;
    std::shared_ptr<NodePath> next_node;
    std::chrono::nanoseconds delta_time;
    uint32_t referenced;
};

class Path{
    public:
        
        Path(StateVector pose, uint32_t k);
        Path(const Path &path_to_copy); // copy constructor
        ~Path();

        void addPose(StateVector new_pose, uint32_t k, std::chrono::nanoseconds delta_time);
        uint32_t countLengthOfPath();

        StateVector getPose(uint32_t n_pose);
        inline StateVector getPose(){return path_root_->next_node->pose;}
        inline uint32_t getLength() const {return path_length_;}

        friend std::ostream &operator<<(std::ostream &out, Path &obj){
            obj.printAllPathPoses(out);
            return out;
        }

    private:
        void printAllPathPoses(std::ostream &out);
        uint32_t countLengthRecurse(std::shared_ptr<NodePath> & next);
        std::shared_ptr<NodePath> getPoseNode(uint32_t n_pose, std::shared_ptr<NodePath> &next);

        std::shared_ptr<NodePath> path_root_;
        uint32_t path_length_;
};

} //namespace fastslam

#endif
