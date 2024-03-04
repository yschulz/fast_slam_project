#ifndef FASTSLAM_MAPTREE_HPP
#define FASTSLAM_MAPTREE_HPP

#include <eigen3/Eigen/Core>
#include <vector>
#include <iostream>
#include <string>
#include <memory>
#include "fast_slam/PathList.hpp"


namespace fastslam{

struct Landmark
{
  static unsigned int global_landmark_counter; // can be used to check if number of Landmarks does not grow without bound
  uint32_t landmark_identifier; 		/* Landmark identifier */

  Eigen::Vector2d landmark_pose;
  Eigen::Matrix2d landmark_covariance;

  Landmark() //Constructor
  {
      global_landmark_counter++;
  }
  ~Landmark(){//Destructor
      global_landmark_counter--;
  }

};

struct MapNode{
    // all children have to be shared because once we resample we will share data
    // can be used to check if number of MapNodes does not grow without bound
    static uint32_t global_map_node_counter; 
    uint32_t key_value;    /* node identifier */
    std::shared_ptr<MapNode> left;               /* pointer for the left node */
    std::shared_ptr<MapNode> right;              /* pointer for the right node */
    std::shared_ptr<Landmark> landmark;              /* pointer for a Landmark; is used when *left == NULL or *right == NULL */
    uint32_t referenced;  /* how many nodes/paticles points to this node? if zero the node should be deleted! */

    MapNode() //Constructor
    {
        global_map_node_counter++;
    }
    ~MapNode(){//Destructor
        global_map_node_counter--;
    }
};

class MapTree
{
    static int map_tree_identifier_counter;
    public:
        MapTree();
        // copy constructer
        MapTree(const MapTree &map_to_copy); 
        ~MapTree();

        
        void insertLandmark(std::shared_ptr<Landmark> &new_landmark_data);
        void correctLandmark(std::shared_ptr<Landmark> &new_landmark_data);
        
        inline std::shared_ptr<Landmark> extractLandmarkNodePointer(uint32_t landmark_identifier){return extractLeafNodePointer(map_root_, landmark_identifier)->landmark;}
        inline uint32_t getIdentifier() const {return map_tree_identifier_;}
        inline uint32_t getNLandmarks() const {return n_landmarks_;}
        inline uint32_t getNLayers() const {return n_layers_;}
        inline uint32_t getNnodes() const {return n_nodes_;}

        friend std::ostream &operator<<(std::ostream &out, MapTree &obj){
            obj.printAllLandmarkPositions(out);
            return out;
        }

    private:
        void printAllLandmarkPositions(std::ostream &out);
        std::shared_ptr<MapNode> makeNewPath(std::shared_ptr<Landmark> &new_landmark_data, std::shared_ptr<MapNode> &starting_node);
        void creatNewLayers(int needed_n_layers);
        void removeReferenceToSubTree(std::shared_ptr<MapNode> &sub_tree_root);
        std::shared_ptr<MapNode> extractLeafNodePointer(std::shared_ptr<MapNode> &current_ptr, uint32_t landmark_identifier);

        uint32_t map_tree_identifier_;
        uint32_t n_landmarks_;
        uint32_t n_layers_;
        uint32_t n_nodes_;
        std::shared_ptr<MapNode> map_root_;

};

}

#endif