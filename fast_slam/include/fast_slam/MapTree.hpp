#ifndef FASTSLAM_MAPTREE_HPP
#define FASTSLAM_MAPTREE_HPP

#include <eigen3/Eigen/Core>
#include <vector>
#include <iostream>
#include <string>
#include <memory>
#include "fast_slam/PathList.hpp"


namespace fastslam{
class MapTreeRange;
class OptimizedRangeIterator;

struct Landmark
{
  static unsigned int global_landmark_counter;
  uint32_t landmark_identifier;

  Eigen::Vector2d landmark_pose;
  Eigen::Matrix2d landmark_covariance;

  Landmark()
  {
      global_landmark_counter++;
  }
  ~Landmark(){
      global_landmark_counter--;
  }

};

struct MapNode{
    static uint32_t global_map_node_counter;
    uint32_t key_value;
    std::shared_ptr<MapNode> left;
    std::shared_ptr<MapNode> right;
    std::shared_ptr<Landmark> landmark;
    uint32_t referenced;

    MapNode()
    {
        global_map_node_counter++;
    }
    ~MapNode(){
        global_map_node_counter--;
    }
};

class MapTree
{
    static int map_tree_identifier_counter;
    friend class MapTreeRangeIterator;
    public:
        MapTree();
        MapTree(const MapTree &map_to_copy); 
        ~MapTree();

        
        void insertLandmark(std::shared_ptr<Landmark> &new_landmark_data);
        void correctLandmark(std::shared_ptr<Landmark> &new_landmark_data);
        
        inline std::shared_ptr<Landmark> extractLandmarkNodePointer(uint32_t landmark_identifier) const {return extractLeafNodePointer(map_root_, landmark_identifier)->landmark;}
        inline uint32_t getIdentifier() const {return map_tree_identifier_;}
        inline uint32_t getNLandmarks() const {return n_landmarks_;}
        inline uint32_t getNLayers() const {return n_layers_;}
        inline uint32_t getNnodes() const {return n_nodes_;}
        inline std::shared_ptr<MapNode> getRoot() const {return map_root_;}
        
        MapTreeRange range(uint32_t start_id, uint32_t end_id) const;
        MapTreeRange rangeInclusive(uint32_t start_id, uint32_t end_id) const;
        MapTreeRange rangeExclusive(uint32_t start_id, uint32_t end_id) const;
        MapTreeRange rangeMixed(uint32_t start_id, uint32_t end_id, bool start_inclusive, bool end_inclusive) const;
        OptimizedRangeIterator singleElement(uint32_t landmark_id) const;
        OptimizedRangeIterator smallRange(uint32_t start_id, uint32_t end_id) const;
        OptimizedRangeIterator optimizedEnd() const;
        std::vector<std::shared_ptr<Landmark>> getLandmarksInRange(uint32_t start_id, uint32_t end_id) const;
        size_t countLandmarksInRange(uint32_t start_id, uint32_t end_id) const;
        bool hasLandmarkInRange(uint32_t start_id, uint32_t end_id) const;

        friend std::ostream &operator<<(std::ostream &out, MapTree &obj){
            obj.printAllLandmarkPositions(out);
            return out;
        }


    private:
        void printAllLandmarkPositions(std::ostream &out);
        std::shared_ptr<MapNode> makeNewPath(std::shared_ptr<Landmark> &new_landmark_data, std::shared_ptr<MapNode> &starting_node);
        void creatNewLayers(int needed_n_layers);
        void removeReferenceToSubTree(std::shared_ptr<MapNode> &sub_tree_root);
        std::shared_ptr<MapNode> extractLeafNodePointer(const std::shared_ptr<MapNode> &current_ptr, uint32_t landmark_identifier) const;

        uint32_t map_tree_identifier_;
        uint32_t n_landmarks_;
        uint32_t n_layers_;
        uint32_t n_nodes_;
        std::shared_ptr<MapNode> map_root_;

};

}

#endif