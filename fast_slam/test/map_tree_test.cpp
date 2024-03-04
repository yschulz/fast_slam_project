#include <gtest/gtest.h>

#include "fast_slam/MapTree.hpp"
#include <eigen3/Eigen/Core>

/*
Balanced binary tree of map with 10 landmarks should look like this:

                                    8
                            /               \
                4                            12
        /               \                   /               
        2               6                10    
    /       \       /       \          /   
    1       3       5       7          9
   /  \   /  \    /  \     /  \       /  \ 
   1  2   3  4    5   6   7   8       9  10
*/

class MapTreeTester : public ::testing::Test {
protected:
    std::unique_ptr<fastslam::MapTree> map_tree = std::make_unique<fastslam::MapTree>();

    virtual void SetUp(){
        for(size_t i=1;i<=10; i++){
            std::shared_ptr<fastslam::Landmark> new_lm = std::make_shared<fastslam::Landmark>();
            new_lm->landmark_identifier = i;
            Eigen::Vector2d pose;
            pose << 1.0 * i, 2.0 * i;
            Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();
            new_lm->landmark_pose = pose;
            new_lm->landmark_covariance = cov;
            map_tree->insertLandmark(new_lm);
        }
    }
};

TEST_F(MapTreeTester, nLandmarks_test){
    auto n_landmarks = map_tree->getNLandmarks();
    EXPECT_EQ(n_landmarks, 10);
}

TEST_F(MapTreeTester, nLayers_test){
    auto n_layers = map_tree->getNLayers();
    EXPECT_EQ(n_layers, 4);
}

TEST_F(MapTreeTester, nNodes_test){
    auto n_nodes = map_tree->getNnodes();
    EXPECT_EQ(n_nodes, 21);
}

TEST_F(MapTreeTester, extractLandmark_test){

    // test first
    auto lm_back = map_tree->extractLandmarkNodePointer(1);
    
    EXPECT_DOUBLE_EQ(lm_back->landmark_pose(0), 1.0 * 1);
    EXPECT_DOUBLE_EQ(lm_back->landmark_pose(1), 2.0 * 1);

    // test last
    lm_back = map_tree->extractLandmarkNodePointer(10);
    
    EXPECT_DOUBLE_EQ(lm_back->landmark_pose(0), 1.0 * 10);
    EXPECT_DOUBLE_EQ(lm_back->landmark_pose(1), 2.0 * 10);

    // test middle odd
    lm_back = map_tree->extractLandmarkNodePointer(5);
    
    EXPECT_DOUBLE_EQ(lm_back->landmark_pose(0), 1.0 * 5);
    EXPECT_DOUBLE_EQ(lm_back->landmark_pose(1), 2.0 * 5);

    // test middle even
    lm_back = map_tree->extractLandmarkNodePointer(8);
    
    EXPECT_DOUBLE_EQ(lm_back->landmark_pose(0), 1.0 * 8);
    EXPECT_DOUBLE_EQ(lm_back->landmark_pose(1), 2.0 * 8);
}

TEST_F(MapTreeTester, copy_test){
    std::unique_ptr<fastslam::MapTree> map_tree_copy = std::make_unique<fastslam::MapTree>(*map_tree);

    // prints contents and addresses of leaf nodes
    // since we want the resource to be shared a copy here means same addresses for all leafs
    std::stringstream out, out_copy;
    out << *map_tree;
    out_copy << *map_tree_copy;

    EXPECT_EQ(out.str(), out_copy.str());
}

TEST_F(MapTreeTester, copyAndChange_test){
    std::unique_ptr<fastslam::MapTree> map_tree_copy = std::make_unique<fastslam::MapTree>(*map_tree);

    // correct the 10th landmark
    std::shared_ptr<fastslam::Landmark> new_lm = std::make_shared<fastslam::Landmark>();
    new_lm->landmark_identifier = 10;
    Eigen::Vector2d pose;
    pose << 1.0 , 2.0;
    Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();
    new_lm->landmark_pose = pose;
    new_lm->landmark_covariance = cov;

    map_tree_copy->correctLandmark(new_lm);


    std::stringstream out, out_copy;
    out << *map_tree;
    out_copy << *map_tree_copy;


    for(size_t i=1; i<=10; i++){
        std::string line, line_copy;
        std::getline(out, line);
        std::getline(out_copy, line_copy);
        if(i == 10){
            EXPECT_FALSE(line == line_copy);
        }
        else{
            EXPECT_TRUE(line == line_copy);
        }
    }
}