#include <gtest/gtest.h>

#include "fast_slam/PathList.hpp"
#include <eigen3/Eigen/Core>
#include <chrono>

class PathTester : public ::testing::Test {
protected:
    Eigen::Vector3d initial_pose = {1.0 , 2.0 , 3.0};
    std::unique_ptr<fastslam::Path> path = std::make_unique<fastslam::Path>(initial_pose, 1);

    virtual void SetUp(){
        for(size_t i=2;i<=10; i++){
            std::chrono::nanoseconds time = std::chrono::nanoseconds(10);
            Eigen::Vector3d pose;
            pose << 1.0 * i, 2.0 * i, 3.0 * i;

            path->addPose(pose, i, time);
        }
    }
};

TEST_F(PathTester, pathLength_test){
    auto length = path->getLength();
    EXPECT_EQ(length, 10);
}

TEST_F(PathTester, latestPose_test){
    auto pose = path->getPose();
    EXPECT_EQ(pose(0), 1.0 * 10);
    EXPECT_EQ(pose(1), 2.0 * 10);
    EXPECT_EQ(pose(2), 3.0 * 10);
}

TEST_F(PathTester, specificPose_test){
    auto pose = path->getPose(5);
    EXPECT_EQ(pose(0), 1.0 * 5);
    EXPECT_EQ(pose(1), 2.0 * 5);
    EXPECT_EQ(pose(2), 3.0 * 5);
}

TEST_F(PathTester, copy_test){
    std::unique_ptr<fastslam::Path> path_copy = std::make_unique<fastslam::Path>(*path);

    std::stringstream out, out_copy;
    out << *path;
    out_copy << *path_copy;

    EXPECT_EQ(out.str(), out_copy.str());
}

TEST_F(PathTester, copyAndAdd_test){
    std::unique_ptr<fastslam::Path> path_copy = std::make_unique<fastslam::Path>(*path);

    std::chrono::nanoseconds time = std::chrono::nanoseconds(10);
    Eigen::Vector3d pose;
    uint32_t i = 11;

    // add pose to original path
    pose << 1.0 * i, 2.0 * i, 3.0 * i;
    path->addPose(pose, i, time);

    // add same pose to copied path
    path_copy->addPose(pose, i, time);


    std::stringstream out, out_copy;
    out << *path;
    out_copy << *path_copy;


    for(size_t i=1; i<=11; i++){
        std::string line, line_copy;
        std::getline(out, line);
        std::getline(out_copy, line_copy);
        if(i == 11){
            EXPECT_FALSE(line == line_copy);
        }
        else{
            EXPECT_TRUE(line == line_copy);
        }
    }
}