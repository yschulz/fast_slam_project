#include <gtest/gtest.h>
#include "fast_slam/MapTree.hpp"
#include "fast_slam/MapTreeIterator.hpp"
#include <eigen3/Eigen/Core>
#include <algorithm>
#include <vector>
#include <set>

/**
 * @brief Comprehensive tests for MapTree range iterators
 * 
 * These tests validate the range iterator functionality for the FastSLAM
 * tree structure, ensuring O(log n + k) complexity and proper handling
 * of all edge cases.
 */

class MapTreeIteratorTester : public ::testing::Test {
protected:
    std::unique_ptr<fastslam::MapTree> map_tree = std::make_unique<fastslam::MapTree>();
    std::vector<uint32_t> inserted_ids;

    virtual void SetUp() {
        // Insert landmarks with gaps to test edge cases
        std::vector<uint32_t> test_ids = {1, 3, 5, 7, 8, 10, 12, 15, 20, 25};
        
        for (uint32_t id : test_ids) {
            std::shared_ptr<fastslam::Landmark> new_lm = std::make_shared<fastslam::Landmark>();
            new_lm->landmark_identifier = id;
            Eigen::Vector2d pose;
            pose << 1.0 * id, 2.0 * id;
            Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();
            new_lm->landmark_pose = pose;
            new_lm->landmark_covariance = cov;
            map_tree->insertLandmark(new_lm);
            inserted_ids.push_back(id);
        }
        
        std::sort(inserted_ids.begin(), inserted_ids.end());
    }

    // Helper function to collect IDs from iterator range
    template<typename Iterator>
    std::vector<uint32_t> collectIds(Iterator begin, Iterator end) {
        std::vector<uint32_t> result;
        for (auto it = begin; it != end; ++it) {
            if (*it) {
                result.push_back((*it)->landmark_identifier);
            }
        }
        return result;
    }
};

// Basic Range Iterator Tests

TEST_F(MapTreeIteratorTester, BasicRangeIteration) {
    // Test normal range [5, 15] inclusive
    auto range = map_tree->range(5, 15);
    auto collected_ids = collectIds(range.begin(), range.end());
    
    std::vector<uint32_t> expected = {5, 7, 8, 10, 12, 15};
    EXPECT_EQ(collected_ids, expected);
}

TEST_F(MapTreeIteratorTester, EmptyRange) {
    // Test empty range (no landmarks in range)
    auto range = map_tree->range(2, 4);
    auto collected_ids = collectIds(range.begin(), range.end());
    
    EXPECT_TRUE(collected_ids.empty());
    EXPECT_TRUE(range.empty());
}

TEST_F(MapTreeIteratorTester, SingleElementRange) {
    // Test single element range
    auto range = map_tree->range(8, 8);
    auto collected_ids = collectIds(range.begin(), range.end());
    
    std::vector<uint32_t> expected = {8};
    EXPECT_EQ(collected_ids, expected);
}

TEST_F(MapTreeIteratorTester, FullRange) {
    // Test full range
    auto range = map_tree->range(1, 25);
    auto collected_ids = collectIds(range.begin(), range.end());
    
    EXPECT_EQ(collected_ids, inserted_ids);
}

TEST_F(MapTreeIteratorTester, BoundaryEdgeCases) {
    // Test boundaries that don't exist
    auto range1 = map_tree->range(0, 30);
    auto collected_ids1 = collectIds(range1.begin(), range1.end());
    EXPECT_EQ(collected_ids1, inserted_ids);

    // Test range starting at non-existent landmark
    auto range2 = map_tree->range(2, 8);
    auto collected_ids2 = collectIds(range2.begin(), range2.end());
    std::vector<uint32_t> expected2 = {3, 5, 7, 8};
    EXPECT_EQ(collected_ids2, expected2);
}

// Boundary Type Tests

TEST_F(MapTreeIteratorTester, InclusiveBoundaries) {
    auto range = map_tree->rangeInclusive(7, 12);
    auto collected_ids = collectIds(range.begin(), range.end());
    
    std::vector<uint32_t> expected = {7, 8, 10, 12};
    EXPECT_EQ(collected_ids, expected);
}

TEST_F(MapTreeIteratorTester, ExclusiveBoundaries) {
    auto range = map_tree->rangeExclusive(7, 12);
    auto collected_ids = collectIds(range.begin(), range.end());
    
    std::vector<uint32_t> expected = {8, 10};
    EXPECT_EQ(collected_ids, expected);
}

TEST_F(MapTreeIteratorTester, MixedBoundaries) {
    // Start inclusive, end exclusive
    auto range1 = map_tree->rangeMixed(7, 12, true, false);
    auto collected_ids1 = collectIds(range1.begin(), range1.end());
    std::vector<uint32_t> expected1 = {7, 8, 10};
    EXPECT_EQ(collected_ids1, expected1);

    // Start exclusive, end inclusive
    auto range2 = map_tree->rangeMixed(7, 12, false, true);
    auto collected_ids2 = collectIds(range2.begin(), range2.end());
    std::vector<uint32_t> expected2 = {8, 10, 12};
    EXPECT_EQ(collected_ids2, expected2);
}

// Iterator Operations Tests

TEST_F(MapTreeIteratorTester, ForwardIteration) {
    auto range = map_tree->range(5, 15);
    auto it = range.begin();
    
    EXPECT_EQ((*it)->landmark_identifier, 5);
    
    ++it;
    EXPECT_EQ((*it)->landmark_identifier, 7);
    
    it++;
    EXPECT_EQ((*it)->landmark_identifier, 8);
    
    ++it;
    EXPECT_EQ((*it)->landmark_identifier, 10);
}

TEST_F(MapTreeIteratorTester, BackwardIteration) {
    auto range = map_tree->range(5, 15);
    auto it = range.end();
    
    --it;
    EXPECT_EQ((*it)->landmark_identifier, 15);
    
    it--;
    EXPECT_EQ((*it)->landmark_identifier, 12);
    
    --it;
    EXPECT_EQ((*it)->landmark_identifier, 10);
}

TEST_F(MapTreeIteratorTester, IteratorComparison) {
    auto range = map_tree->range(5, 15);
    auto it1 = range.begin();
    auto it2 = range.begin();
    auto end_it = range.end();
    
    EXPECT_TRUE(it1 == it2);
    EXPECT_FALSE(it1 != it2);
    EXPECT_FALSE(it1 == end_it);
    EXPECT_TRUE(it1 != end_it);
    
    ++it1;
    EXPECT_FALSE(it1 == it2);
    EXPECT_TRUE(it1 != it2);
}

// STL Algorithm Compatibility Tests

TEST_F(MapTreeIteratorTester, STLAlgorithmCompatibility) {
    auto range = map_tree->range(5, 15);
    
    // Test std::distance
    auto distance = std::distance(range.begin(), range.end());
    EXPECT_EQ(distance, 6); // 5, 7, 8, 10, 12, 15
    
    // Test std::find_if
    auto it = std::find_if(range.begin(), range.end(), 
        [](const std::shared_ptr<fastslam::Landmark>& landmark) {
            return landmark->landmark_identifier == 10;
        });
    
    EXPECT_NE(it, range.end());
    EXPECT_EQ((*it)->landmark_identifier, 10);
}

// Optimized Iterator Tests

TEST_F(MapTreeIteratorTester, SingleElementOptimization) {
    auto it = map_tree->singleElement(8);
    
    EXPECT_EQ((*it)->landmark_identifier, 8);
    
    ++it;
    // Should be at end after incrementing single element
    EXPECT_EQ(it, map_tree->optimizedEnd());
}

TEST_F(MapTreeIteratorTester, SmallRangeOptimization) {
    auto it = map_tree->smallRange(7, 12);
    
    std::vector<uint32_t> collected_ids;
    while (it != map_tree->optimizedEnd()) {
        if (*it) {
            collected_ids.push_back((*it)->landmark_identifier);
        }
        ++it;
    }
    
    std::vector<uint32_t> expected = {7, 8, 10, 12};
    EXPECT_EQ(collected_ids, expected);
}

// Utility Method Tests

TEST_F(MapTreeIteratorTester, GetLandmarksInRange) {
    auto landmarks = map_tree->getLandmarksInRange(7, 12);
    
    EXPECT_EQ(landmarks.size(), 4);
    EXPECT_EQ(landmarks[0]->landmark_identifier, 7);
    EXPECT_EQ(landmarks[1]->landmark_identifier, 8);
    EXPECT_EQ(landmarks[2]->landmark_identifier, 10);
    EXPECT_EQ(landmarks[3]->landmark_identifier, 12);
}

TEST_F(MapTreeIteratorTester, CountLandmarksInRange) {
    auto count1 = map_tree->countLandmarksInRange(5, 15);
    EXPECT_EQ(count1, 6);
    
    auto count2 = map_tree->countLandmarksInRange(2, 4);
    EXPECT_EQ(count2, 0);
    
    auto count3 = map_tree->countLandmarksInRange(8, 8);
    EXPECT_EQ(count3, 1);
}

TEST_F(MapTreeIteratorTester, HasLandmarkInRange) {
    EXPECT_TRUE(map_tree->hasLandmarkInRange(7, 12));
    EXPECT_FALSE(map_tree->hasLandmarkInRange(2, 4));
    EXPECT_TRUE(map_tree->hasLandmarkInRange(8, 8));
    EXPECT_FALSE(map_tree->hasLandmarkInRange(9, 9));
}

// Performance and Edge Case Tests

TEST_F(MapTreeIteratorTester, LargeRangePerformance) {
    // Add more landmarks for performance testing
    std::unique_ptr<fastslam::MapTree> large_tree = std::make_unique<fastslam::MapTree>();
    
    // Insert 1000 landmarks with some gaps
    for (uint32_t i = 1; i <= 1000; i += 2) { // Only odd numbers
        std::shared_ptr<fastslam::Landmark> new_lm = std::make_shared<fastslam::Landmark>();
        new_lm->landmark_identifier = i;
        Eigen::Vector2d pose;
        pose << 1.0 * i, 2.0 * i;
        new_lm->landmark_covariance = Eigen::Matrix2d::Identity();
        new_lm->landmark_pose = pose;
        large_tree->insertLandmark(new_lm);
    }
    
    // Test range iteration performance
    auto range = large_tree->range(100, 200);
    auto start_time = std::chrono::high_resolution_clock::now();
    
    size_t count = 0;
    for (auto& landmark : range) {
        if (landmark) count++;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    EXPECT_EQ(count, 50); // 50 odd numbers from 101 to 199
    
    // Performance should be reasonable (adjust threshold as needed)
    EXPECT_LT(duration.count(), 10000); // Less than 10ms
}

TEST_F(MapTreeIteratorTester, ConcurrentReadAccess) {
    // Test thread safety for read operations
    // This is a basic test - full thread safety testing would require more complex scenarios
    auto range = map_tree->range(1, 25);
    
    std::vector<std::vector<uint32_t>> results(4);
    
    // Simulate concurrent access (though actual concurrency would need std::thread)
    for (int i = 0; i < 4; ++i) {
        auto local_range = map_tree->range(1, 25);
        results[i] = collectIds(local_range.begin(), local_range.end());
    }
    
    // All results should be identical
    for (int i = 1; i < 4; ++i) {
        EXPECT_EQ(results[0], results[i]);
    }
}

TEST_F(MapTreeIteratorTester, InvalidRangeHandling) {
    // Test invalid ranges
    auto range1 = map_tree->range(15, 5); // start > end
    EXPECT_TRUE(range1.empty());
    
    // Test ranges on empty tree
    std::unique_ptr<fastslam::MapTree> empty_tree = std::make_unique<fastslam::MapTree>();
    auto range2 = empty_tree->range(1, 10);
    EXPECT_TRUE(range2.empty());
}

// Copy-on-Write Preservation Tests

TEST_F(MapTreeIteratorTester, CopyOnWritePreservation) {
    // Create a copy of the tree
    std::unique_ptr<fastslam::MapTree> tree_copy = std::make_unique<fastslam::MapTree>(*map_tree);
    
    // Test that both trees return the same ranges
    auto range1 = map_tree->range(5, 15);
    auto range2 = tree_copy->range(5, 15);
    
    auto ids1 = collectIds(range1.begin(), range1.end());
    auto ids2 = collectIds(range2.begin(), range2.end());
    
    EXPECT_EQ(ids1, ids2);
    
    // Verify that iterators are read-only and don't modify shared state
    auto it1 = range1.begin();
    auto it2 = range2.begin();
    
    // Both should point to same landmark data initially
    EXPECT_EQ((*it1)->landmark_identifier, (*it2)->landmark_identifier);
    
    ++it1;
    ++it2;
    
    // Should still be consistent
    EXPECT_EQ((*it1)->landmark_identifier, (*it2)->landmark_identifier);
}