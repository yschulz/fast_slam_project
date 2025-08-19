/**
 * @file range_iterator_demo.cpp
 * @brief Demonstration of FastSLAM MapTree range iterator usage patterns
 * 
 * This file demonstrates various usage patterns for the range iterators,
 * showing how they integrate with the FastSLAM tree structure and
 * provide efficient O(log n + k) range queries.
 */

#include "fast_slam/MapTree.hpp"
#include "fast_slam/MapTreeIterator.hpp"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <chrono>
#include <iomanip>

namespace fastslam_demo {

/**
 * @brief Create a sample MapTree with landmarks for demonstration
 */
std::unique_ptr<fastslam::MapTree> createSampleTree() {
    auto tree = std::make_unique<fastslam::MapTree>();
    
    // Insert landmarks at various positions to demonstrate range queries
    std::vector<uint32_t> landmark_ids = {
        1, 5, 8, 12, 15, 18, 22, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100
    };
    
    for (uint32_t id : landmark_ids) {
        auto landmark = std::make_shared<fastslam::Landmark>();
        landmark->landmark_identifier = id;
        
        // Position landmarks in a grid pattern for realistic scenario
        double x = (id % 10) * 2.0;
        double y = (id / 10) * 2.0;
        landmark->landmark_pose = Eigen::Vector2d(x, y);
        landmark->landmark_covariance = Eigen::Matrix2d::Identity() * 0.1;
        
        tree->insertLandmark(landmark);
    }
    
    return tree;
}

/**
 * @brief Demonstrate basic range iteration
 */
void demonstrateBasicRangeIteration(const fastslam::MapTree& tree) {
    std::cout << "\n=== Basic Range Iteration Demo ===\n";
    
    // Example 1: Simple range query
    std::cout << "\n1. Landmarks in range [15, 35]:\n";
    auto range = tree.range(15, 35);
    for (const auto& landmark : range) {
        if (landmark) {
            std::cout << "  ID: " << std::setw(3) << landmark->landmark_identifier 
                      << ", Position: (" << std::setprecision(1) << std::fixed 
                      << landmark->landmark_pose.x() << ", " 
                      << landmark->landmark_pose.y() << ")\n";
        }
    }
    
    // Example 2: Count landmarks in range
    auto count = tree.countLandmarksInRange(20, 50);
    std::cout << "\n2. Number of landmarks in range [20, 50]: " << count << "\n";
    
    // Example 3: Check if range has any landmarks
    bool has_landmarks = tree.hasLandmarkInRange(75, 85);
    std::cout << "\n3. Range [75, 85] has landmarks: " << (has_landmarks ? "Yes" : "No") << "\n";
}

/**
 * @brief Demonstrate boundary type variations
 */
void demonstrateBoundaryTypes(const fastslam::MapTree& tree) {
    std::cout << "\n=== Boundary Type Variations Demo ===\n";
    
    uint32_t start_id = 22, end_id = 40;
    
    // Inclusive boundaries
    std::cout << "\n1. Range [" << start_id << ", " << end_id << "] (inclusive):\n";
    auto inclusive_range = tree.rangeInclusive(start_id, end_id);
    std::cout << "   IDs: ";
    for (const auto& landmark : inclusive_range) {
        if (landmark) {
            std::cout << landmark->landmark_identifier << " ";
        }
    }
    std::cout << "\n";
    
    // Exclusive boundaries
    std::cout << "\n2. Range (" << start_id << ", " << end_id << ") (exclusive):\n";
    auto exclusive_range = tree.rangeExclusive(start_id, end_id);
    std::cout << "   IDs: ";
    for (const auto& landmark : exclusive_range) {
        if (landmark) {
            std::cout << landmark->landmark_identifier << " ";
        }
    }
    std::cout << "\n";
    
    // Mixed boundaries
    std::cout << "\n3. Range [" << start_id << ", " << end_id << ") (start inclusive, end exclusive):\n";
    auto mixed_range = tree.rangeMixed(start_id, end_id, true, false);
    std::cout << "   IDs: ";
    for (const auto& landmark : mixed_range) {
        if (landmark) {
            std::cout << landmark->landmark_identifier << " ";
        }
    }
    std::cout << "\n";
}

/**
 * @brief Demonstrate optimized iterator patterns
 */
void demonstrateOptimizedPatterns(const fastslam::MapTree& tree) {
    std::cout << "\n=== Optimized Iterator Patterns Demo ===\n";
    
    // Single element access
    std::cout << "\n1. Single element access (ID: 25):\n";
    auto single_it = tree.singleElement(25);
    if (*single_it) {
        std::cout << "   Found landmark " << (*single_it)->landmark_identifier 
                  << " at position (" << (*single_it)->landmark_pose.x() 
                  << ", " << (*single_it)->landmark_pose.y() << ")\n";
    }
    
    // Small range optimization
    std::cout << "\n2. Small range optimization [18, 30]:\n";
    auto small_range_it = tree.smallRange(18, 30);
    std::cout << "   IDs found: ";
    while (small_range_it != fastslam::OptimizedRangeIterator(nullptr, 0, 0, 
                             fastslam::OptimizedRangeIterator::OptimizationHint::SMALL_RANGE)) {
        if (*small_range_it) {
            std::cout << (*small_range_it)->landmark_identifier << " ";
        }
        ++small_range_it;
    }
    std::cout << "\n";
    
    // Full traversal
    std::cout << "\n3. Full tree traversal (first 10 landmarks):\n";
    auto full_it = tree.fullTraversal();
    int count = 0;
    std::cout << "   IDs: ";
    while (count < 10 && full_it != fastslam::OptimizedRangeIterator(nullptr, 0, 0, 
                                      fastslam::OptimizedRangeIterator::OptimizationHint::FULL_TRAVERSAL)) {
        if (*full_it) {
            std::cout << (*full_it)->landmark_identifier << " ";
            count++;
        }
        ++full_it;
    }
    std::cout << "\n";
}

/**
 * @brief Demonstrate STL algorithm integration
 */
void demonstrateSTLIntegration(const fastslam::MapTree& tree) {
    std::cout << "\n=== STL Algorithm Integration Demo ===\n";
    
    auto range = tree.range(20, 60);
    
    // Use std::find_if to locate specific landmark
    auto it = std::find_if(range.begin(), range.end(),
        [](const std::shared_ptr<fastslam::Landmark>& landmark) {
            return landmark && landmark->landmark_pose.x() > 4.0;
        });
    
    if (it != range.end() && *it) {
        std::cout << "\n1. First landmark with x > 4.0:\n";
        std::cout << "   ID: " << (*it)->landmark_identifier 
                  << ", Position: (" << (*it)->landmark_pose.x() 
                  << ", " << (*it)->landmark_pose.y() << ")\n";
    }
    
    // Use std::count_if to count landmarks matching criteria
    auto high_y_count = std::count_if(range.begin(), range.end(),
        [](const std::shared_ptr<fastslam::Landmark>& landmark) {
            return landmark && landmark->landmark_pose.y() >= 4.0;
        });
    
    std::cout << "\n2. Landmarks with y >= 4.0 in range [20, 60]: " << high_y_count << "\n";
    
    // Use std::distance to measure range size
    auto range_size = std::distance(range.begin(), range.end());
    std::cout << "\n3. Total landmarks in range [20, 60]: " << range_size << "\n";
}

/**
 * @brief Demonstrate bidirectional iteration
 */
void demonstrateBidirectionalIteration(const fastslam::MapTree& tree) {
    std::cout << "\n=== Bidirectional Iteration Demo ===\n";
    
    auto range = tree.range(25, 50);
    
    // Forward iteration
    std::cout << "\n1. Forward iteration:\n   ";
    for (auto it = range.begin(); it != range.end(); ++it) {
        if (*it) {
            std::cout << (*it)->landmark_identifier << " -> ";
        }
    }
    std::cout << "END\n";
    
    // Reverse iteration
    std::cout << "\n2. Reverse iteration:\n   ";
    auto it = range.end();
    std::cout << "END";
    while (it != range.begin()) {
        --it;
        if (*it) {
            std::cout << " <- " << (*it)->landmark_identifier;
        }
    }
    std::cout << "\n";
    
    // Mixed iteration pattern
    std::cout << "\n3. Mixed iteration (forward then backward):\n";
    auto forward_it = range.begin();
    if (forward_it != range.end() && *forward_it) {
        std::cout << "   First: " << (*forward_it)->landmark_identifier << "\n";
        
        ++forward_it;
        if (forward_it != range.end() && *forward_it) {
            std::cout << "   Second: " << (*forward_it)->landmark_identifier << "\n";
            
            --forward_it;  // Go back to first
            if (*forward_it) {
                std::cout << "   Back to first: " << (*forward_it)->landmark_identifier << "\n";
            }
        }
    }
}

/**
 * @brief Performance comparison between different access methods
 */
void performanceComparison(const fastslam::MapTree& tree) {
    std::cout << "\n=== Performance Comparison Demo ===\n";
    
    const uint32_t start_id = 20, end_id = 80;
    const int iterations = 1000;
    
    // Method 1: Range iterator
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        auto range = tree.range(start_id, end_id);
        size_t count = 0;
        for (const auto& landmark : range) {
            if (landmark) count++;
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto range_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    // Method 2: Individual extractions
    start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        size_t count = 0;
        for (uint32_t id = start_id; id <= end_id; ++id) {
            try {
                auto landmark = tree.extractLandmarkNodePointer(id);
                if (landmark) count++;
            } catch (...) {
                // Landmark not found
            }
        }
    }
    end_time = std::chrono::high_resolution_clock::now();
    auto individual_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    std::cout << "\nPerformance comparison (" << iterations << " iterations):\n";
    std::cout << "  Range Iterator: " << range_duration.count() << " µs\n";
    std::cout << "  Individual Extraction: " << individual_duration.count() << " µs\n";
    std::cout << "  Speedup: " << std::setprecision(2) 
              << (double)individual_duration.count() / range_duration.count() << "x\n";
}

/**
 * @brief Demonstrate copy-on-write preservation
 */
void demonstrateCopyOnWrite(const fastslam::MapTree& original_tree) {
    std::cout << "\n=== Copy-on-Write Preservation Demo ===\n";
    
    // Create a copy
    fastslam::MapTree tree_copy(original_tree);
    
    // Show that both trees have the same range content
    std::cout << "\n1. Range [30, 50] in original tree:\n   ";
    auto orig_range = original_tree.range(30, 50);
    for (const auto& landmark : orig_range) {
        if (landmark) {
            std::cout << landmark->landmark_identifier << " ";
        }
    }
    
    std::cout << "\n\n2. Range [30, 50] in copied tree:\n   ";
    auto copy_range = tree_copy.range(30, 50);
    for (const auto& landmark : copy_range) {
        if (landmark) {
            std::cout << landmark->landmark_identifier << " ";
        }
    }
    
    std::cout << "\n\n3. Both ranges should be identical (preserves shared state)\n";
    
    // Verify that iterators don't modify shared state
    auto orig_it = orig_range.begin();
    auto copy_it = copy_range.begin();
    
    if (*orig_it && *copy_it) {
        std::cout << "   Original first element ID: " << (*orig_it)->landmark_identifier << "\n";
        std::cout << "   Copy first element ID: " << (*copy_it)->landmark_identifier << "\n";
        std::cout << "   Memory addresses " << ((*orig_it).get() == (*copy_it).get() ? "same" : "different") 
                  << " (should be same due to COW)\n";
    }
}

} // namespace fastslam_demo

/**
 * @brief Main demonstration function
 */
int main() {
    std::cout << "FastSLAM MapTree Range Iterator Demonstration\n";
    std::cout << "============================================\n";
    
    // Create sample tree
    auto tree = fastslam_demo::createSampleTree();
    
    std::cout << "\nCreated tree with " << tree->getNLandmarks() 
              << " landmarks across " << tree->getNLayers() << " layers\n";
    
    // Run demonstrations
    fastslam_demo::demonstrateBasicRangeIteration(*tree);
    fastslam_demo::demonstrateBoundaryTypes(*tree);
    fastslam_demo::demonstrateOptimizedPatterns(*tree);
    fastslam_demo::demonstrateSTLIntegration(*tree);
    fastslam_demo::demonstrateBidirectionalIteration(*tree);
    fastslam_demo::performanceComparison(*tree);
    fastslam_demo::demonstrateCopyOnWrite(*tree);
    
    std::cout << "\n=== Demo Complete ===\n";
    std::cout << "The range iterators provide efficient O(log n + k) access\n";
    std::cout << "to landmarks in the FastSLAM tree while preserving all\n";
    std::cout << "copy-on-write semantics and thread safety properties.\n";
    
    return 0;
}

/**
 * Usage examples for different scenarios:
 * 
 * // Nearby landmarks for sensor processing
 * auto nearby = tree.range(current_id - 5, current_id + 5);
 * for (const auto& landmark : nearby) {
 *     processSensorMeasurement(landmark);
 * }
 * 
 * // Batch processing with boundaries
 * auto batch = tree.rangeExclusive(start_batch, end_batch);
 * std::vector<LandmarkUpdate> updates;
 * for (const auto& landmark : batch) {
 *     updates.push_back(processLandmark(landmark));
 * }
 * 
 * // Performance-critical single access
 * auto single = tree.singleElement(target_id);
 * if (*single) {
 *     return processCriticalLandmark(*single);
 * }
 * 
 * // Optimized small range with caching
 * auto small = tree.smallRange(id1, id2);
 * while (small != tree.smallRange(0, 0)) { // End condition
 *     processOptimized(*small);
 *     ++small;
 * }
 * 
 * // Full tree analysis
 * auto full = tree.fullTraversal();
 * AnalysisStats stats;
 * while (full != tree.fullTraversal().end()) {
 *     stats.update(*full);
 *     ++full;
 * }
 */