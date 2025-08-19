#ifndef FASTSLAM_MAPTREEITERATOR_HPP
#define FASTSLAM_MAPTREEITERATOR_HPP

#include <iterator>
#include <memory>
#include <stack>
#include <type_traits>

#include "fast_slam/MapTree.hpp"

namespace fastslam {

class MapTree;
struct MapNode;
struct Landmark;

// STL-compatible bidirectional iterator for MapTree ranges
class MapTreeRangeIterator {
public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = std::shared_ptr<Landmark>;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;

    enum class BoundaryType { INCLUSIVE, EXCLUSIVE };

    // Creates iterator for landmark IDs in specified range with boundary control
    MapTreeRangeIterator(const MapTree* tree, uint32_t start_id, uint32_t end_id, BoundaryType start_boundary = BoundaryType::INCLUSIVE,
                         BoundaryType end_boundary = BoundaryType::INCLUSIVE);

    // Creates end iterator for range comparisons
    explicit MapTreeRangeIterator();

    MapTreeRangeIterator& operator++();
    MapTreeRangeIterator operator++(int);
    MapTreeRangeIterator& operator--();
    MapTreeRangeIterator operator--(int);

    reference operator*() const;
    pointer operator->() const;

    bool operator==(const MapTreeRangeIterator& other) const;
    bool operator!=(const MapTreeRangeIterator& other) const;

    bool isValid() const;
    uint32_t getCurrentId() const;
    bool isEmpty() const;

private:
    struct StackEntry {
        std::shared_ptr<MapNode> node;
        bool left_visited;
        bool right_visited;

        StackEntry(std::shared_ptr<MapNode> n) : node(n), left_visited(false), right_visited(false) {}
    };

    const MapTree* tree_;
    uint32_t start_id_;
    uint32_t end_id_;
    BoundaryType start_boundary_;
    BoundaryType end_boundary_;

    std::shared_ptr<Landmark> current_landmark_;
    uint32_t current_id_;
    std::stack<StackEntry> traversal_stack_;
    bool is_end_;

    void initializeForward();
    void initializeBackward();
    bool findNext();
    bool findPrevious();
    void findFirstInRange();
    void findLastInRange();
    bool isInRange(uint32_t landmark_id) const;
    bool isLeafNode(const std::shared_ptr<MapNode>& node) const;
    void pushLeftPath(std::shared_ptr<MapNode> node);
    void pushRightPath(std::shared_ptr<MapNode> node);
};

// Range-based container for STL-style iteration over landmark IDs
class MapTreeRange {
public:
    // Creates range object for for-loop iteration over landmark IDs
    MapTreeRange(const MapTree* tree, uint32_t start_id, uint32_t end_id,
                 MapTreeRangeIterator::BoundaryType start_boundary = MapTreeRangeIterator::BoundaryType::INCLUSIVE,
                 MapTreeRangeIterator::BoundaryType end_boundary = MapTreeRangeIterator::BoundaryType::INCLUSIVE);

    MapTreeRangeIterator begin() const;
    MapTreeRangeIterator end() const;

    bool empty() const;
    size_t size() const;

private:
    const MapTree* tree_;
    uint32_t start_id_;
    uint32_t end_id_;
    MapTreeRangeIterator::BoundaryType start_boundary_;
    MapTreeRangeIterator::BoundaryType end_boundary_;
};

// Optimized iterator for specific access patterns - faster than standard version
class OptimizedRangeIterator {
public:
    enum class OptimizationHint { SINGLE_ELEMENT, SMALL_RANGE, LARGE_RANGE, FULL_TRAVERSAL, SPARSE_ACCESS };

    // Creates iterator optimized for different usage patterns and range sizes
    OptimizedRangeIterator(const MapTree* tree, uint32_t start_id, uint32_t end_id, OptimizationHint hint = OptimizationHint::SMALL_RANGE);

    // Creates end iterator for optimized range comparisons
    OptimizedRangeIterator();
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = std::shared_ptr<Landmark>;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;

    OptimizedRangeIterator& operator++();
    reference operator*() const;
    bool operator==(const OptimizedRangeIterator& other) const;
    bool operator!=(const OptimizedRangeIterator& other) const;

private:
    const MapTree* tree_;
    uint32_t start_id_;
    uint32_t end_id_;
    uint32_t current_id_;
    OptimizationHint hint_;
    std::shared_ptr<Landmark> current_landmark_;
    bool is_end_;

    std::vector<uint32_t> cached_ids_;
    size_t cache_index_;

    void initializeOptimized();
    void populateCache();
};

}  // namespace fastslam

#endif  // FASTSLAM_MAPTREEITERATOR_HPP