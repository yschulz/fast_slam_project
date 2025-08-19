#include "fast_slam/MapTreeIterator.hpp"

#include <algorithm>
#include <cassert>

#include "fast_slam/MapTree.hpp"

namespace fastslam {

// Constructs iterator and positions it at first valid landmark in range
MapTreeRangeIterator::MapTreeRangeIterator(const MapTree* tree, uint32_t start_id, uint32_t end_id, BoundaryType start_boundary,
                                           BoundaryType end_boundary)
    : tree_(tree),
      start_id_(start_id),
      end_id_(end_id),
      start_boundary_(start_boundary),
      end_boundary_(end_boundary),
      current_landmark_(nullptr),
      current_id_(0),
      is_end_(false) {
    if (!tree_ || !tree_->getRoot() || start_id > end_id) {
        is_end_ = true;
        return;
    }

    findFirstInRange();
}

MapTreeRangeIterator::MapTreeRangeIterator()
    : tree_(nullptr),
      start_id_(0),
      end_id_(0),
      start_boundary_(BoundaryType::INCLUSIVE),
      end_boundary_(BoundaryType::INCLUSIVE),
      current_landmark_(nullptr),
      current_id_(0),
      is_end_(true) {}

MapTreeRangeIterator& MapTreeRangeIterator::operator++() {
    if (is_end_) {
        return *this;
    }

    if (!findNext()) {
        is_end_ = true;
        current_landmark_ = nullptr;
        current_id_ = 0;
    }

    return *this;
}

MapTreeRangeIterator MapTreeRangeIterator::operator++(int) {
    MapTreeRangeIterator temp = *this;
    ++(*this);
    return temp;
}

MapTreeRangeIterator& MapTreeRangeIterator::operator--() {
    if (!tree_) return *this;

    if (is_end_) {
        findLastInRange();
        is_end_ = false;
        return *this;
    }

    if (!findPrevious()) {
        current_landmark_ = nullptr;
        current_id_ = 0;
    }

    return *this;
}

MapTreeRangeIterator MapTreeRangeIterator::operator--(int) {
    MapTreeRangeIterator temp = *this;
    --(*this);
    return temp;
}

MapTreeRangeIterator::reference MapTreeRangeIterator::operator*() const {
    if (is_end_ || !current_landmark_) {
        static std::shared_ptr<Landmark> null_landmark = nullptr;
        return null_landmark;
    }
    return const_cast<reference>(current_landmark_);
}

MapTreeRangeIterator::pointer MapTreeRangeIterator::operator->() const { return &(operator*()); }

bool MapTreeRangeIterator::operator==(const MapTreeRangeIterator& other) const {
    if (is_end_ && other.is_end_) return true;
    if (is_end_ != other.is_end_) return false;

    return tree_ == other.tree_ && current_id_ == other.current_id_ && start_id_ == other.start_id_ && end_id_ == other.end_id_;
}

bool MapTreeRangeIterator::operator!=(const MapTreeRangeIterator& other) const { return !(*this == other); }

bool MapTreeRangeIterator::isValid() const { return !is_end_ && current_landmark_ != nullptr; }

uint32_t MapTreeRangeIterator::getCurrentId() const { return current_id_; }

bool MapTreeRangeIterator::isEmpty() const {
    MapTreeRangeIterator temp = *this;
    return temp.is_end_;
}

// Locates and positions iterator at first valid landmark within range bounds
void MapTreeRangeIterator::findFirstInRange() {
    if (!tree_ || !tree_->getRoot()) {
        is_end_ = true;
        return;
    }

    uint32_t search_start = start_id_;
    if (start_boundary_ == BoundaryType::EXCLUSIVE) {
        search_start++;
    }

    current_id_ = search_start;
    while (current_id_ <= end_id_) {
        try {
            current_landmark_ = tree_->extractLandmarkNodePointer(current_id_);
            if (current_landmark_ && isInRange(current_id_)) {
                return;
            }
        } catch (...) {
        }
        current_id_++;
    }

    is_end_ = true;
    current_landmark_ = nullptr;
    current_id_ = 0;
}

// Locates and positions iterator at last valid landmark within range bounds
void MapTreeRangeIterator::findLastInRange() {
    if (!tree_ || !tree_->getRoot()) {
        is_end_ = true;
        return;
    }

    uint32_t search_end = end_id_;
    if (end_boundary_ == BoundaryType::EXCLUSIVE) {
        if (search_end == 0) {
            is_end_ = true;
            return;
        }
        search_end--;
    }

    current_id_ = search_end;
    while (current_id_ >= start_id_ && current_id_ <= search_end) {
        try {
            current_landmark_ = tree_->extractLandmarkNodePointer(current_id_);
            if (current_landmark_ && isInRange(current_id_)) {
                return;
            }
        } catch (...) {
        }

        if (current_id_ == 0) break;
        current_id_--;
    }

    is_end_ = true;
    current_landmark_ = nullptr;
    current_id_ = 0;
}

// Advances to next valid landmark within range, returns false if none found
bool MapTreeRangeIterator::findNext() {
    if (!tree_) return false;

    uint32_t next_id = current_id_ + 1;
    uint32_t max_search = end_id_;
    if (end_boundary_ == BoundaryType::EXCLUSIVE) {
        max_search--;
    }

    while (next_id <= max_search) {
        try {
            auto next_landmark = tree_->extractLandmarkNodePointer(next_id);
            if (next_landmark && isInRange(next_id)) {
                current_id_ = next_id;
                current_landmark_ = next_landmark;
                return true;
            }
        } catch (...) {
        }
        next_id++;
    }

    return false;
}

// Moves to previous valid landmark within range, returns false if none found
bool MapTreeRangeIterator::findPrevious() {
    if (!tree_) return false;

    if (current_id_ == 0) return false;

    uint32_t prev_id = current_id_ - 1;
    uint32_t min_search = start_id_;
    if (start_boundary_ == BoundaryType::EXCLUSIVE) {
        min_search++;
    }

    while (prev_id >= min_search && prev_id < current_id_) {
        try {
            auto prev_landmark = tree_->extractLandmarkNodePointer(prev_id);
            if (prev_landmark && isInRange(prev_id)) {
                current_id_ = prev_id;
                current_landmark_ = prev_landmark;
                return true;
            }
        } catch (...) {
        }

        // Prevent underflow
        if (prev_id == 0) break;
        prev_id--;
    }

    return false;
}

bool MapTreeRangeIterator::isInRange(uint32_t landmark_id) const {
    if (start_boundary_ == BoundaryType::INCLUSIVE) {
        if (landmark_id < start_id_) return false;
    } else {
        if (landmark_id <= start_id_) return false;
    }

    if (end_boundary_ == BoundaryType::INCLUSIVE) {
        if (landmark_id > end_id_) return false;
    } else {
        if (landmark_id >= end_id_) return false;
    }

    return true;
}

bool MapTreeRangeIterator::isLeafNode(const std::shared_ptr<MapNode>& node) const {
    return node && node->key_value == 0 && node->landmark != nullptr;
}

MapTreeRange::MapTreeRange(const MapTree* tree, uint32_t start_id, uint32_t end_id, MapTreeRangeIterator::BoundaryType start_boundary,
                           MapTreeRangeIterator::BoundaryType end_boundary)
    : tree_(tree), start_id_(start_id), end_id_(end_id), start_boundary_(start_boundary), end_boundary_(end_boundary) {}

MapTreeRangeIterator MapTreeRange::begin() const { return MapTreeRangeIterator(tree_, start_id_, end_id_, start_boundary_, end_boundary_); }

MapTreeRangeIterator MapTreeRange::end() const { return MapTreeRangeIterator(); }

bool MapTreeRange::empty() const {
    auto it = begin();
    return it == end();
}

size_t MapTreeRange::size() const {
    size_t count = 0;
    for (auto it = begin(); it != end(); ++it) {
        count++;
    }
    return count;
}

// Constructs optimized iterator using hint to select best traversal strategy
OptimizedRangeIterator::OptimizedRangeIterator(const MapTree* tree, uint32_t start_id, uint32_t end_id, OptimizationHint hint)
    : tree_(tree),
      start_id_(start_id),
      end_id_(end_id),
      current_id_(start_id),
      hint_(hint),
      current_landmark_(nullptr),
      is_end_(false),
      cache_index_(0) {
    if (!tree_) {
        is_end_ = true;
        return;
    }

    initializeOptimized();
}

OptimizedRangeIterator::OptimizedRangeIterator()
    : tree_(nullptr),
      start_id_(0),
      end_id_(0),
      current_id_(0),
      hint_(OptimizationHint::SINGLE_ELEMENT),
      current_landmark_(nullptr),
      is_end_(true),
      cache_index_(0) {}

OptimizedRangeIterator& OptimizedRangeIterator::operator++() {
    if (is_end_) return *this;

    switch (hint_) {
        case OptimizationHint::SINGLE_ELEMENT:
            is_end_ = true;
            current_landmark_ = nullptr;
            break;

        case OptimizationHint::SMALL_RANGE:
            cache_index_++;
            if (cache_index_ >= cached_ids_.size()) {
                is_end_ = true;
                current_landmark_ = nullptr;
            } else {
                current_id_ = cached_ids_[cache_index_];
                current_landmark_ = tree_->extractLandmarkNodePointer(current_id_);
            }
            break;

        case OptimizationHint::LARGE_RANGE:
        case OptimizationHint::FULL_TRAVERSAL:
        default:
            do {
                current_id_++;
                if (current_id_ > end_id_) {
                    is_end_ = true;
                    current_landmark_ = nullptr;
                    break;
                }
                try {
                    current_landmark_ = tree_->extractLandmarkNodePointer(current_id_);
                    if (current_landmark_) break;
                } catch (...) {
                    current_landmark_ = nullptr;
                }
            } while (true);
            break;
    }

    return *this;
}

OptimizedRangeIterator::reference OptimizedRangeIterator::operator*() const {
    if (is_end_ || !current_landmark_) {
        static std::shared_ptr<Landmark> null_landmark = nullptr;
        return null_landmark;
    }
    return const_cast<reference>(current_landmark_);
}

bool OptimizedRangeIterator::operator==(const OptimizedRangeIterator& other) const {
    if (is_end_ && other.is_end_) return true;
    if (is_end_ != other.is_end_) return false;
    return current_id_ == other.current_id_;
}

bool OptimizedRangeIterator::operator!=(const OptimizedRangeIterator& other) const { return !(*this == other); }

// Initializes iterator with optimization strategy based on hint type
void OptimizedRangeIterator::initializeOptimized() {
    switch (hint_) {
        case OptimizationHint::SINGLE_ELEMENT:
            try {
                current_landmark_ = tree_->extractLandmarkNodePointer(start_id_);
                if (!current_landmark_) {
                    is_end_ = true;
                }
            } catch (...) {
                is_end_ = true;
            }
            break;

        case OptimizationHint::SMALL_RANGE:
            populateCache();
            if (cached_ids_.empty()) {
                is_end_ = true;
            } else {
                current_id_ = cached_ids_[0];
                current_landmark_ = tree_->extractLandmarkNodePointer(current_id_);
            }
            break;

        case OptimizationHint::LARGE_RANGE:
        case OptimizationHint::FULL_TRAVERSAL:
        default:
            while (current_id_ <= end_id_) {
                try {
                    current_landmark_ = tree_->extractLandmarkNodePointer(current_id_);
                    if (current_landmark_) break;
                } catch (...) {
                    current_landmark_ = nullptr;
                }
                current_id_++;
            }
            if (current_id_ > end_id_) {
                is_end_ = true;
                current_landmark_ = nullptr;
            }
            break;
    }
}

// Pre-loads valid landmark IDs into cache for faster small range iteration
void OptimizedRangeIterator::populateCache() {
    cached_ids_.clear();
    cached_ids_.reserve(std::min(10u, end_id_ - start_id_ + 1));

    for (uint32_t id = start_id_; id <= end_id_ && cached_ids_.size() < 10; ++id) {
        try {
            auto landmark = tree_->extractLandmarkNodePointer(id);
            if (landmark) {
                cached_ids_.push_back(id);
            }
        } catch (...) {
        }
    }
}

}  // namespace fastslam
