#include "fast_slam/MapTree.hpp"

namespace fastslam{

// can be used to check if number of Landmarks does not grow without bound
unsigned int Landmark::global_landmark_counter;

// can be used to check if number of MapNodes does not grow without bound
unsigned int MapNode::global_map_node_counter; 
int MapTree::map_tree_identifier_counter = 1;

MapTree::MapTree():
        map_tree_identifier_(map_tree_identifier_counter),
        n_landmarks_(0),
        n_layers_(0),
        n_nodes_(0),
        map_root_(nullptr)
        {
    map_tree_identifier_counter++;
}

MapTree::MapTree(const MapTree &map_to_copy){
    map_tree_identifier_ = map_tree_identifier_counter;

    map_tree_identifier_counter++;
    map_root_ = map_to_copy.map_root_;

    // we add new reference for a MapNode and have to increment its reference counter
    if (map_to_copy.map_root_ != nullptr){
        map_to_copy.map_root_->referenced++;
    }
    
    n_landmarks_ = map_to_copy.n_landmarks_;
    n_layers_ = map_to_copy.n_layers_;
    n_nodes_ = map_to_copy.n_nodes_;
}

MapTree::~MapTree(){}

void MapTree::removeReferenceToSubTree(std::shared_ptr<MapNode> &sub_tree_root){
    sub_tree_root.reset();
}

void MapTree::insertLandmark(std::shared_ptr<Landmark> &new_landmark){
    // first landmark
    if (new_landmark->landmark_identifier == 1){ // handle special case
        int need_n_layers = 1;
        if (need_n_layers > n_layers_){
            creatNewLayers(need_n_layers);
        }
    }
    else{
        auto c_tmp = (float) new_landmark->landmark_identifier;
        int need_n_layers = (int)ceil(log2(c_tmp));

        if (need_n_layers > n_layers_){
            creatNewLayers(need_n_layers);
        }
    }

    // get pointer to make unique pointer iterable
    std::shared_ptr<MapNode> *curr = &map_root_;
    size_t i2 = ((*curr)->key_value) / 2;

    /*
    We are filling the tree in the following structure:
                        O(2)                 Level 2
                    /       \
                O(1)          O(3)          Level 1
                /     \       /     \
            O(0)    O(0)   O(0)   O(0)
            LM(1)    LM(2) LM(3)   LM(4)
    */

    for(size_t i = n_layers_; i>1; i--){

        // if identifir is greater than current node go right
        if(new_landmark->landmark_identifier > (*curr)->key_value){
            // if the right node is not null use right as curr
            if((*curr)->right != nullptr)
                curr = &((*curr)->right);
            
            // else create a new node
            else{
                (*curr)->right = std::make_shared<MapNode>();
                (*curr)->right->key_value = (*curr)->key_value + i2;
                (*curr)->right->right = nullptr;
                (*curr)->right->left = nullptr;
                (*curr)->right->landmark = nullptr;
                (*curr)->right->referenced = 1;

                curr = &((*curr)->right);
                n_nodes_++;
            }
        }
        // else go left
        else{
            // same here. If not null go left
            if((*curr)->left != nullptr)
                curr = &((*curr)->left);
            // same here. If null, create new node
            else{
                (*curr)->left = std::make_shared<MapNode>();
                (*curr)->left->key_value = (*curr)->key_value - i2;
                (*curr)->left->right = nullptr;
                (*curr)->left->left = nullptr;
                (*curr)->left->landmark = nullptr;
                (*curr)->left->referenced = 1;

                curr = &((*curr)->left);
                n_nodes_++;
            }
        }
        i2 = i2 / 2;
    }

    // Now we are at the bottom, ready to add a new leaf

    std::shared_ptr<MapNode> new_leaf = std::make_shared<MapNode>();
    new_leaf->key_value = 0;
    new_leaf->left = nullptr;
    new_leaf->right = nullptr;
    new_leaf->referenced = 1;
    new_leaf->landmark = new_landmark;

    // again, if landmark is greater than current key value, go right
    if(new_landmark->landmark_identifier > (*curr)->key_value)
        (*curr)->right = std::move(new_leaf);
    else
        (*curr)->left = std::move(new_leaf);

    n_nodes_++;
    n_landmarks_++;
}

void MapTree::creatNewLayers(int needed_n_layers){
    int missing_layers = needed_n_layers - n_layers_;

    for(size_t i = 1; i<= missing_layers; i++){
        std::shared_ptr<MapNode> newmap_root_node = std::make_shared<MapNode>();
        newmap_root_node->key_value = static_cast<int>(pow(2,(n_layers_ + i) - 1));
        newmap_root_node->right = nullptr;
        newmap_root_node->landmark = nullptr;
        newmap_root_node->referenced = 1;

        // give new node ownership of old root 
        newmap_root_node->left = std::move(map_root_);
        n_nodes_++;

        // move new node to root
        map_root_ = std::move(newmap_root_node);
    }

     n_layers_ = needed_n_layers;
}


std::shared_ptr<MapNode> MapTree::extractLeafNodePointer(std::shared_ptr<MapNode> &current_ptr, uint32_t landmark_identifier){

    if(current_ptr == nullptr){
        std::cout << "got a nullptr someting went wrong! \n";
        return nullptr;
    }
    
    if(current_ptr->key_value == 0)
        return current_ptr;

    // // go right
    if(landmark_identifier > current_ptr->key_value)
        return extractLeafNodePointer(current_ptr->right, landmark_identifier);
    
    // // go left
    else
        return extractLeafNodePointer(current_ptr->left, landmark_identifier);
    

    return nullptr;
}

void MapTree::correctLandmark(std::shared_ptr<Landmark> &new_landmark_data){
    // reassignment should release the original pointer after assignment
    map_root_ = makeNewPath(new_landmark_data, map_root_);
}

std::shared_ptr<MapNode> MapTree::makeNewPath(std::shared_ptr<Landmark> &new_landmark_data, std::shared_ptr<MapNode> &starting_node){
    if(starting_node->key_value > 0){
        std::shared_ptr<MapNode> new_map_node = std::make_shared<MapNode>();
        new_map_node->landmark = nullptr;
        new_map_node->referenced = 1;
        new_map_node->key_value = starting_node->key_value;

        if(new_landmark_data->landmark_identifier > starting_node->key_value){
            // link entire left branch to original subtree since landmark is on the right
            new_map_node->left = starting_node->left;
            if(new_map_node->left != nullptr)
                new_map_node->left->referenced++;

            // now traverse until leaf nodes
            new_map_node->right = makeNewPath(new_landmark_data, starting_node->right);
        }
        else if(new_landmark_data->landmark_identifier <= starting_node->key_value){
            // link entire right branch to original subtree since landmark is on the left
            new_map_node->right = starting_node->right;
            if(new_map_node->right != nullptr)
                new_map_node->right->referenced++;

            // now traverse until leaf nodes
            new_map_node->left = makeNewPath(new_landmark_data, starting_node->left);   
        }
        else{
            std::cout << "error in makeNewPath";
        }
        return new_map_node;
    }
    // now we reached the leaf nodes
    else{
        std::shared_ptr<MapNode> new_leaf_node = std::make_shared<MapNode>();
        new_leaf_node->key_value = 0;
        new_leaf_node->left = nullptr;
        new_leaf_node->right = nullptr;
        new_leaf_node->landmark = new_landmark_data;
        new_leaf_node->referenced = 1;

        return new_leaf_node;
    }
}


void MapTree::printAllLandmarkPositions(std::ostream &out){

    for(size_t i = 1 ; i<=n_landmarks_ ; i++){
        auto leaf = extractLeafNodePointer(map_root_, i);

        if (leaf != nullptr){
             out << "l_" << i <<" pose: "<< leaf->landmark->landmark_pose.transpose() << "  at address: " <<  std::hex << leaf << std::dec << std::endl;
        }
        else{
        	out <<"Error: NULL pointer!";
        }
    }

}


} //namespace fastslam
