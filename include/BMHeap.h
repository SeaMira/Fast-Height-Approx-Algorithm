#include <cmath>
#include <vector>

template<typename KT, typename CT>
class HeapNode;

template<typename KT, typename CT>
class BMHeap;

// Generic node for heap.
template<typename KT, typename CT>
class HeapNode {
    public:
        friend class BMHeap<KT, CT>;
        CT content;
        KT key() {
            return _key;
        }
    private:
        // Private constructor which can only be used by the BMHeap class.
        HeapNode(KT key, CT content) {
            this->_key = key;
            this->content = content;
        }
        KT _key;
        ssize_t heap_position = -1;
};

// Binary max-heap, organized with floating point keys and with each node containing a value of type CT.
template<typename KT, typename CT>
class BMHeap {
    public:
        BMHeap() {}

        bool empty() const {
            return heap.empty();
        }

        void clear() {
            permanent_nodes.clear();
            heap.clear();
            return;
        }

        // Returns a reference to the top element of the heap.
        HeapNode<KT, CT> &top() {
            return permanent_nodes.at(heap.front());
        }

        // Removes the top element from the heap. Equivalent to remove(top())
        // Calling on an empty heap is undefined behavior
        void pop() {
            remove(permanent_nodes.at(heap.front()));
        }

        // Removes the referenced node from the heap.
        // Calling on an empty heap, or on a node reference that doesn't belong to the heap, is undefined behavior.
        void remove(HeapNode<KT, CT> &node) {
            ssize_t node_permanent_index = heap.at(node.heap_position);
            ssize_t backnode_permanent_index = heap.back();
            ssize_t node_new_pos = heap.size() - 1;
            ssize_t backnode_new_pos = node.heap_position;

            heap.at(node_new_pos) = node_permanent_index;
            heap.at(backnode_new_pos) = backnode_permanent_index;

            heap.pop_back();
            node.heap_position = -1;
            HeapNode<KT, CT> &backnode = permanent_nodes.at(backnode_permanent_index);
            backnode.heap_position = backnode_new_pos;
            if (!empty()) reheap_down(backnode); // backnode is not at the back anymore
            return;
        };

        // Change the key of a node and reorder the heap
        void update(HeapNode<KT, CT> &node, KT key) {
            node._key = key;
            reheap(node);
            return;
        };

        // Insert element and reorder the heap
        HeapNode<KT, CT> &push(KT key, CT content) {
            permanent_nodes.push_back(HeapNode<KT, CT>(key, content));
            HeapNode<KT, CT> &new_node = permanent_nodes.back();
            ssize_t new_node_permanent_index = permanent_nodes.size() - 1;
            heap.push_back(new_node_permanent_index);
            ssize_t new_heap_position = heap.size() - 1;
            new_node.heap_position = new_heap_position;

            reheap_up(new_node);
            return new_node;
        };

        // find the permanent index of a node
        ssize_t index_of(const HeapNode<KT, CT> &node) const {
            ptrdiff_t pos = std::distance(permanent_nodes.begin(), &node);
            return pos; // addr(node) - addr(array) / size
        };

        int heap_pos_of_parent(ssize_t pos) {
            return (int)((((float)pos)-1)/2.0);
        };

        int heap_pos_of_left(ssize_t pos) {
            return 2 * pos + 1;
        };

        int heap_pos_of_right(ssize_t pos) {
            return 2 * pos + 2;
        }

    private:
        std::vector<HeapNode<KT, CT>> permanent_nodes; // Vector for storing nodes in a permanent address. Nodes are never deleted.
        std::vector<ssize_t> heap; // Vector rearranging pointers to said nodes in heap shape
        
        void reheap_up(HeapNode<KT, CT> &node) {
            HeapNode<KT, CT> *parent = &(permanent_nodes.at(heap.at(heap_pos_of_parent(node.heap_position))));
            // compare with parent, if key[i]<=key[parent] then its all ok. if key[i]>key[parent] then swap and reheap
            while (node._key > parent->_key) {
                ssize_t node_permanent_index = heap.at(node.heap_position);
                ssize_t parent_permanent_index = heap.at(parent->heap_position);
                ssize_t node_new_pos = parent->heap_position;
                ssize_t parent_new_pos = node.heap_position;

                heap.at(node_new_pos) = node_permanent_index;
                heap.at(parent_new_pos) = parent_permanent_index;
                node.heap_position = node_new_pos;
                parent->heap_position = parent_new_pos;

                parent = &(permanent_nodes.at(heap.at(heap_pos_of_parent(node_new_pos)))); // new parent after moving up
            }
            return;
        }

        void reheap_down(HeapNode<KT, CT> &node) {
            while (1) {
                ssize_t left_pos = heap_pos_of_left(node.heap_position);
                ssize_t right_pos = heap_pos_of_right(node.heap_position);
                HeapNode<KT, CT> *left_node;
                HeapNode<KT, CT> *right_node;
                if (left_pos < heap.size()) left_node = &(permanent_nodes.at(heap.at(left_pos)));
                if (right_pos < heap.size()) right_node = &(permanent_nodes.at(heap.at(right_pos)));
                
                if (left_pos < heap.size() && node._key < left_node->_key && left_node->_key > right_node->_key) {
                    ssize_t node_permanent_index = heap.at(node.heap_position);
                    ssize_t left_permanent_index = heap.at(left_pos);
                    ssize_t node_new_pos = left_pos;
                    ssize_t left_new_pos = node.heap_position;

                    heap.at(node_new_pos) = node_permanent_index;
                    heap.at(left_new_pos) = left_permanent_index;
                    node.heap_position = node_new_pos;
                    left_node->heap_position = left_new_pos;
                } else if (right_pos < heap.size() && node._key < right_node->_key && left_node->_key < right_node->_key) {
                    ssize_t node_permanent_index = heap.at(node.heap_position);
                    ssize_t right_permanent_index = heap.at(right_pos);
                    ssize_t node_new_pos = right_pos;
                    ssize_t right_new_pos = node.heap_position;

                    heap.at(node_new_pos) = node_permanent_index;
                    heap.at(right_new_pos) = right_permanent_index;
                    node.heap_position = node_new_pos;
                    right_node->heap_position = right_new_pos;
                } else {
                    break;
                }
            }
            return;
        };
        void reheap(HeapNode<KT, CT> &node) {
            HeapNode<KT, CT> &parent = permanent_nodes.at(heap.at(heap_pos_of_parent(node.heap_position)));
            if (node._key > parent._key) {
                reheap_up(node);
                return;
            } else {
                reheap_down(node);
                return;
            }
        }
};
