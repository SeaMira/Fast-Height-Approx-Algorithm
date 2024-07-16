#include <cmath>
#include <vector>
#include <stdexcept>
#include <iostream>

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
        KT key() const {
            return _key;
        }
    private:
        // Private constructor which can only be used by the BMHeap class.
        HeapNode(KT key, CT content) : _key(key), content(content) {}
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
        }

        // Returns a reference to the top element of the heap.
        HeapNode<KT, CT>& top() {
            if (empty()) {
                throw std::out_of_range("Heap is empty");
            }
            return permanent_nodes.at(heap.front());
        }

        // Removes the top element from the heap. Equivalent to remove(top())
        // Calling on an empty heap is undefined behavior
        void pop() {
            if (empty()) {
                throw std::out_of_range("Heap is empty");
            }
            remove(permanent_nodes.at(heap.front()));
        }

        // Removes the referenced node from the heap.
        // Calling on an empty heap, or on a node reference that doesn't belong to the heap, is undefined behavior.
        void remove(HeapNode<KT, CT>& node) {
            if (empty()) {
                throw std::out_of_range("Heap is empty");
            }
            ssize_t node_permanent_index = heap.at(node.heap_position);
            ssize_t backnode_permanent_index = heap.back();
            ssize_t node_new_pos = heap.size() - 1;
            ssize_t backnode_new_pos = node.heap_position;

            heap.at(node_new_pos) = node_permanent_index;
            heap.at(backnode_new_pos) = backnode_permanent_index;

            heap.pop_back();
            node.heap_position = -1;
            if (!heap.empty()) {
                HeapNode<KT, CT>& backnode = permanent_nodes.at(backnode_permanent_index);
                backnode.heap_position = backnode_new_pos;
                reheap_down(backnode); // backnode is not at the back anymore
            }
        }

        // Change the key of a node and reorder the heap
        void update(HeapNode<KT, CT>& node, KT key) {
            node._key = key;
            reheap(node);
        }

        // Insert element and reorder the heap
        HeapNode<KT, CT>& push(KT key, CT content) {
            permanent_nodes.push_back(HeapNode<KT, CT>(key, content));
            HeapNode<KT, CT>& new_node = permanent_nodes.back();
            ssize_t new_node_permanent_index = permanent_nodes.size() - 1;
            heap.push_back(new_node_permanent_index);
            ssize_t new_heap_position = heap.size() - 1;
            new_node.heap_position = new_heap_position;

            reheap_up(new_node);
            return new_node;
        }

        // Find the permanent index of a node
        ssize_t index_of(const HeapNode<KT, CT>& node) const {
            return &node - permanent_nodes.data();
        }

    private:
        std::vector<HeapNode<KT, CT>> permanent_nodes; // Vector for storing nodes in a permanent address. Nodes are never deleted.
        std::vector<ssize_t> heap; // Vector rearranging pointers to said nodes in heap shape

        int heap_pos_of_parent(ssize_t pos) const {
            return (pos - 1) / 2;
        }

        int heap_pos_of_left(ssize_t pos) const {
            return 2 * pos + 1;
        }

        int heap_pos_of_right(ssize_t pos) const {
            return 2 * pos + 2;
        }

        void reheap_up(HeapNode<KT, CT>& node) {
            while (node.heap_position > 0) {
                ssize_t parent_pos = heap_pos_of_parent(node.heap_position);
                HeapNode<KT, CT>& parent = permanent_nodes.at(heap.at(parent_pos));
                if (node._key <= parent._key) {
                    break;
                }
                swap_nodes(node, parent);
            }
        }

        void reheap_down(HeapNode<KT, CT>& node) {
            while (true) {
                ssize_t left_pos = heap_pos_of_left(node.heap_position);
                ssize_t right_pos = heap_pos_of_right(node.heap_position);
                ssize_t largest_pos = node.heap_position;

                if (left_pos < heap.size() && permanent_nodes.at(heap.at(left_pos))._key > permanent_nodes.at(heap.at(largest_pos))._key) {
                    largest_pos = left_pos;
                }
                if (right_pos < heap.size() && permanent_nodes.at(heap.at(right_pos))._key > permanent_nodes.at(heap.at(largest_pos))._key) {
                    largest_pos = right_pos;
                }
                if (largest_pos == node.heap_position) {
                    break;
                }
                swap_nodes(node, permanent_nodes.at(heap.at(largest_pos)));
            }
        }

        void reheap(HeapNode<KT, CT>& node) {
            if (node.heap_position > 0) {
                ssize_t parent_pos = heap_pos_of_parent(node.heap_position);
                if (node._key > permanent_nodes.at(heap.at(parent_pos))._key) {
                    reheap_up(node);
                    return;
                }
            }
            reheap_down(node);
        }

        void swap_nodes(HeapNode<KT, CT>& node1, HeapNode<KT, CT>& node2) {
            ssize_t node1_pos = node1.heap_position;
            ssize_t node2_pos = node2.heap_position;

            std::swap(heap[node1_pos], heap[node2_pos]);
            std::swap(node1.heap_position, node2.heap_position);
        }
};
