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
        typedef BMHeap<CT> BMHeap;
        friend class BMHeap;
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
};

// Binary max-heap, organized with floating point keys and with each node containing a value of type CT.
template<typename KT, typename CT>
class BMHeap {
    public:
        typedef HeapNode<CT> HeapNode;
        BMHeap() {}

        bool empty() const {
            return _elements.empty();
        }

        void clear() {
            _elements.clear();
            return;
        }

        // Returns a reference to the top element of the heap
        HeapNode &top() {
            return _elements.front();
        }

        // Removes the top element from the heap. Equivalent to remove(top())
        // Calling on an empty heap is undefined behavior
        void pop() {
            remove(_elements.front());
        }

        // Removes the referenced node from the heap.
        // Calling on an empty heap, or on a node reference that doesn't belong to the heap, is undefined behavior
        void remove(HeapNode &node) {
            std::swap(node, _elements.back());
            _elements.pop_back();
            if (!empty()) reheap_down(node); //node ref now holds the value of the last element. original node value doesnt exist anymore
            return;
        };

        // Change the key of a node and reorder the heap
        void update(HeapNode &node, KT key) {
            node._key = key;
            reheap(node);
            return;
        };

        // Insert element and reorder the heap
        HeapNode &push(KT key, CT content) {
            _elements.push_back(HeapNode(key, content));
            return reheap_up(_elements.back());
        };

        int index_of(const HeapNode &node) const {
            return (int)(&node - _elements.data()); // addr(node) - addr(array) / size
        };

        int index_of_parent(HeapNode &node) {
            double idx = (double)(this->index_of(node));
            return (int)((idx-1)/2.0);
        };

        int index_of_left(HeapNode &node) {
            return 2*index_of(node)+1;
        };

        int index_of_right(HeapNode &node) {
            return 2*index_of(node)+2;
        }

    private:
        std::vector<HeapNode> _elements;
        
        HeapNode &reheap_up(HeapNode &node) {
            HeapNode *curr = &node;
            HeapNode *parent = &_elements[index_of_parent(node)];
            // compare with parent, if key[i]<=key[parent] then its all ok. if key[i]>key[parent] then swap and reheap
            while (curr->_key > parent->_key) {
                std::swap(*curr, *parent);
                curr = parent; // new pointers after swap
                parent = &_elements[index_of_parent(*curr)];
            }
            return *curr;
        }

        void reheap_down(HeapNode &node) {
            HeapNode *curr = &node;
            HeapNode *left;
            HeapNode *right;
            int il = index_of_left(*curr);
            int ir = index_of_right(*curr);
            while (1) {
                left = &_elements[il];
                right = &_elements[ir];
                if (il < _elements.size() && curr->_key < left->_key && left->_key > right->_key) {
                    std::swap(*curr, *left);
                    curr = left;
                } else if (ir < _elements.size() && curr->_key < right->_key && left->_key < right->_key) {
                    std::swap(*curr, *right);
                    curr = right;
                } else {
                    break;
                }
                il = index_of_left(*curr);
                ir = index_of_right(*curr);
            }
            return;
        };
        void reheap(HeapNode &node) {
            if (node._key > _elements[index_of_parent(*node)]._key) {
                reheap_up(node);
                return;
            } else {
                reheap_down(node);
                return;
            }
        }
};
