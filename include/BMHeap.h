#include <cmath>
#include <vector>

template<typename CT>
class HeapNode;

template<typename CT>
class BMHeap;

// Generic node for heap.
template<typename CT>
class HeapNode {
    public:
        friend class BMHeap<CT>;
        CT content;
        float key() {
            return _key;
        }
    private:
        // Private constructor which can only be used by the BMHeap class.
        HeapNode(float key, CT content) {
            this->_key = key;
            this->content = content;
        }
        float _key;
};

// Binary max-heap, organized with floating point keys and with each node containing a value of type CT.
template<typename CT>
class BMHeap {
    public:
        BMHeap() {
            return;
        }

        bool empty() const {
            return _elements.empty();
        }

        // Returns a reference to the top element of the heap
        HeapNode<CT> &top() {
            return _elements.front();
        }

        // Removes the top element from the heap. Equivalent to remove(top())
        // Calling on an empty heap is undefined behavior
        void pop() {
            remove(_elements.front());
        }

        // Removes the referenced node from the heap.
        // Calling on an empty heap, or on a node reference that doesn't belong to the heap, is undefined behavior
        void remove(HeapNode<CT> &node) {
            std::swap(node, _elements.back());
            _elements.pop_back();
            if (!empty()) reheap_down(node); //node ref now holds the value of the last element. original node value doesnt exist anymore
            return;
        };

        // Change the key of a node and reorder the heap
        void update(HeapNode<CT> &node, float key) {
            node._key = key;
            reheap(node);
            return;
        };

        // Insert element and reorder the heap
        HeapNode<CT> &push(float key, CT content) {
            _elements.push_back(HeapNode<CT>(key, content));
            return reheap_up(_elements.back());
        };

        int index_of(const HeapNode<CT> &node) const {
            return (int)(&node - _elements.data()); // addr(node) - addr(array) / size
        };

        int index_of_parent(HeapNode<CT> &node) {
            double idx = (double)(this->index_of(node));
            return (int)((idx-1)/2.0);
        };

        int index_of_left(HeapNode<CT> &node) {
            return 2*index_of(node)+1;
        };

        int index_of_right(HeapNode<CT> &node) {
            return 2*index_of(node)+2;
        }

    private:
        std::vector<HeapNode<CT>> _elements;
        
        HeapNode<CT> &reheap_up(HeapNode<CT> &node) {
            HeapNode<CT> *curr = &node;
            HeapNode<CT> *parent = &_elements[index_of_parent(node)];
            // compare with parent, if key[i]<=key[parent] then its all ok. if key[i]>key[parent] then swap and reheap
            while (curr->_key > parent->_key) {
                std::swap(*curr, *parent);
                curr = parent; // new pointers after swap
                parent = &_elements[index_of_parent(*curr)];
            }
            return *curr;
        }

        void reheap_down(HeapNode<CT> &node) {
            HeapNode<CT> *curr = &node;
            HeapNode<CT> *left;
            HeapNode<CT> *right;
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
        void reheap(HeapNode<CT> &node) {
            if (node._key > _elements[index_of_parent(*node)]._key) {
                reheap_up(node);
                return;
            } else {
                reheap_down(node);
                return;
            }
        }
};
