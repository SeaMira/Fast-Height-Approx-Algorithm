#include <algorithm>
#include <numeric>

#include "gtest/gtest.h"

#include "BMHeap.h"


class BMHeapTest : public testing::Test {
    protected:
        BMHeapTest() {
            d0_keys = {1.0, 3.0, 44.5, 6.7, 66.4, 9.0, 11.1, 23.3, 11.5, 4.3};
            d0_data = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j'};
        }

        BMHeap<char> heap;
        std::vector<float> d0_keys;
        std::vector<char> d0_data;
};

TEST_F(BMHeapTest, PushAndPop) {
    heap.push(1.0,'a');
    heap.pop();
}

TEST_F(BMHeapTest, InitializedEmpty) {
    EXPECT_TRUE(heap.empty());
}

TEST_F(BMHeapTest, CheckEmpty) {
    EXPECT_TRUE(heap.empty());
    heap.push(1.0,'a');
    EXPECT_FALSE(heap.empty());
    heap.pop();
    EXPECT_TRUE(heap.empty());
}

TEST_F(BMHeapTest, TopReference) {
    heap.push(1.0,'a');
    HeapNode<char> &top = heap.top();

    EXPECT_FLOAT_EQ(top.key(), 1.0);
    EXPECT_EQ(top.content, 'a');

    heap.push(0.5,'b');
    heap.pop();
    top = heap.top();

    EXPECT_FLOAT_EQ(top.key(), 0.5);
    EXPECT_EQ(top.content, 'b');
}


TEST_F(BMHeapTest, ExtractionOrder) {
    for (std::size_t i=0; i<d0_keys.size(); i++) {
        heap.push(d0_keys[i],d0_data[i]);
    }

    // d0 sorted in descending order
    std::vector<int> max_sorted_indices(d0_keys.size());
    std::iota(max_sorted_indices.begin(), max_sorted_indices.end(), 0);
    std::sort(max_sorted_indices.begin(), max_sorted_indices.end(),[&](int A, int B) -> bool {
        return d0_keys[A] > d0_keys[B];
    });

    // Extract one by one. The extraction order should match max_sorted_keys;
    std::vector<float> extracted_keys;
    std::vector<char> extracted_data;
    std::vector<float> expected_keys(d0_keys.size());
    std::vector<char> expected_data(d0_keys.size());
    HeapNode<char> &top = heap.top();
    for (std::size_t i=0; i<d0_keys.size(); i++) {
        top = heap.top();
        extracted_keys.push_back(top.key());
        extracted_data.push_back(top.content);
        heap.pop();
        expected_keys[i] = d0_keys[max_sorted_indices[i]];
        expected_data[i] = d0_data[max_sorted_indices[i]];
    }

    EXPECT_EQ(extracted_keys, expected_keys);
    EXPECT_EQ(extracted_data, expected_data);
}

int main(int argc, char**argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}