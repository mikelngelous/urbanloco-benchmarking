#include <gtest/gtest.h>
#include "robot_communication/data_transfer_queue.hpp"
#include "sensor_data.pb.h"
#include <thread>
#include <chrono>
#include <vector>

namespace robot_communication {
namespace test {

class DataTransferQueueTest : public ::testing::Test {
protected:
    void SetUp() override {
        queue_ = std::make_unique<DataTransferQueue>();
    }

    void TearDown() override {
        if (queue_) {
            queue_.reset();
        }
    }

    std::unique_ptr<DataTransferQueue> queue_;
};

TEST_F(DataTransferQueueTest, BasicEnqueueDequeue) {
    // Basic enqueue/dequeue test
    robot_communication::SensorDataPacket msg;
    msg.set_sequence_id(1);
    msg.set_frame_id("test");

    EXPECT_TRUE(queue_->enqueue(msg));

    robot_communication::SensorDataPacket retrieved_msg;
    EXPECT_TRUE(queue_->dequeue(retrieved_msg));
    EXPECT_EQ(retrieved_msg.sequence_id(), 1);
    EXPECT_EQ(retrieved_msg.frame_id(), "test");
}

TEST_F(DataTransferQueueTest, EmptyQueueDequeue) {
    // Test dequeue on empty queue with timeout
    SensorDataMessage msg;

    auto start_time = std::chrono::steady_clock::now();
    bool result = queue_->dequeue(msg, 100); // 100ms timeout
    auto end_time = std::chrono::steady_clock::now();

    EXPECT_FALSE(result);

    // Verify that timeout worked
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    EXPECT_GE(elapsed.count(), 90); // At least 90ms (allowing some margin)
}

TEST_F(DataTransferQueueTest, MultipleMessages) {
    // Test multiple messages in FIFO order
    std::vector<SensorDataMessage> messages(5);

    // Enqueue 5 messages
    for (int i = 0; i < 5; ++i) {
        messages[i].sequence_id = i;
        messages[i].data = {static_cast<uint8_t>(i)};
        EXPECT_TRUE(queue_->enqueue(messages[i]));
    }

    // Dequeue and verify order
    for (int i = 0; i < 5; ++i) {
        SensorDataMessage retrieved_msg;
        EXPECT_TRUE(queue_->dequeue(retrieved_msg));
        EXPECT_EQ(retrieved_msg.sequence_id, i);
        EXPECT_EQ(retrieved_msg.data[0], i);
    }
}

TEST_F(DataTransferQueueTest, ThreadSafetyProducerConsumer) {
    // Test thread safety with producer/consumer
    const int num_messages = 100;
    std::vector<int> produced_ids;
    std::vector<int> consumed_ids;

    // Producer thread
    std::thread producer([this, &produced_ids, num_messages]() {
        for (int i = 0; i < num_messages; ++i) {
            SensorDataMessage msg;
            msg.sequence_id = i;
            msg.data = {static_cast<uint8_t>(i % 256)};

            if (queue_->enqueue(msg)) {
                produced_ids.push_back(i);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    });

    // Consumer thread
    std::thread consumer([this, &consumed_ids, num_messages]() {
        for (int i = 0; i < num_messages; ++i) {
            SensorDataMessage msg;
            if (queue_->dequeue(msg, 1000)) { // 1 second timeout
                consumed_ids.push_back(msg.sequence_id);
            }
        }
    });

    producer.join();
    consumer.join();

    // Verify all messages were transferred
    EXPECT_EQ(produced_ids.size(), num_messages);
    EXPECT_EQ(consumed_ids.size(), num_messages);
    EXPECT_EQ(produced_ids, consumed_ids);
}

TEST_F(DataTransferQueueTest, QueueSizeTracking) {
    // Test that queue size is tracked correctly
    EXPECT_EQ(queue_->size(), 0);

    // Add messages
    for (int i = 0; i < 3; ++i) {
        SensorDataMessage msg;
        msg.sequence_id = i;
        queue_->enqueue(msg);
    }

    EXPECT_EQ(queue_->size(), 3);

    // Remove one message
    SensorDataMessage msg;
    queue_->dequeue(msg);
    EXPECT_EQ(queue_->size(), 2);

    // Clear remaining
    while (queue_->dequeue(msg, 10)) {
        // Consume all
    }
    EXPECT_EQ(queue_->size(), 0);
}

TEST_F(DataTransferQueueTest, EnqueueTimeout) {
    // Test enqueue with timeout (if implementation supports it)
    SensorDataMessage msg;
    msg.sequence_id = 1;

    // Basic enqueue should succeed
    EXPECT_TRUE(queue_->enqueue(msg, 100));

    // If queue has capacity limits, this would test timeout behavior
    // For now, just verify the method signature works
}

TEST_F(DataTransferQueueTest, ConcurrentProducers) {
    // Test multiple concurrent producers
    const int num_threads = 4;
    const int messages_per_thread = 25;
    std::vector<std::thread> producers;
    std::atomic<int> total_produced{0};

    for (int t = 0; t < num_threads; ++t) {
        producers.emplace_back([this, t, messages_per_thread, &total_produced]() {
            for (int i = 0; i < messages_per_thread; ++i) {
                SensorDataMessage msg;
                msg.sequence_id = t * messages_per_thread + i;
                msg.data = {static_cast<uint8_t>(t), static_cast<uint8_t>(i)};

                if (queue_->enqueue(msg)) {
                    total_produced.fetch_add(1);
                }
            }
        });
    }

    // Wait for all producers
    for (auto& producer : producers) {
        producer.join();
    }

    EXPECT_EQ(total_produced.load(), num_threads * messages_per_thread);
    EXPECT_EQ(queue_->size(), num_threads * messages_per_thread);
}

} // namespace test
} // namespace robot_communication