#include "robot_communication/communication_manager.hpp"
#include <ros/ros.h>

namespace robot_communication {

CommunicationManager::CommunicationManager(const Config& config) 
    : config_(config), io_context_() {
    ROS_INFO("CommunicationManager initialized");
}

CommunicationManager::~CommunicationManager() {
    stop();
}

bool CommunicationManager::start(const ConnectionStatusCallback& status_callback,
                                const CommandCallback& command_callback) {
    status_callback_ = status_callback;
    command_callback_ = command_callback;
    
    socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
    
    // Start IO thread
    io_thread_ = std::make_unique<std::thread>(&CommunicationManager::ioWorkerThread, this);
    
    // Start send thread
    send_thread_ = std::make_unique<std::thread>(&CommunicationManager::sendWorkerThread, this);
    
    // Attempt initial connection
    attemptConnection();
    
    ROS_INFO("CommunicationManager started");
    return true;
}

void CommunicationManager::stop() {
    should_stop_.store(true);
    
    io_context_.stop();
    
    if (io_thread_ && io_thread_->joinable()) {
        io_thread_->join();
    }
    
    if (send_thread_ && send_thread_->joinable()) {
        queue_cv_.notify_all();
        send_thread_->join();
    }
    
    ROS_INFO("CommunicationManager stopped");
}

bool CommunicationManager::sendSensorData(const SensorDataPacket& packet) {
    return serializeAndQueue(packet, 1); // Normal priority
}

bool CommunicationManager::sendRobotStatus(const RobotStatus& status) {
    return serializeAndQueue(status, 0); // High priority
}

void CommunicationManager::ioWorkerThread() {
    while (!should_stop_.load()) {
        try {
            io_context_.run();
        } catch (const std::exception& e) {
            ROS_ERROR("IO worker thread error: %s", e.what());
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void CommunicationManager::sendWorkerThread() {
    while (!should_stop_.load()) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        queue_cv_.wait(lock, [this] { 
            return !send_queue_.empty() || should_stop_.load(); 
        });
        
        if (should_stop_.load()) break;
        
        if (!send_queue_.empty() && is_connected_.load()) {
            auto message = send_queue_.top();
            send_queue_.pop();
            lock.unlock();
            
            // Send message via TCP
            boost::system::error_code ec;
            boost::asio::write(*socket_, boost::asio::buffer(message.data), ec);
            
            if (ec) {
                ROS_WARN("Send failed: %s", ec.message().c_str());
                is_connected_.store(false);
                if (status_callback_) {
                    status_callback_(false);
                }
            } else {
                std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                statistics_.packets_sent++;
                statistics_.bytes_sent += message.data.size();
            }
        }
    }
}

void CommunicationManager::attemptConnection() {
    if (should_stop_.load()) return;
    
    boost::asio::ip::tcp::endpoint endpoint(
        boost::asio::ip::address::from_string(config_.server_host),
        config_.server_port);
    
    socket_->async_connect(endpoint,
        [this](const boost::system::error_code& error) {
            handleConnect(error);
        });
}

void CommunicationManager::handleConnect(const boost::system::error_code& error) {
    if (!error) {
        is_connected_.store(true);
        reconnect_attempts_.store(0);
        
        ROS_INFO("Connected to server %s:%d", 
                 config_.server_host.c_str(), config_.server_port);
        
        if (status_callback_) {
            status_callback_(true);
        }
        
        startReceive();
    } else {
        ROS_WARN("Connection failed: %s", error.message().c_str());
        is_connected_.store(false);
        
        if (status_callback_) {
            status_callback_(false);
        }
        
        // Schedule reconnection
        uint32_t attempts = reconnect_attempts_.fetch_add(1);
        if (attempts < config_.max_reconnect_attempts) {
            reconnect_timer_ = std::make_unique<boost::asio::steady_timer>(io_context_);
            reconnect_timer_->expires_after(
                std::chrono::milliseconds(config_.reconnect_interval_ms));
            
            reconnect_timer_->async_wait([this](const boost::system::error_code&) {
                attemptConnection();
            });
        } else {
            ROS_ERROR("Max reconnection attempts reached");
        }
    }
}

void CommunicationManager::startReceive() {
    // Simplified receive implementation
    // In full version, would handle protobuf message framing
}

bool CommunicationManager::serializeAndQueue(const google::protobuf::Message& message, uint8_t priority) {
    std::string serialized;
    if (!message.SerializeToString(&serialized)) {
        return false;
    }
    
    QueuedMessage queued_msg;
    queued_msg.data.assign(serialized.begin(), serialized.end());
    queued_msg.priority = priority;
    queued_msg.timestamp = std::chrono::steady_clock::now();
    
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        send_queue_.push(queued_msg);
    }
    
    queue_cv_.notify_one();
    return true;
}

CommunicationManager::Statistics CommunicationManager::getStatistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return statistics_;
}

} // namespace robot_communication