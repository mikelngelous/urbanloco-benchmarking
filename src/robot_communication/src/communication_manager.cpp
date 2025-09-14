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
    ROS_INFO("Initializing CommunicationManager");

    try {
        status_callback_ = status_callback;
        command_callback_ = command_callback;
        socket_ = std::make_unique<boost::asio::ip::tcp::socket>(io_context_);
        io_thread_ = std::make_unique<std::thread>(&CommunicationManager::ioWorkerThread, this);
        send_thread_ = std::make_unique<std::thread>(&CommunicationManager::sendWorkerThread, this);
        attemptConnection();

    } catch (const std::exception& e) {
        ROS_ERROR("Failed to start CommunicationManager: %s", e.what());
        return false;
    }

    ROS_INFO("CommunicationManager started successfully");
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
    ROS_DEBUG("IO worker thread started");
    while (!should_stop_.load()) {
        try {
            auto handlers_executed = io_context_.run();
            if (handlers_executed == 0) {
                io_context_.reset();
            }
        } catch (const std::exception& e) {
            ROS_ERROR("IO worker thread error: %s", e.what());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_DEBUG("IO worker thread exiting");
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
            
            // Send message via TCP with length prefix framing
            boost::system::error_code ec;
            
            // Create length-prefixed frame: [4 bytes length][protobuf data]
            uint32_t payload_size = static_cast<uint32_t>(message.data.size());
            uint32_t network_size = htonl(payload_size);  // Convert to network byte order
            
            std::vector<boost::asio::const_buffer> buffers;
            buffers.push_back(boost::asio::buffer(&network_size, sizeof(network_size)));
            buffers.push_back(boost::asio::buffer(message.data));
            
            boost::asio::write(*socket_, buffers, ec);
            
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
    if (should_stop_.load()) {
        return;
    }

    ROS_DEBUG("Attempting TCP connection to %s:%d", config_.server_host.c_str(), config_.server_port);

    // Convert localhost to IP address for boost::asio
    std::string host = config_.server_host;
    if (host == "localhost") {
        host = "127.0.0.1";
    }

    try {
        // Use TCP resolver for hostname resolution
        boost::asio::ip::tcp::endpoint endpoint;
        try {
            boost::asio::ip::tcp::resolver resolver(io_context_);
            boost::asio::ip::tcp::resolver::results_type results = resolver.resolve(host, std::to_string(config_.server_port));

            if (results.empty()) {
                throw std::runtime_error("Failed to resolve hostname: " + host);
            }

            endpoint = *results.begin();
        } catch (const boost::system::system_error& resolve_error) {
            ROS_ERROR("DNS resolution failed for '%s': %s", host.c_str(), resolve_error.what());
            throw;
        } catch (const std::exception& resolve_exception) {
            ROS_ERROR("Exception during DNS resolution for '%s': %s", host.c_str(), resolve_exception.what());
            throw;
        }

        try {
            socket_->async_connect(endpoint,
            [this](const boost::system::error_code& error) {
                handleConnect(error);
            });
        } catch (const boost::system::system_error& connect_error) {
            ROS_ERROR("Connection attempt failed: %s", connect_error.what());
            throw;
        } catch (const std::exception& connect_exception) {
            ROS_ERROR("Connection exception: %s", connect_exception.what());
            throw;
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Connection failed: %s", e.what());
        is_connected_.store(false);
        if (status_callback_) {
            status_callback_(false);
        }
    }
}

void CommunicationManager::handleConnect(const boost::system::error_code& error) {
    if (!error) {
        ROS_INFO("TCP connection established to %s:%d",
                 config_.server_host.c_str(), config_.server_port);
        is_connected_.store(true);
        reconnect_attempts_.store(0);

        if (status_callback_) {
            status_callback_(true);
        }

        startReceive();

    } else {
        ROS_WARN("TCP connection failed: %s", error.message().c_str());
        is_connected_.store(false);

        if (status_callback_) {
            status_callback_(false);
        }
        
        // Schedule reconnection
        uint32_t attempts = reconnect_attempts_.fetch_add(1);
        ROS_DEBUG("Reconnection attempt %u/%u", attempts + 1, config_.max_reconnect_attempts);

        if (attempts < config_.max_reconnect_attempts) {
            reconnect_timer_ = std::make_unique<boost::asio::steady_timer>(io_context_);
            reconnect_timer_->expires_after(
                std::chrono::milliseconds(config_.reconnect_interval_ms));

            reconnect_timer_->async_wait([this](const boost::system::error_code& timer_error) {
                if (!timer_error) {
                    attemptConnection();
                }
            });
        } else {
            ROS_ERROR("Max reconnection attempts (%u) reached", config_.max_reconnect_attempts);
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