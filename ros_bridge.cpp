#include "ros_bridge.hpp"

ROSBridge::ROSBridge() : connected_(false) {
    client_.clear_access_channels(websocketpp::log::alevel::all);
    client_.init_asio();
    
    client_.set_message_handler([this](auto hdl, auto msg) {
        this->onMessage(hdl, msg);
    });
    client_.set_open_handler([this](auto hdl) {
        this->onOpen(hdl);
    });
    client_.set_close_handler([this](auto hdl) {
        this->onClose(hdl);
    });
}

ROSBridge::~ROSBridge() {
    disconnect();
}

bool ROSBridge::connect(const std::string& uri) {
    std::cout << "Attempting to connect to " << uri << std::endl;
    websocketpp::lib::error_code ec;
    Client::connection_ptr con = client_.get_connection(uri, ec);
    if (ec) {
        std::cout << "Could not create connection: " << ec.message() << std::endl;
        return false;
    }

    client_.connect(con);
    client_thread_ = std::thread([this]() {
        std::cout << "WebSocket thread starting" << std::endl;
        client_.run();
        std::cout << "WebSocket thread ending" << std::endl;
    });
    return true;
}

void ROSBridge::disconnect() {
    if (connected_) {
        client_.close(connection_, websocketpp::close::status::normal, "");
        if (client_thread_.joinable()) {
            client_thread_.join();
        }
        connected_ = false;
    }
}

bool ROSBridge::subscribe(const std::string& topic, const std::string& type,
                         MessageCallback callback) {
    if (!connected_) {
        std::cout << "Cannot subscribe: not connected" << std::endl;
        return false;
    }
    
    json msg = {
        {"op", "subscribe"},
        {"topic", topic},
        {"type", type}
    };
    
    std::cout << "Subscribing to topic: " << topic << " with type: " << type << std::endl;
    std::cout << "Sending subscribe message: " << msg.dump() << std::endl;
    
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_[topic] = callback;
    client_.send(connection_, msg.dump(), websocketpp::frame::opcode::text);
    return true;
}

bool ROSBridge::advertise(const std::string& topic, const std::string& type) {
    if (!connected_) {
        std::cerr << "Cannot advertise: not connected" << std::endl;
        return false;
    }
    
    std::string id = generateId("adv");
    json msg = {
        {"op", "advertise"},
        {"id", id},
        {"topic", topic},
        {"type", type}
    };
    
    if (debug_mode_) {
        std::cout << "Sending advertise message: " << msg.dump(2) << std::endl;
    }
    
    client_.send(connection_, msg.dump(), websocketpp::frame::opcode::text);
    advertised_topics_[topic] = true;  // Mark as advertised immediately
    return true;
}

void ROSBridge::handleAdvertiseResponse(const json& data) {
    if (data.contains("topic") && debug_mode_) {
        std::string topic = data["topic"];
        std::cout << "Received advertise response for topic: " << topic << std::endl;
    }
}

bool ROSBridge::publish(const std::string& topic, const std::string& type,
                       const json& message) {
    if (!connected_) return false;
    
    // Check if topic was advertised
    if (advertised_topics_.find(topic) == advertised_topics_.end() || !advertised_topics_[topic]) {
        if (debug_mode_) {
            std::cerr << "Cannot publish to topic " << topic << ": not advertised" << std::endl;
        }
        return false;
    }
    
    json msg = {
        {"op", "publish"},
        {"topic", topic},
        {"type", type},
        {"msg", message}
    };
    
    if (debug_mode_) {
        std::cout << "Publishing to " << topic << ": " << msg.dump(2) << std::endl;
    }
    
    client_.send(connection_, msg.dump(), websocketpp::frame::opcode::text);
    return true;
}

void ROSBridge::onMessage(websocketpp::connection_hdl hdl, 
                         Client::message_ptr msg) {
    try {
        json data = json::parse(msg->get_payload());
        
        if (debug_mode_) {
            std::cout << "Received message: " << data.dump(2) << std::endl;
        }
        
        if (data.contains("op")) {
            // Track response ID if present
            std::string id = data.value("id", "");
            
            if (data["op"] == "publish") {
                std::string topic = data["topic"];
                
                std::lock_guard<std::mutex> lock(mutex_);
                auto it = callbacks_.find(topic);
                if (it != callbacks_.end()) {
                    it->second(data["msg"]);
                }
            } else if (data["op"] == "advertise") {
                handleAdvertiseResponse(data);
            } else if (!id.empty()) {
                std::cout << "Received response for message " << id << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error parsing message: " << e.what() << std::endl;
    }
}

void ROSBridge::onOpen(websocketpp::connection_hdl hdl) {
    std::cout << "WebSocket connection established" << std::endl;
    connection_ = hdl;
    connected_.store(true);  // Use atomic store
}

void ROSBridge::onClose(websocketpp::connection_hdl hdl) {
    connected_.store(false);  // Use atomic store
}

std::vector<std::string> ROSBridge::getTopics() {
    std::vector<std::string> topics;
    if (!connected_) return topics;
    
    json msg = {
        {"op", "call_service"},
        {"service", "/rosapi/topics"},
        {"args", json::object()}
    };
    
    client_.send(connection_, msg.dump(), websocketpp::frame::opcode::text);
    // Note: This is synchronous, you might want to implement
    // an async version with callbacks
    return topics;
}

bool ROSBridge::waitForConnection(int timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    while (!connected_.load()) {  // Use atomic load
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count() > timeout_ms) {
            std::cout << "Connection timeout after " << timeout_ms << "ms" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
}
