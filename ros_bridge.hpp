#pragma once

// Add these defines to fix compatibility with newer Boost versions
#define ASIO_STANDALONE
#define _WEBSOCKETPP_CPP11_INTERNAL_
#include <boost/version.hpp>
#if BOOST_VERSION >= 106600
#define _WEBSOCKETPP_ASIO_HPP
#include <boost/asio/version.hpp>
#endif

#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <nlohmann/json.hpp>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <future>
#include <atomic>  // Add this include

using json = nlohmann::json;

class ROSBridge {
public:
    using MessageCallback = std::function<void(const json&)>;
    
    ROSBridge();
    ~ROSBridge();

    bool connect(const std::string& uri);
    void disconnect();
    
    bool subscribe(const std::string& topic, const std::string& type, 
                  MessageCallback callback);
    bool unsubscribe(const std::string& topic);
    
    bool publish(const std::string& topic, const std::string& type, 
                const json& message);
    
    bool advertise(const std::string& topic, const std::string& type);
    
    std::vector<std::string> getTopics();
    
    bool waitForConnection(int timeout_ms = 5000);
    bool isConnected() const { return connected_; }

private:
    using Client = websocketpp::client<websocketpp::config::asio_client>;
    
    void onMessage(websocketpp::connection_hdl hdl, 
                   Client::message_ptr msg);
    void onOpen(websocketpp::connection_hdl hdl);
    void onClose(websocketpp::connection_hdl hdl);
    
    Client client_;
    websocketpp::connection_hdl connection_;
    std::mutex mutex_;
    std::map<std::string, MessageCallback> callbacks_;
    std::atomic<bool> connected_;  // Changed to atomic
    
    std::thread client_thread_;
    std::map<std::string, bool> advertised_topics_;
    std::atomic<bool> debug_mode_ = false;  // Changed to atomic
    
    void handleAdvertiseResponse(const json& data);
    
    int next_id_ = 0;
    std::string generateId(const std::string& prefix) {
        return prefix + "_" + std::to_string(next_id_++);
    }
};
