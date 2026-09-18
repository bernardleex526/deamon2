/**
 * @file android_comm.hpp
 * @brief Android UDP通讯模块
 */

#ifndef ANDROID_COMM_HPP
#define ANDROID_COMM_HPP

#include "common_types.hpp"
#include <thread>
#include <sstream>
#include <iomanip>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#ifndef SOCKET
#define SOCKET int
#endif
#ifndef INVALID_SOCKET
#define INVALID_SOCKET -1
#endif
#ifndef SOCKET_ERROR
#define SOCKET_ERROR -1
#endif
#define closesocket close

class AndroidCommManager {
public:
    using LogCallback = std::function<void(const std::string&)>;
    
    AndroidCommManager(SharedState& state) : state_(state) {}
    ~AndroidCommManager() { stop(); }
    
    void setLogCallback(LogCallback cb) { log_callback_ = std::move(cb); }
    void start();
    void stop();
    void sendScanData();

private:
    SharedState& state_;
    SOCKET udp_send_socket_ = INVALID_SOCKET;
    SOCKET udp_recv_socket_ = INVALID_SOCKET;
    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    
    std::mutex client_mutex_;
    std::string client_ip_;
    bool client_connected_ = false;
    
    LogCallback log_callback_;
    
    void log(const std::string& msg) { if (log_callback_) log_callback_(msg); }
    void receiveLoop();
    void parseCommand(const std::string& json);
};

#endif // ANDROID_COMM_HPP
