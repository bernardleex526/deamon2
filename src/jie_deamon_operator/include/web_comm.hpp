/**
 * @file web_comm.hpp
 * @brief Web通讯模块 - HTTP和WebSocket服务器
 */

#ifndef WEB_COMM_HPP
#define WEB_COMM_HPP

#include "common_types.hpp"
#define CPPHTTPLIB_NO_EXCEPTIONS
#undef CPPHTTPLIB_OPENSSL_SUPPORT
#include "httplib.h"

#include <thread>
#include <set>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <ifaddrs.h>

#ifndef SOCKET
#define SOCKET int
#endif
#ifndef INVALID_SOCKET
#define INVALID_SOCKET -1
#endif
#define closesocket close

class WebCommManager {
public:
    using LogCallback = std::function<void(const std::string&)>;
    using DirectCmdCallback = std::function<void(double, double, double)>;
    using ActionCmdCallback = std::function<void(const std::string&)>;

    WebCommManager(SharedState& state) : state_(state) {}
    ~WebCommManager() { stop(); }

    void setLogCallback(LogCallback cb) { log_callback_ = std::move(cb); }
    void setDirectCmdCallback(DirectCmdCallback cb) { direct_cmd_callback_ = std::move(cb); }
    void setActionCmdCallback(ActionCmdCallback cb) { action_cmd_callback_ = std::move(cb); }
    void setWebRoot(const std::string& root) { web_root_ = root; }

    void autoDetectWebRoot(const std::vector<std::string>& paths) {
        if (!web_root_.empty()) return;
        for (const auto& path : paths) {
            std::ifstream test(path + "/index.html");
            if (test.good()) { web_root_ = path; break; }
        }
    }

    std::string getLocalIP();
    void start();
    void stop();
    void broadcastData();

private:
    SharedState& state_;
    std::string web_root_;
    std::atomic<bool> running_{false};

    std::unique_ptr<httplib::Server> http_server_;
    std::thread http_thread_;
    SOCKET ws_server_socket_ = INVALID_SOCKET;
    std::thread ws_thread_;
    std::mutex ws_clients_mutex_;
    std::set<int> ws_clients_;
    LogCallback log_callback_;
    DirectCmdCallback direct_cmd_callback_;
    ActionCmdCallback action_cmd_callback_;

    void log(const std::string& msg) { if (log_callback_) log_callback_(msg); }
    void startWebSocketServer();
    void wsAcceptLoop();
    void handleWebSocketClient(int fd);
    void handleWebSocketMessage(const std::string& message);
    std::string computeWebSocketAcceptKey(const std::string& key);
    bool recvWebSocketMessage(int fd, std::string& message);
    bool sendWebSocketMessage(int fd, const std::string& message);
    void broadcastToWebSockets(const std::string& message);
};

#endif // WEB_COMM_HPP
