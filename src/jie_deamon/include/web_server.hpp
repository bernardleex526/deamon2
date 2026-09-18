/**
 * @file web_server.hpp
 * @brief 简易HTTP和WebSocket服务器实现
 * 
 * 用于雷达可视化Web界面，仅需标准库依赖
 */

#ifndef WEB_SERVER_HPP
#define WEB_SERVER_HPP

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <sstream>
#include <fstream>
#include <cstring>
#include <algorithm>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <openssl/sha.h>

namespace webserver {

// Base64编码表
static const char* BASE64_CHARS = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Base64编码
inline std::string base64_encode(const unsigned char* data, size_t len) {
    std::string result;
    int i = 0;
    unsigned char arr3[3], arr4[4];

    while (len--) {
        arr3[i++] = *(data++);
        if (i == 3) {
            arr4[0] = (arr3[0] & 0xfc) >> 2;
            arr4[1] = ((arr3[0] & 0x03) << 4) + ((arr3[1] & 0xf0) >> 4);
            arr4[2] = ((arr3[1] & 0x0f) << 2) + ((arr3[2] & 0xc0) >> 6);
            arr4[3] = arr3[2] & 0x3f;
            for (i = 0; i < 4; i++) result += BASE64_CHARS[arr4[i]];
            i = 0;
        }
    }

    if (i) {
        for (int j = i; j < 3; j++) arr3[j] = '\0';
        arr4[0] = (arr3[0] & 0xfc) >> 2;
        arr4[1] = ((arr3[0] & 0x03) << 4) + ((arr3[1] & 0xf0) >> 4);
        arr4[2] = ((arr3[1] & 0x0f) << 2) + ((arr3[2] & 0xc0) >> 6);
        for (int j = 0; j < i + 1; j++) result += BASE64_CHARS[arr4[j]];
        while (i++ < 3) result += '=';
    }
    return result;
}

// 计算WebSocket Accept Key
inline std::string compute_accept_key(const std::string& key) {
    std::string magic = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    unsigned char hash[SHA_DIGEST_LENGTH];
    SHA1(reinterpret_cast<const unsigned char*>(magic.c_str()), magic.size(), hash);
    return base64_encode(hash, SHA_DIGEST_LENGTH);
}

// WebSocket客户端连接
struct WSClient {
    int fd;
    std::string ip;
    bool connected;
    
    WSClient(int f, const std::string& i) : fd(f), ip(i), connected(true) {}
};

/**
 * @class SimpleWebServer
 * @brief 简易HTTP + WebSocket服务器
 */
class SimpleWebServer {
public:
    using MessageHandler = std::function<void(const std::string&, std::shared_ptr<WSClient>)>;
    
    SimpleWebServer(int http_port = 8080, int ws_port = 8890)
        : http_port_(http_port), ws_port_(ws_port), running_(false) {}
    
    ~SimpleWebServer() { stop(); }
    
    // 设置静态文件目录
    void set_static_dir(const std::string& dir) { static_dir_ = dir; }
    
    // 设置WebSocket消息处理器
    void set_message_handler(MessageHandler handler) { message_handler_ = handler; }
    
    // 启动服务器
    bool start() {
        if (running_) return true;
        
        // 创建HTTP socket
        http_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (http_socket_ < 0) return false;
        
        int opt = 1;
        setsockopt(http_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(http_port_);
        
        if (bind(http_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(http_socket_);
            return false;
        }
        
        listen(http_socket_, 10);
        
        // 创建WebSocket socket
        ws_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (ws_socket_ < 0) {
            close(http_socket_);
            return false;
        }
        
        setsockopt(ws_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        addr.sin_port = htons(ws_port_);
        if (bind(ws_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(http_socket_);
            close(ws_socket_);
            return false;
        }
        
        listen(ws_socket_, 10);
        
        running_ = true;
        http_thread_ = std::thread(&SimpleWebServer::http_loop, this);
        ws_thread_ = std::thread(&SimpleWebServer::ws_loop, this);
        
        return true;
    }
    
    // 停止服务器
    void stop() {
        running_ = false;
        
        if (http_socket_ >= 0) { close(http_socket_); http_socket_ = -1; }
        if (ws_socket_ >= 0) { close(ws_socket_); ws_socket_ = -1; }
        
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (auto& client : ws_clients_) {
                if (client->fd >= 0) close(client->fd);
            }
            ws_clients_.clear();
        }
        
        if (http_thread_.joinable()) http_thread_.join();
        if (ws_thread_.joinable()) ws_thread_.join();
    }
    
    // 广播消息到所有WebSocket客户端
    void broadcast(const std::string& message) {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        std::vector<std::shared_ptr<WSClient>> dead_clients;
        
        for (auto& client : ws_clients_) {
            if (!client->connected) {
                dead_clients.push_back(client);
                continue;
            }
            if (!send_ws_message(client->fd, message)) {
                client->connected = false;
                dead_clients.push_back(client);
            }
        }
        
        // 清理断开的连接
        for (auto& dead : dead_clients) {
            if (dead->fd >= 0) close(dead->fd);
            ws_clients_.erase(
                std::remove(ws_clients_.begin(), ws_clients_.end(), dead),
                ws_clients_.end()
            );
        }
    }
    
    // 获取连接的客户端数量
    size_t client_count() const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        return ws_clients_.size();
    }

private:
    int http_port_, ws_port_;
    int http_socket_ = -1, ws_socket_ = -1;
    std::atomic<bool> running_;
    std::thread http_thread_, ws_thread_;
    std::string static_dir_;
    MessageHandler message_handler_;
    
    mutable std::mutex clients_mutex_;
    std::vector<std::shared_ptr<WSClient>> ws_clients_;
    
    // HTTP服务循环
    void http_loop() {
        while (running_) {
            struct pollfd pfd = {http_socket_, POLLIN, 0};
            if (poll(&pfd, 1, 100) <= 0) continue;
            
            struct sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(http_socket_, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd < 0) continue;
            
            // 设置超时
            struct timeval tv = {5, 0};
            setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            
            handle_http_request(client_fd);
            close(client_fd);
        }
    }
    
    // 处理HTTP请求
    void handle_http_request(int fd) {
        char buffer[4096];
        int n = recv(fd, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) return;
        buffer[n] = '\0';
        
        std::string request(buffer);
        std::string path = "/";
        
        // 解析请求路径
        size_t start = request.find("GET ");
        if (start != std::string::npos) {
            start += 4;
            size_t end = request.find(" ", start);
            if (end != std::string::npos) {
                path = request.substr(start, end - start);
            }
        }
        
        // 默认首页
        if (path == "/") path = "/index.html";
        
        // 安全检查
        if (path.find("..") != std::string::npos) {
            send_http_response(fd, 403, "text/plain", "Forbidden");
            return;
        }
        
        // 读取文件
        std::string filepath = static_dir_ + path;
        std::ifstream file(filepath, std::ios::binary);
        if (!file) {
            send_http_response(fd, 404, "text/plain", "Not Found");
            return;
        }
        
        std::stringstream ss;
        ss << file.rdbuf();
        std::string content = ss.str();
        
        // 确定MIME类型
        std::string mime = "text/plain";
        if (path.find(".html") != std::string::npos) mime = "text/html; charset=utf-8";
        else if (path.find(".css") != std::string::npos) mime = "text/css; charset=utf-8";
        else if (path.find(".js") != std::string::npos) mime = "application/javascript; charset=utf-8";
        else if (path.find(".json") != std::string::npos) mime = "application/json";
        else if (path.find(".png") != std::string::npos) mime = "image/png";
        else if (path.find(".jpg") != std::string::npos) mime = "image/jpeg";
        else if (path.find(".svg") != std::string::npos) mime = "image/svg+xml";
        
        send_http_response(fd, 200, mime, content);
    }
    
    // 发送HTTP响应
    void send_http_response(int fd, int code, const std::string& mime, const std::string& body) {
        std::string status = (code == 200) ? "OK" : (code == 404) ? "Not Found" : "Error";
        std::ostringstream oss;
        oss << "HTTP/1.1 " << code << " " << status << "\r\n"
            << "Content-Type: " << mime << "\r\n"
            << "Content-Length: " << body.size() << "\r\n"
            << "Access-Control-Allow-Origin: *\r\n"
            << "Connection: close\r\n\r\n"
            << body;
        
        std::string response = oss.str();
        send(fd, response.c_str(), response.size(), 0);
    }
    
    // WebSocket服务循环
    void ws_loop() {
        while (running_) {
            struct pollfd pfd = {ws_socket_, POLLIN, 0};
            if (poll(&pfd, 1, 100) <= 0) continue;
            
            struct sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(ws_socket_, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd < 0) continue;
            
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, ip, INET_ADDRSTRLEN);
            
            // 启动客户端处理线程
            std::thread(&SimpleWebServer::handle_ws_client, this, client_fd, std::string(ip)).detach();
        }
    }
    
    // 处理WebSocket客户端连接
    void handle_ws_client(int fd, std::string ip) {
        // 接收握手请求
        char buffer[4096];
        int n = recv(fd, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) { close(fd); return; }
        buffer[n] = '\0';
        
        std::string request(buffer);
        
        // 提取Sec-WebSocket-Key
        std::string ws_key;
        size_t key_pos = request.find("Sec-WebSocket-Key:");
        if (key_pos != std::string::npos) {
            key_pos += 18;
            while (key_pos < request.size() && request[key_pos] == ' ') key_pos++;
            size_t end = request.find("\r\n", key_pos);
            if (end != std::string::npos) {
                ws_key = request.substr(key_pos, end - key_pos);
            }
        }
        
        if (ws_key.empty()) { close(fd); return; }
        
        // 发送握手响应
        std::string accept_key = compute_accept_key(ws_key);
        std::ostringstream oss;
        oss << "HTTP/1.1 101 Switching Protocols\r\n"
            << "Upgrade: websocket\r\n"
            << "Connection: Upgrade\r\n"
            << "Sec-WebSocket-Accept: " << accept_key << "\r\n\r\n";
        
        std::string response = oss.str();
        send(fd, response.c_str(), response.size(), 0);
        
        // 添加到客户端列表
        auto client = std::make_shared<WSClient>(fd, ip);
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            ws_clients_.push_back(client);
        }
        
        // 接收消息循环
        while (running_ && client->connected) {
            struct pollfd pfd = {fd, POLLIN, 0};
            if (poll(&pfd, 1, 100) <= 0) continue;
            
            std::string message;
            if (!recv_ws_message(fd, message)) {
                client->connected = false;
                break;
            }
            
            if (!message.empty() && message_handler_) {
                message_handler_(message, client);
            }
        }
        
        // 清理
        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            ws_clients_.erase(
                std::remove(ws_clients_.begin(), ws_clients_.end(), client),
                ws_clients_.end()
            );
        }
        close(fd);
    }
    
    // 接收WebSocket消息
    bool recv_ws_message(int fd, std::string& message) {
        unsigned char header[2];
        if (recv(fd, header, 2, 0) != 2) return false;
        
        bool fin = header[0] & 0x80;
        int opcode = header[0] & 0x0F;
        bool masked = header[1] & 0x80;
        uint64_t payload_len = header[1] & 0x7F;
        
        // 关闭帧
        if (opcode == 0x08) return false;
        
        // 扩展长度
        if (payload_len == 126) {
            unsigned char ext[2];
            if (recv(fd, ext, 2, 0) != 2) return false;
            payload_len = (ext[0] << 8) | ext[1];
        } else if (payload_len == 127) {
            unsigned char ext[8];
            if (recv(fd, ext, 8, 0) != 8) return false;
            payload_len = 0;
            for (int i = 0; i < 8; i++) {
                payload_len = (payload_len << 8) | ext[i];
            }
        }
        
        // 掩码密钥
        unsigned char mask[4] = {0};
        if (masked) {
            if (recv(fd, mask, 4, 0) != 4) return false;
        }
        
        // 读取数据
        if (payload_len > 0 && payload_len < 65536) {
            std::vector<char> data(payload_len);
            size_t received = 0;
            while (received < payload_len) {
                int n = recv(fd, data.data() + received, payload_len - received, 0);
                if (n <= 0) return false;
                received += n;
            }
            
            // 解码
            if (masked) {
                for (size_t i = 0; i < payload_len; i++) {
                    data[i] ^= mask[i % 4];
                }
            }
            
            message.assign(data.begin(), data.end());
        }
        
        (void)fin;  // 暂不处理分片
        return true;
    }
    
    // 发送WebSocket消息
    bool send_ws_message(int fd, const std::string& message) {
        std::vector<unsigned char> frame;
        
        // 帧头：FIN + 文本帧
        frame.push_back(0x81);
        
        size_t len = message.size();
        if (len < 126) {
            frame.push_back(static_cast<unsigned char>(len));
        } else if (len < 65536) {
            frame.push_back(126);
            frame.push_back((len >> 8) & 0xFF);
            frame.push_back(len & 0xFF);
        } else {
            frame.push_back(127);
            for (int i = 7; i >= 0; i--) {
                frame.push_back((len >> (8 * i)) & 0xFF);
            }
        }
        
        // 数据
        frame.insert(frame.end(), message.begin(), message.end());
        
        return send(fd, frame.data(), frame.size(), MSG_NOSIGNAL) == (ssize_t)frame.size();
    }
};

} // namespace webserver

#endif // WEB_SERVER_HPP
